/*
    Run YOLO detection inference on videos

    Usage:
        $ ./detect --bin weights.bin --xml weights.xml --classes /path/to/classes.txt --conf-thres 0.7 --source 0                                       # webcam
                                                                                                                path/to/vid.mp4                         # video
                                                                                                                *.mp4                                   # videos
*/
#include <opencv2/opencv.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <zbar.h>
#include <thread>
#include <yolo.hpp>
#include <util.hpp>
#include <argparse.hpp>
#include <filesystem>
#include <algorithm>

#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "sys/times.h"
#include "sys/vtimes.h"

constexpr int slider_max = 100;
constexpr int bordersize = 32;
util::rotating_buffer<int16_t, 25> frame_times;
util::rotating_buffer<int16_t, 25> cpu_percentages;

static void on_trackbar(int slider_value, void *data)
{
    yolo::yolo_net *net = (yolo::yolo_net *)data;
    net->set_conf_thresh((float)slider_value / slider_max);
}

int main(int argc, const char **argv)
{
#ifdef NDEBUG
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_ERROR);
#else
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_VERBOSE);
#endif
    util::cpu_timer_t cpu_timer;
    argparse::ArgumentParser program("detect");

    program.add_argument("--bin")
        .help("Path to the weights.bin")
        .required();

    program.add_argument("--xml")
        .help("Path to the weights.xml")
        .required();

    program.add_argument("--source")
        .help("The camera index or path the video")
        .default_value(std::vector<std::string>({"0"}))
        .nargs(argparse::nargs_pattern::at_least_one);

    program.add_argument("--target-fps")
        .help("The target fps for detection")
        .scan<'g', float>()
        .default_value(30.f);

    program.add_argument("--conf-thres")
        .help("The confidence threshold for detection")
        .scan<'g', float>()
        .default_value(0.7f);

    program.add_argument("--GPU", "--gpu")
        .help("Whether to use the GPU")
        .implicit_value(true)
        .default_value(false);

    program.add_argument("--classes")
        .help("Optional path to the classlist.txt. The classes will be enumerated if this option isn't set.")
        .default_value(std::string(""));

    program.add_argument("--output-path")
        .help("The (optional) path where videos will be saved")
        .default_value(std::string(""));

    program.add_argument("--no-show")
        .help("Whether show the detections in a window")
        .implicit_value(true)
        .default_value(false);

    program.add_argument("--no-qr")
        .help("Whether to detect QR codes")
        .implicit_value(true)
        .default_value(false);

    program.add_argument("--no-yolo")
        .help("Whether to perform the object detection")
        .implicit_value(true)
        .default_value(false);

    // program.add_argument("--output-layer-name")
    //     .help("Currently does nothing")
    //     .default_value(std::string(""));

    try
    {
        program.parse_args(argc, argv);
    }
    catch (std::runtime_error &e)
    {
        std::cerr << e.what() << std::endl;
        std::cerr << program;
        return -1;
    }

    float target_fps = program.get<float>("target-fps");
    {
        std::string bin = program.get<std::string>("bin");
        std::string xml = program.get<std::string>("xml");
        std::string txt = program.get<std::string>("classes");
    }
    yolo::yolo_net net(
        program.get<std::string>("bin"),
        program.get<std::string>("xml"),
        program.get<std::string>("classes"),
        program.get<bool>("GPU") ? cv::dnn::DNN_TARGET_OPENCL_FP16 : cv::dnn::DNN_TARGET_CPU,
        program.get<float>("conf-thres"));

    std::vector<std::string> sources = program.get<std::vector<std::string>>("source");

    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    cv::Mat frame;
    std::vector<yolo::Detection> output;
    int64_t inference_time = 0;

    const char *windowname = "Yolo Detection";
    const char *slidername = "Confidence";
    if (!program.get<bool>("no-show"))
    {
        cv::namedWindow(windowname);
        cv::createTrackbar(slidername, windowname, nullptr, slider_max, on_trackbar, &net);
        cv::setTrackbarPos(slidername, windowname, net.get_conf_thresh() * slider_max);
    }
    std::string updated_path = "this/path/doesnt/exist";
    for (std::string video_path : sources)
    {
        int64_t frame_count = 0;
        char *not_numeric;
        long id = strtol(video_path.c_str(), &not_numeric, 10);

        std::filesystem::path fullpath(video_path);
        if (*not_numeric && !std::filesystem::exists(fullpath))
        {
            break;
        }
        cv::VideoCapture cap = *not_numeric ? cv::VideoCapture(video_path) : cv::VideoCapture(id);

        std::filesystem::path video_name = *not_numeric ? fullpath.filename() : std::filesystem::path(std::string("video") + std::to_string(id) + ".mp4");

        if (!cap.isOpened())
        {
            if (*not_numeric)
            {

                std::cerr << "Can't access video " << video_path << std::endl;
            }
            else
            {
                std::cerr << "Can't access camera id " << id << std::endl;
            }
            return -1;
        }

        cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
        auto total_frames = (long long)cap.get(cv::CAP_PROP_FRAME_COUNT);

        cv::VideoWriter writer;
        std::string outpath = program.get<std::string>("output-path");
        std::string outfile;
        if (outpath.size() > 0)
        {
            static bool first = true;
            if (first)
            {
                updated_path = outpath; // increment the path if it exists
                first = false;
                for (int i = 1; std::filesystem::exists(updated_path); i++)
                {
                    updated_path = outpath + std::to_string(i);
                }
                if (!std::filesystem::create_directory(updated_path))
                {
                    std::cout << "couldn't open the output directory\n";
                    std::exit(-1);
                }
            }

            outfile = updated_path / video_name;
            // increment the file name if it exists
            for (int i = 0; std::filesystem::exists(outfile); i++)
            {
                std::filesystem::path newname = video_name.stem().string() + std::to_string(i) + video_name.extension().string();
                outfile = outpath / newname;
            }
            std::cout << outfile << std::endl;
            cv::Size video_size{(int)cap.get(cv::CAP_PROP_FRAME_WIDTH), (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT) + bordersize};
            writer = cv::VideoWriter(outfile,
                                     0x31637661,
                                     target_fps,
                                     video_size);
        }
        while (true)
        {
            util::Rate timer(1'000'000. / target_fps);
            using namespace std::chrono;
            cap >> frame;
            if (frame.empty())
                break;

            auto t0 = high_resolution_clock::now();
            cv::Mat grey;
            cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);
            zbar::Image image(grey.cols, grey.rows, "Y800", (uchar *)grey.data, grey.cols * grey.rows);
            auto detect_yolo = [&]()
            {
                if (!program.get<bool>("no-yolo"))
                    output = net.detect(frame);
            };
            auto detect_qr = [&]()
            {
                if (!program.get<bool>("no-qr"))
                    scanner.scan(image);
            };

            std::thread yolo_thread(detect_yolo);
            std::thread qr_thread(detect_qr);
            qr_thread.join();
            yolo_thread.join();
            auto t1 = high_resolution_clock::now();

            yolo::draw_qrs(image, frame);
            yolo::draw_boxes(output, frame, net);

            inference_time = duration_cast<milliseconds>(t1 - t0).count();
            {

                cv::Mat out(cv::Size(frame.cols, frame.rows + bordersize), CV_8UC3);
                cv::copyMakeBorder(frame, out, bordersize, 0, 0, 0, cv::BORDER_CONSTANT, 0);
                frame = out;
            }
            char buff[100];
            frame_times.add_value((int16_t)inference_time);
            cpu_percentages.add_value((int16_t)cpu_timer.getCPUPercentage());
            snprintf(buff, sizeof(buff), "Inference: %4d ms        CPU: %3d%%", frame_times.get_average(), cpu_percentages.get_average());
            cv::putText(frame, buff, {15, 25}, cv::FONT_HERSHEY_DUPLEX, 1.0, {255, 0, 0}, 2);

            if (writer.isOpened())
            {
                int chars = fprintf(stdout, "FRAME %6ld / %6lld\r", frame_count, total_frames);
                fflush(stdout);
                if (frame_count + 1 == total_frames)
                    printf("%*s", chars, "\r");
                writer << frame;
            }

            if (!program.get<bool>("no-show"))
            {
                // very convoluted way to exit when the window is closed
                try
                {
                    double button_pressed = cv::getWindowProperty(windowname, cv::WND_PROP_FULLSCREEN);
                    if (button_pressed < 0.)
                    {
                        if (frame_count == 0 && writer.isOpened())
                        {
                            std::cout << "DELETING FILE: " << outfile << std::endl;
                            writer.release();
                            std::filesystem::remove(outfile);
                        }
                        break;
                    }
                }
                catch (const cv::Exception &e)
                {
                    break;
                }
                cv::imshow(windowname, frame);
                char c = (char)cv::waitKey(1);
                if (c == 'q')
                    break;
                timer.wait();
            }
            else
            {
                // cv::waitKey(1);
                timer.wait();
            }
            frame_count++;
        }
        cap.release();
        writer.release();
    }
    if (std::filesystem::exists(updated_path) && std::filesystem::is_empty(updated_path))
    {
        std::cout << "DELETING " << updated_path << " AS IT'S EMPTY" << std::endl;
        std::filesystem::remove(updated_path);
    }
    cv::destroyAllWindows();
}