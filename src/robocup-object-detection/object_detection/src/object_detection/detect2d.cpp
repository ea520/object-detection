#include <opencv2/opencv.hpp>
#include <zbar.h>
#include <thread>
#include "yolo.h"
#include "util.h"
cv::dnn::Net net;
util::CPU_timer cpu_timer;
int main(int argc, char **argv)
{
    bool GPU = false;
    const char* error_msg = "Available cmd line arguments:\n\t--GPU/--gpu (whether inference is done on GPU. Default is false)\n";
    switch (argc)
    {
    case 1:
        GPU = false;
        break;
    case 2:
        if(!strcmp(argv[1], "--gpu") || !(strcmp(argv[1], "--GPU"))){
            GPU = true;
        }
        else{
            std::cerr << error_msg;
            std::cerr << "\t" << argv[1] << std::endl;
            std::exit(1);        
        }
        break;
    default:
        std::cerr << error_msg;
        std::exit(1);    
    }

    int frame_time = 100;
    if (argc == 2)
    {

        frame_time = atoi(argv[1]);
        if (frame_time <= 0)
            frame_time = 100;
    }
    std::cerr << (GPU ? "Using GPU\n" : "Using CPU\n") << std::endl;
    yolo::load_net(net, !GPU);
    cv::VideoCapture cap(0);

    if (!cap.isOpened())
    {
        std::cerr << "Can't access primary camera" << std::endl;
        return -1;
    }
    cap.set(cv::CAP_PROP_BUFFERSIZE, 1);
    zbar::ImageScanner scanner;
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    cv::Mat frame;
    std::vector<yolo::Detection> output;
    int64_t inference_time = 0;
    int64_t frame_count = 0;
    int64_t cumulative_time = 0;
    // Rate rate(83); // aim for 12 fps
    yolo::Rate rate(frame_time); // aim for 10 fps
    while (true)
    {
        using namespace std::chrono;
        cap >> frame;

        auto t0 = high_resolution_clock::now();
        cv::Mat grey;
        cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);
        zbar::Image image(grey.cols, grey.rows, "Y800", (uchar *)grey.data, grey.cols * grey.rows);

        auto detect_yolo = [&]()
        {
            output = yolo::detect(frame, net);
        };
        auto detect_qr = [&]()
        {
            scanner.scan(image);
        };

        std::thread yolo_thread(detect_yolo);
        std::thread qr_thread(detect_qr);

        qr_thread.join();
        yolo_thread.join();
        auto t1 = high_resolution_clock::now();
        yolo::draw_qrs(image, frame);
        yolo::draw_boxes(output, frame);

        cumulative_time += duration_cast<milliseconds>(t1 - t0).count();
        frame_count++;
        if (cumulative_time > 250 && frame_count > 10)
        {
            inference_time = cumulative_time / frame_count; // 1000 * 1000 * 1000 * frame_count / cumulative_time;
            frame_count = 0;
            cumulative_time = 0;
            // std::cout << fps << std::endl;
        }
        cv::putText(frame, std::to_string(inference_time) + "ms inference", {25, 460}, cv::FONT_HERSHEY_DUPLEX, 1.0, {0, 0, 0}, 2);
        cpu_timer.update();
        double cpu_usage = cpu_timer.get_usage();
        int integer_usage = cpu_usage;
        int tens = 10 * (cpu_usage - integer_usage);
        cv::putText(frame, std::string("CPU ") + std::to_string(integer_usage) + "." + std::to_string(tens) + "%", {10, 30}, cv::FONT_HERSHEY_DUPLEX, 0.7, {100, 200, 0}, 1);
        cv::imshow("Detections", frame);
        char c = (char)cv::waitKey(1);
        if (c == 'q')
            break;
        rate.pause();
    }
}