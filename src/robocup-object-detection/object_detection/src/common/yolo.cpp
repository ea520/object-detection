#include "yolo.h"
#include <ros/package.h>
#include <fstream>
#include <thread>
namespace yolo
{
    cv::Mat format_yolov5(const cv::Mat &source)
    {
        int col = source.cols;
        int row = source.rows;
        float aspect = (float)row / (float)col;

        int new_width = (int)INPUT_WIDTH;
        int new_height = int(new_width * aspect);
        cv::Mat resized;
        // assume it's landscape with an aspect ratio greater than 640:512
        cv::resize(source, resized, cv::Size(new_width, new_height), 0, 0, cv::INTER_AREA);
        cv::Mat result = cv::Mat::zeros(int(INPUT_HEIGHT), int(INPUT_WIDTH), CV_8UC3);

        resized.copyTo(result(cv::Rect(0, 0, new_width, new_height)));
        return result;
    }

    void load_net(cv::dnn::Net &net, bool cpu)
    {
        std::string path = ros::package::getPath("object_detection") + "/weights/best";
        std::string bin_path{path + ".bin"}, xml_path{path + ".xml"};
        std::ifstream bin{bin_path}, xml{xml_path};
        assert(bin.is_open() && xml.is_open());
        bin.close();
        xml.close();
        net = cv::dnn::readNetFromModelOptimizer(xml_path, bin_path);
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_INFERENCE_ENGINE);
        net.setPreferableTarget(cpu ? cv::dnn::DNN_TARGET_CPU : cv::dnn::DNN_TARGET_OPENCL_FP16);
    }

    std::vector<Detection> detect(const cv::Mat &image, cv::dnn::Net &net)
    /*
     - resize the input image and add borders so it's the right input shape
     - run the YOLO inference on that image
     - the output format is an array of {x,y,w,h,conf,score1,...,score16}
     - where x,y are the coordinates of the top left of the bounding box
     - these coordinates are normalised to [0,1]
     - w,h are the width and height of the bounding box. Also normalised.
     - conf is the confidence in the prediction [0,1]
     - the detected class is the one with the highest score
     - The variable `rows` is the number of such arrays there are.
     - Perhaps the yolo output format is best shown by looking at the variable `data`
     -
    */
    {
        cv::Mat blob;
        auto input_image = format_yolov5(image); // make the image the right shape by scaling and adding black bars
        cv::dnn::blobFromImage(input_image, blob, 1 / 255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
        net.setInput(blob);

        cv::Mat outputs;
        // net.forward(outputs, "output"); // perform inference, get the data
        net.forward(outputs, "model/tf_detect/concat_3"); // perform inference, get the data
        struct yolo_output_data_t
        {
            float x, y, w, h;                         // Position of the object (normalised)
            float conf;                               // The confidence in range [0, 1]
            std::array<float, dimensions - 5> scores; // demensions - 5 is the number of classes (5 comes from the 5 floats x,y,w,h,conf)
                                                      // Scores 1 per class. The object is represented by the class with the highest score.
        };
        static_assert(sizeof(yolo_output_data_t) == sizeof(float) * dimensions); // sanity check
        const std::array<yolo_output_data_t, rows> &data = *(const std::array<yolo_output_data_t, rows> *)outputs.data;

        std::vector<int> class_ids; // class_ids[0] would be the class id corresponding to the networks 1st confident output
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;

        for (const auto &datum : data)
        {
            assert(datum.conf >= 0. && datum.conf <= 1.); // sanity check
            if (datum.conf <= CONFIDENCE_THRESHOLD)       // threshold the confidence
                continue;

            const float *max_score_ptr = std::max_element(datum.scores.begin(), datum.scores.end()); // get a pointer to the element with the max score
            int class_id = max_score_ptr - datum.scores.begin();                                     // find the index of the class with the highest score using pointer arithmetic
            assert(class_id >= 0 && class_id <= dimensions - 5);
            constexpr int door_id = 2;
            if (class_id == door_id && datum.conf < 0.95)
                continue;
            if (*max_score_ptr > SCORE_THRESHOLD) // threshold the max score
            {

                confidences.push_back(datum.conf);
                class_ids.push_back(class_id);

                cv::Point2i top_left(int((datum.x - 0.5 * datum.w) * INPUT_WIDTH), int((datum.y - 0.5 * datum.h) * INPUT_HEIGHT));
                cv::Size2i box_size((int)(datum.w * INPUT_WIDTH), (int)(datum.h * INPUT_HEIGHT));

                boxes.emplace_back(top_left, box_size); // add the corresponding box to the list
            }
        }
        // Some of these predictions correspond to the same object.
        // Perform non-max suppression in order to only choose one of the objects that overlap a lot

        std::vector<int> nms_result;
        cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);

        std::vector<Detection> output;
        output.reserve(nms_result.size());

        for (int idx : nms_result)
        {
            Detection result;
            result.class_id = class_ids[idx];
            result.confidence = confidences[idx];
            result.box = boxes[idx];
            output.push_back(result);
        }
        return output;
    }

    void draw_boxes(const std::vector<Detection> &detections, cv::Mat &frame)
    {
        for (size_t i = 0; i < detections.size(); ++i)
        {

            const auto &detection = detections[i];
            const auto &box = detection.box;
            auto classId = detection.class_id;
            assert(classId >= 0 && classId <= dimensions - 5);
            const auto &color = colors[classId];

            cv::rectangle(frame, box, color, 3);
            cv::rectangle(frame, cv::Point(box.x, box.y - 10), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
            cv::putText(frame, class_list[classId], cv::Point(box.x, box.y), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 0));
        }
    }

    void draw_qrs(const zbar::Image &image, cv::Mat &frame)
    {
        for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
        {
            const std::string text = symbol->get_data();

            std::vector<int> xs, ys;
            for (int i = 0; i < symbol->get_location_size(); i++)
            {
                xs.push_back(symbol->get_location_x(i));
                ys.push_back(symbol->get_location_y(i));
            }

            int x1 = *std::min_element(xs.begin(), xs.end());
            int y1 = *std::min_element(ys.begin(), ys.end());
            int x2 = *std::max_element(xs.begin(), xs.end());
            int y2 = *std::max_element(ys.begin(), ys.end());

            cv::rectangle(frame, cv::Rect(cv::Point2i(x1, y1), cv::Point2i(x2, y2)), {255, 0, 0}, 3);
            cv::putText(frame, symbol->get_data(), {x1, y1}, cv::FONT_HERSHEY_PLAIN, 1.5, {0, 0, 255}, 2);
        }
    }

    void Rate::pause()
    {
        using namespace std::chrono;
        auto remaining = ms - (high_resolution_clock::now() - t0);
        std::this_thread::sleep_for(remaining);
        m_overspent = remaining < 0ms;
        t0 = high_resolution_clock::now();
    }
}
