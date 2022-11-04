#pragma once
#include <opencv2/opencv.hpp>
#include <zbar.h>
namespace yolo
{
    struct Detection
    {
        int class_id;
        float confidence;
        cv::Rect box;
    };
    struct yolo_net
    {
        yolo_net() {}
        yolo_net(const std::string &bin_path, const std::string &xml_path, const std::string &class_list_path, int target, float conf_thresh = 0.8);
        std::vector<Detection> detect(const cv::Mat &image);
        void set_conf_thresh(float thresh) { conf_thresh = thresh; }
        float get_conf_thresh() const { return conf_thresh; }
        const std::vector<std::string> &get_class_list() const { return class_list; }

    private:
        cv::Mat format_yolov5(const cv::Mat &source, int &xoffset, int &yoffset, float &scale_factor);
        cv::dnn::Net net;
        float conf_thresh;
        unsigned detection_count, floats_per_detection, INPUT_HEIGHT, INPUT_WIDTH;
        std::vector<std::string> class_list;
    };
    void draw_qrs(const zbar::Image &image, cv::Mat &frame);
    void draw_boxes(const std::vector<Detection> &detections, cv::Mat &frame, const yolo_net &net);
}
