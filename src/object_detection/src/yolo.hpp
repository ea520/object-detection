#pragma once
#include <opencv2/opencv.hpp>
#include <zbar.h>
#include "object.hpp"

struct yolo_net
{
    yolo_net(const std::string &bin_path, const std::string &xml_path, const std::string &class_list_path, int target, float conf_thresh = 0.8);
    yolo_net(){};
    std::vector<object2d> detect(const cv::Mat &image);
    void set_conf_thresh(float thresh) { conf_thresh = thresh; }
    float get_conf_thresh() const { return conf_thresh; }
    const std::vector<std::string> &get_class_list() const { return class_list; }

private:
    cv::Mat format_yolov5(const cv::Mat &source, int &xoffset, int &yoffset, float &scale_factor);
    cv::dnn::Net net;
    float conf_thresh;
    int detection_count, floats_per_detection, INPUT_HEIGHT, INPUT_WIDTH;
    std::vector<std::string> class_list;
    std::string output_name;
};
