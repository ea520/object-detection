#pragma once
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <thread>
#include <nav_msgs/Odometry.h>

namespace util
{
    sensor_msgs::Image toImageMsg(const cv::Mat &image);
    cv::Mat toCVMat(const sensor_msgs::ImageConstPtr &img);
}
namespace cpu_monitor
{
    void init();
    double getCurrentValue();
}
