#pragma once
#include <string>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
#include <librealsense2/rs.hpp>
enum class object_type
{
    QR,
    PERSON,
    FIRE_EXTINGUISHER,
    DOOR,
    HAZMAT,
    SIZE
};

// Data straight from YOLO/QR code detector
struct object2d
{
    int u, v, w, h;
    object_type type;
    std::string text;
    float confidence;
};

// Adding the info from the depth camera
struct object2d_with_depth
{
    int u, v;
    float depth;
    Eigen::Vector3f normal;
    object_type type;
    std::string text;
    float confidence;
};

struct camera_pose_t
{
    Eigen::Vector3f translation;
    Eigen::Quaternionf rotation;
    Eigen::Matrix3f covariance;
};

// Adding the info from the tracking camera
struct observation_3d
{
    Eigen::Vector3f position;
    Eigen::Matrix3f covariance;
    Eigen::Vector3f normal;
    object_type type;
    std::string text;
    float confidence;
    bool normal_x_pos;
};

void draw_boxes(const std::vector<object2d> &detections, cv::Mat &frame);
std::vector<object2d_with_depth> get_2d_with_depths(const std::vector<object2d> &detections, cv::Mat &depth, const rs2_intrinsics &intrinsics);
std::vector<observation_3d> get_3d(const std::vector<object2d_with_depth> &detections, const camera_pose_t &camera_pose, const rs2_intrinsics &intrinsics);