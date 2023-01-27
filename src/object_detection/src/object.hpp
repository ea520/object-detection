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
    int u, v, w, h;               // the coordinates of the object
    object_type type;             // the most likely type of the object
    float confidence;             // the probablity that there's an object of type `type`
    std::string text;             // the object type as a string
    Eigen::VectorXf distribution; // For hazmat signs, the probabilities of each hazmat type
};

// Adding the info from the depth camera
struct object2d_with_depth
{
    int u, v;                           // the coordinates of the object in the image
    float depth;                        // the perpendicular distance of the object to the camera
    Eigen::Matrix3f orientation_matrix; // the smallest eigenvalue of this matrix corresponds to the orientation of the object (its normal for planes or axis for cylinders)
    object_type type;                   // the most likely type of the object
    float confidence;                   // the probability that there is an object of that most likely type
    std::string text;                   // the object type as a string
    Eigen::VectorXf distribution;       // For hazmat signs, the probabilities of each hazmat type
};

struct camera_pose_t
{
    Eigen::Vector3f translation; // the position of the tracking camera relative to its initial position
    Eigen::Quaternionf rotation; // the rotation of the tracking camera relative to its initial position
    Eigen::Matrix3f covariance;  // an estimate of the covariance of the tracking camera position
};

// Adding the info from the tracking camera
struct observation_3d
{
    Eigen::Vector3f position;           // the position of the observed object's centre (corrupted by noise) relative to the world coordinate system
    Eigen::Matrix3f covariance;         // the covariance of the measurement noise
    Eigen::Matrix3f orientation_matrix; // in the world coordinate system
    object_type type;                   // the most likely type of the object
    std::string text;                   // the object type as a string
    float confidence;                   // the probability that there is an object of that most likely type
    Eigen::VectorXf distribution;       // For hazmat signs, the probabilities of each hazmat type
};

void draw_boxes(const std::vector<object2d> &detections, cv::Mat &frame);
std::vector<object2d_with_depth> get_2d_with_depths(const std::vector<object2d> &detections, cv::Mat &depth, const rs2_intrinsics &intrinsics);
std::vector<observation_3d> get_3d(const std::vector<object2d_with_depth> &detections, const camera_pose_t &camera_pose, const rs2_intrinsics &intrinsics);
Eigen::Vector3f covar_to_orientation(const Eigen::Matrix3f &covar);