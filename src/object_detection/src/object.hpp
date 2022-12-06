#pragma once
#include <string>
#include <Eigen/Dense>
#include <vector>
#include <opencv2/opencv.hpp>
#include <visualization_msgs/Marker.h>
enum class object_type
{
    QR,
    PERSON,
    FIRE_EXTINGUISHER,
    DOOR,
    HAZMAT,
    SIZE
};
struct object2d
{
    int u, v, w, h;
    object_type type;
    std::string text;
    float confidence;
};

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

// struct QR_state
// {
//     Eigen::Vector3f position;
//     Eigen::Matrix3f covariance;
//     Eigen::Vector3f normal; // THIS IN GENERAL WON'T BE NORMALIZED AS IT'S THE AVERAGE OF ALL THE ESTIMATED NORMAL VECTORS
//     std::string text;
//     int miss_count = 0;
//     int hit_count = 0;
//     void update(const oservation_3d &new_obs)
//     {
//         text = new_obs.text;
//         assert(new_obs.type == object_type::QR && text == new_obs.text);
//         Eigen::Matrix3f gain = covariance * (covariance + new_obs.covariance).inverse();
//         position += gain * (new_obs.position - position);
//         covariance -= gain * covariance;
//         float old_weight = hit_count / (hit_count + 1.f);
//         // filter the normal by taking the average. Don't normalise yet
//         normal = normal * old_weight + new_obs.normal * (1.f - old_weight);
//         hit_count++;
//         miss_count = 0;
//     }
//     float dist(const oservation_3d &new_obs) const
//     {
//         assert(new_obs.type == object_type::QR && text == new_obs.text);
//         // assume QR codes can't decode the wrong text
//         if (new_obs.text != text)
//             return INFINITY;
//         Eigen::Vector3f e = position - new_obs.position;
//         Eigen::Matrix3f S = covariance + new_obs.covariance;
//         return e.dot(S.inverse() * e); // e^T S^-1 e
//     }
//     visualization_msgs::Marker get_qr_marker() const;
//     visualization_msgs::Marker get_text_marker() const;
// };

// struct Hazmat_state
// {
//     bool operator<(const Hazmat_state &other) { return id < other.id; };
//     Eigen::Vector3f position;
//     Eigen::Matrix3f covariance;
//     Eigen::Matrix<float, 13, 1> probs;
//     Eigen::Vector3f normal;
//     int miss_count = 0;
//     int hit_count = 0;
//     int id;

//     void update(const oservation_3d &new_obs);
//     visualization_msgs::Marker get_hazmat_marker() const;
//     visualization_msgs::Marker get_elipsoid_marker() const;

//     float dist(const oservation_3d &new_obs) const
//     {
//         assert(new_obs.type == object_type::HAZMAT);
//         Eigen::Vector3f e = position - new_obs.position;
//         Eigen::Matrix3f S = covariance + new_obs.covariance;
//         return e.dot(S.inverse() * e); // e^T S^-1 e
//     }

//     friend std::ostream &operator<<(std::ostream &, Hazmat_state);
// };

// struct Fire_extinguisher_state
// {
//     bool operator<(const Fire_extinguisher_state &other) { return id < other.id; };
//     Eigen::Vector3f position;
//     Eigen::Matrix3f covariance;
//     int miss_count = 0;
//     int hit_count = 0;
//     int id;

//     void update(const oservation_3d &new_obs);
//     visualization_msgs::Marker get_fire_extinguisher_marker() const;

//     float dist(const oservation_3d &new_obs) const
//     {
//         assert(new_obs.type == object_type::FIRE_EXTINGUISHER);
//         Eigen::Vector3f e = position - new_obs.position;
//         Eigen::Matrix3f S = covariance + new_obs.covariance;
//         return e.dot(S.inverse() * e); // e^T S^-1 e
//     }
// };

void draw_boxes(const std::vector<object2d> &detections, cv::Mat &frame);
std::vector<object2d_with_depth> get_2d_with_depths(const std::vector<object2d> &detections, cv::Mat &depth);
std::vector<observation_3d> get_3d(const std::vector<object2d_with_depth> &detections, const camera_pose_t &camera_pose);