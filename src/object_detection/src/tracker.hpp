#pragma once
#include "object.hpp"
struct state_3d
{
    object_type type;
    Eigen::Vector3f position;
    Eigen::Matrix3f covariance;
    Eigen::Matrix<float, 13, 1> probs; // redundant for everything but hazmats
    Eigen::Vector3f normal;            // redundant for non-planar objects
    int miss_count = 0;
    int hit_count = 0;
    int id;             // unique ID for other objects
    std::string QR_txt; // QR code text
    void update(const observation_3d &new_obs);
    visualization_msgs::Marker get_object_marker() const;
    visualization_msgs::Marker get_covariance_marker() const;
    visualization_msgs::Marker get_QR_text_marker() const;
    inline float distance(const observation_3d &new_obs) const
    {
        // Objects of different types can't be matched
        assert(new_obs.type == type);
        // QRs are assumed to have unique text
        if (type == object_type::QR)
        {
            return QR_txt == new_obs.text ? 0. : INFINITY;
        }
        // Essentially how many standard deviations away they are
        Eigen::Vector3f e = position - new_obs.position;
        Eigen::Matrix3f S = covariance + new_obs.covariance;
        return e.dot(S.inverse() * e);
    }
};
struct tracker_t
{
    void update(const std::vector<observation_3d> &new_obs);
    std::vector<state_3d> get_objects() const
    {
        std::vector<state_3d> ret;
        for (const auto &[type, objs] : objects)
        {
            for (const auto &obj : objs)
            {
                ret.push_back(obj);
            }
        }
        return ret;
    };
    std::unordered_map<object_type, std::vector<state_3d>> objects;
};