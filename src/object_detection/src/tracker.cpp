#include "tracker.hpp"
#include <ros/package.h>
static int get_id(const std::string &txt)
{
    if (txt == "IH")
        return 0;
    else if (txt == "IS")
        return 1;
    else if (txt == "EX")
        return 2;
    else if (txt == "NF")
        return 3;
    else if (txt == "OP")
        return 4;
    else if (txt == "FL")
        return 5;
    else if (txt == "RA")
        return 6;
    else if (txt == "SC")
        return 7;
    else if (txt == "OX")
        return 8;
    else if (txt == "DA")
        return 9;
    else if (txt == "FS")
        return 10;
    else if (txt == "CO")
        return 11;
    else if (txt == "PO")
        return 12;
    return -1;
}
void state_3d::update(const observation_3d &new_obs)
{
    assert(new_obs.type == type);
    // Kalman filter for stationary object
    Eigen::Matrix3f gain = covariance * (covariance + new_obs.covariance).inverse();
    position += gain * (new_obs.position - position);
    covariance -= gain * covariance;
    float old_weight = hit_count / (hit_count + 1.f);

    // For hazmats, update the probability distribution
    if (type == object_type::HAZMAT)
    {
        Eigen::Matrix<float, 13, 1> ps = Eigen::Matrix<float, 13, 1>::Zero();
        ps.array() = (1 - new_obs.confidence) / 12;
        int class_idx = get_id(new_obs.text);
        if (class_idx < 0)
        {
            std::cerr << "ID ERROR";
            return;
        }
        ps(class_idx) = new_obs.confidence;
        probs = probs * old_weight + ps * (1.f - old_weight);
    }
    // take the average of all the normals
    normal = normal * old_weight + new_obs.normal * (1.f - old_weight);
    hit_count++;
    miss_count = 0;
}

// This quaternon rotates (1,0,0) to the normal "n"
// Without initial rotation about the x axis
Eigen::Quaternionf normal_to_quaternion(Eigen::Vector3f n)
{
    n.normalize();
    auto axis = Eigen::Vector3f::UnitX().cross(n);
    axis.normalize();
    float theta = acosf(Eigen::Vector3f::UnitX().dot(n));
    Eigen::Quaternionf q(Eigen::AngleAxisf(theta, axis));
    return q;
}

visualization_msgs::Marker state_3d::get_object_marker() const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.lifetime = ros::Duration(0.5);
    switch (type)
    {
    case object_type::HAZMAT:
    {
        static const std::array<std::string, 13> filenames = {

            "inhalation-hazard.dae",
            "infectious-substance.dae",
            "explosive.dae",
            "non-flammable-gas.dae",
            "organic-peroxide.dae",
            "flammable.dae",
            "radioactive.dae",
            "spontaneously-combustible.dae",
            "oxygen.dae",
            "dangerous.dae",
            "flammable-solid.dae",
            "corrosive.dae",
            "poison.dae",

        };
        Eigen::Index index;
        probs.maxCoeff(&index);

        marker.ns = "objects";
        marker.id = id;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        std::string path = ros::package::getPath("object_detection") + "/../../resources/";
        marker.mesh_resource = "file://" + path + filenames[index];
        marker.mesh_use_embedded_materials = true;
        marker.scale.x = marker.scale.y = marker.scale.z = .5;
        Eigen::Quaternionf q = normal_to_quaternion(normal);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
    }
    break;
    case object_type::QR:
    {
        marker.ns = "QR";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        Eigen::Quaternionf q = normal_to_quaternion(normal);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 0.001;
        marker.scale.y = 0.07;
        marker.scale.z = 0.07;
        marker.color.a = 1.;
        marker.color.r = .8;
        marker.color.g = .8;
        marker.color.b = .8;
    }
    break;
    case object_type::FIRE_EXTINGUISHER:
    {
        marker.ns = "fire-extinguishers";
        marker.id = id;
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        std::string path = ros::package::getPath("object_detection") + "/../../resources/";
        marker.mesh_resource = "file://" + path + "fire-extinguisher.dae";
        marker.mesh_use_embedded_materials = true;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
        marker.pose.orientation.x = 0;
        marker.pose.orientation.y = 0;
        marker.pose.orientation.z = 0;
        marker.pose.orientation.w = 1;
    }
    break;
    case object_type::DOOR:
    {
        marker.ns = "door";
        marker.id = id;
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        Eigen::Quaternionf q = normal_to_quaternion(normal);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 0.1;
        marker.scale.y = 0.8;
        marker.scale.z = 2.0;
        marker.color.a = 1.;
        marker.color.r = .8;
        marker.color.g = .8;
        marker.color.b = .8;
    }
    break;

    default:
        break;
    }
    return marker;
}

visualization_msgs::Marker state_3d::get_covariance_marker() const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "covariances";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.lifetime = ros::Duration(0.5);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(covariance);
    auto evals = es.eigenvalues();
    auto evecs = es.eigenvectors();
    evecs.col(0).normalize();
    evecs.col(1).normalize();
    evecs.col(2).normalize();
    assert(evecs.isUnitary(0.01));
    Eigen::Quaternionf q(evecs);
    q.normalize();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();
    marker.pose.orientation.w = q.w();
    marker.scale.x = sqrtf(evals[0]);
    marker.scale.y = sqrtf(evals[1]);
    marker.scale.z = sqrtf(evals[2]);
    marker.color.a = 0.2;
    marker.color.g = 1.;
    return marker;
}

visualization_msgs::Marker state_3d::get_QR_text_marker() const
{
    assert(type == object_type::QR);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "QR-text";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z() + 0.05;
    marker.text = "QR: " + QR_txt;
    marker.scale.z = 0.02;
    marker.color.r = marker.color.a = 1;
    marker.lifetime = ros::Duration(0.5);

    return marker;
}

void tracker_t::update(const std::vector<observation_3d> &_new_obs)
{
    std::vector<object_type> available_objects = {object_type::HAZMAT, object_type::FIRE_EXTINGUISHER, object_type::QR, object_type::DOOR};
    // Separate the objects by type
    std::unordered_map<object_type, std::vector<observation_3d>> new_objects;
    for (const auto &obj : _new_obs)
    {
            new_objects[obj.type].push_back(obj);
    }

    // Matrixes of distances from known objects to new objects
    // One matrix per object type

    for (auto t : available_objects)
    {
        Eigen::MatrixXf distances = Eigen::MatrixXf::Zero(objects[t].size(), new_objects[t].size());

        for (int i = 0; i < (int)objects[t].size(); i++)
            for (int j = 0; j < (int)new_objects[t].size(); j++)
                distances(i, j) = objects[t][i].distance(new_objects[t][j]);

        constexpr float threshold = 21.0f;
        std::vector<bool> knowns_paired = std::vector<bool>(objects[t].size(), false);
        std::vector<bool> new_paired = std::vector<bool>(new_objects[t].size(), false);
        if (distances.rows() && distances.cols())
            while (true)
            {
                Eigen::Index known, _new;
                distances.minCoeff(&known, &_new);

                if (distances(known, _new) > threshold)
                    break;

                knowns_paired[known] = true;
                new_paired[_new] = true;
                objects[t][known].update(new_objects[t][_new]);

                distances.row(known).array() = INFINITY;
                distances.col(_new).array() = INFINITY;
            }
        for (int i = knowns_paired.size() - 1; i > -1; i--)
        {
            if (!knowns_paired[i])
            {
                // object wasn't found in the most recent frame
                // increment the miss count
                objects[t][i].miss_count++;
                if (objects[t][i].miss_count > 100 && objects[t][i].hit_count < 10 && t != object_type::QR)
                    objects[t].erase(objects[t].begin() + i);
            }
        }
        for (int i = 0; i < (int)new_paired.size(); i++)
        {
            if (!new_paired[i])
            {
                // New object that needs to be added to the list of known object
                static int ID;
                observation_3d observation = new_objects[t][i];
                state_3d new_state;
                new_state.normal = observation.normal;
                new_state.position = observation.position;
                new_state.covariance = observation.covariance;
                new_state.type = observation.type;
                if (observation.type == object_type::HAZMAT)
                {
                    Eigen::Matrix<float, 13, 1> probs = Eigen::Array<float, 13, 1>((1. - observation.confidence) / 12);
                    int class_idx = get_id(observation.text);
                    if (class_idx < 0)
                    {
                        std::cout << "ID ERROR";
                        continue;
                    }
                    probs(class_idx) = observation.confidence;
                    new_state.probs = probs;
                }
                else if (observation.type == object_type::QR)
                {
                    new_state.QR_txt = observation.text;
                }
                new_state.id = ID++;
                objects[t].push_back(new_state);
            }
        }
    }
}