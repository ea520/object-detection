#include "tracker.hpp"
#include <ros/package.h>
#include <Eigen/Eigenvalues>
#include <fstream>

// WILL CAUSE ERRORS IF AND WHEN THE OBJECT LABELS CHANGE
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
    throw std::runtime_error("The names of the hazmat signs have probably been changed");
    return -1;
}
// Perform a kalman filter update step
void state_3d::update(const observation_3d &new_obs)
{
    assert(new_obs.type == type);
    // Kalman filter for stationary object
    covariance += Eigen::Matrix3f::Identity() * (type == object_type::PERSON ? (0.3 * 0.3) : 0);
    Eigen::Matrix3f gain = covariance * (covariance + new_obs.covariance).inverse();
    position += gain * (new_obs.position - position);
    covariance -= gain * covariance;

    // For hazmats, update the probability distribution
    if (type == object_type::HAZMAT)
    {
        float old_weight = hit_count / (hit_count + 1.f);
        distribution = distribution * old_weight + new_obs.distribution * (1.f - old_weight);
    }
    if (!new_obs.orientation_matrix.hasNaN())
        orientation_matrix += new_obs.orientation_matrix;

    auto path = ros::package::getPath("object_detection") + "/detections.csv";
    std::ofstream output_file(path, std::ios_base::app);
    if (hit_count == min_detection_count())
    {
        output_file << *this << std::endl;
    }
    hit_count++;
    miss_count = 0;
}

// This quaternon rotates (1,0,0) to the normal "n"
// Without initial rotation about the x axis
Eigen::Quaternionf normal_to_quaternion(Eigen::Vector3f n, Eigen::Vector3f start = Eigen::Vector3f::UnitX())
{
    n.normalize();
    auto axis = start.cross(n);
    axis.normalize();
    float theta = acosf(start.dot(n));
    Eigen::Quaternionf q(Eigen::AngleAxisf(theta, axis));
    return q;
}

// Very verbose  but it just produces the messages for rviz to render the objects
visualization_msgs::Marker state_3d::get_object_marker(const Eigen::Quaternionf &camera_orientation) const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.id = id;
    marker.header.stamp = ros::Time();
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    marker.lifetime = ros::Duration(0.5);

    Eigen::Vector3f camera_direction = camera_orientation * Eigen::Vector3f::UnitX();
    auto normal = covar_to_orientation(orientation_matrix);
    if (camera_direction.dot(normal) > 0)
        normal *= -1;
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
        distribution.maxCoeff(&index);

        marker.ns = "objects";

        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        std::string path = ros::package::getPath("object_detection") + "/../../resources/meshes/";
        marker.mesh_resource = "file://" + path + filenames[index];
        marker.mesh_use_embedded_materials = true;
        marker.scale.x = marker.scale.y = marker.scale.z = .3;
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
        marker.type = visualization_msgs::Marker::CUBE;
        marker.action = visualization_msgs::Marker::ADD;

        Eigen::Quaternionf q = normal_to_quaternion(normal);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();

        marker.scale.x = 0.001;
        marker.scale.y = marker.scale.z = 0.07;
        marker.color.a = 1.;
        marker.color.r = .5;
        marker.color.g = .5;
        marker.color.b = .5;
    }
    break;
    case object_type::FIRE_EXTINGUISHER:
    {
        marker.ns = "fire-extinguishers";
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        std::string path = ros::package::getPath("object_detection") + "/../../resources/meshes/";
        marker.mesh_resource = "file://" + path + "fire-extinguisher.dae";
        marker.mesh_use_embedded_materials = true;
        marker.scale.x = marker.scale.y = marker.scale.z = 0.01;
        if (normal.z() < 0) // Always point somewhat upwards
            normal *= -1;
        Eigen::Quaternionf q = normal_to_quaternion(normal, Eigen::Vector3f::UnitZ());
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
    }
    break;
    case object_type::DOOR:
    {
        marker.ns = "door";
        marker.type = visualization_msgs::Marker::MESH_RESOURCE;
        marker.action = visualization_msgs::Marker::ADD;
        std::string path = ros::package::getPath("object_detection") + "/../../resources/meshes/";
        marker.mesh_resource = "file://" + path + "DoorCOLLADA.dae";
        marker.mesh_use_embedded_materials = true;

        Eigen::Quaternionf q = normal_to_quaternion(normal);
        marker.pose.orientation.x = q.x();
        marker.pose.orientation.y = q.y();
        marker.pose.orientation.z = q.z();
        marker.pose.orientation.w = q.w();
        marker.scale.x = marker.scale.y = marker.scale.z = 1.;
    }
    break;
    case object_type::PERSON:
    {
        marker.ns = "Person";
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = position.x();
        marker.pose.position.y = position.y();
        marker.pose.position.z = position.z();
        marker.text = "Person: #" + std::to_string(id);
        marker.scale.z = 0.02;
        marker.color.r = marker.color.a = 1;
        marker.lifetime = ros::Duration(0.5);
    }
    break;

    default:
        break;
    }
    return marker;
}

// Draw elipses for the covariance matrix
visualization_msgs::Marker state_3d::get_covariance_marker() const
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.id = id;
    marker.ns = "covariances";
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

// Marker for the QR code text
visualization_msgs::Marker state_3d::get_QR_text_marker() const
{
    assert(type == object_type::QR);
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.id = id;
    marker.ns = "QR-text";
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

// Match oobjects and update them
void tracker_t::update(const std::vector<observation_3d> &_new_obs)
{
    std::vector<object_type> available_objects = {object_type::HAZMAT, object_type::FIRE_EXTINGUISHER, object_type::QR, object_type::DOOR, object_type::PERSON};

    // Separate the objects by type
    std::unordered_map<object_type, std::vector<observation_3d>> new_objects;
    for (const auto &obj : _new_obs)
    {
        new_objects[obj.type].push_back(obj);
    }

    for (auto t : available_objects)
    {

        Eigen::MatrixXf distances = Eigen::MatrixXf::Zero(objects[t].size(), new_objects[t].size());

        for (int i = 0; i < (int)objects[t].size(); i++)
            for (int j = 0; j < (int)new_objects[t].size(); j++)
                distances(i, j) = objects[t][i].distance(new_objects[t][j]);

        // There is a very low probability of a false negative at this threshold (assuming noise model is correct)
        constexpr float threshold = 9.0f;

        // Keep track of which previously known objects have been paired
        std::vector<bool> knowns_paired = std::vector<bool>(objects[t].size(), false);

        // Keep track of which new observations have been paried
        std::vector<bool> new_paired = std::vector<bool>(new_objects[t].size(), false);
        // Make sure the array isn't empty (i.e. at the start)
        if (distances.size())
            while (true)
            {
                // Find the closest objects and match them if they're within a threshold
                Eigen::Index known, _new;
                distances.minCoeff(&known, &_new);

                if (distances(known, _new) > threshold)
                    break;

                // Mark them as paired and tick them off
                knowns_paired[known] = true;
                new_paired[_new] = true;
                objects[t][known].update(new_objects[t][_new]);

                // Make sure neither of these can be matched again
                distances.row(known).array() = INFINITY;
                distances.col(_new).array() = INFINITY;
            }
        // Go through the known objects which weren't in frame. Delete them if there's insufficient evidence that the objects actually exists
        for (int i = knowns_paired.size() - 1; i > -1; i--)
        {
            if (!knowns_paired[i])
            {
                // object wasn't found in the most recent frame
                // increment the miss count
                objects[t][i].miss_count++;
                if (objects[t][i].type == object_type::PERSON && objects[t][i].miss_count > 100)
                    objects[t].erase(objects[t].begin() + i);
                else if (objects[t][i].miss_count > 100 && objects[t][i].hit_count < objects[t][i].min_detection_count())
                    objects[t].erase(objects[t].begin() + i);
            }
        }

        // Go through the new observations that hadn't been seen before. Add them to the list of known objects.
        for (int i = 0; i < (int)new_paired.size(); i++)
        {
            if (!new_paired[i])
            {
                // New object that needs to be added to the list of known object
                static int ID;
                observation_3d observation = new_objects[t][i];
                state_3d new_state;
                if (!observation.orientation_matrix.hasNaN())
                    new_state.orientation_matrix = observation.orientation_matrix;
                else
                {
                    // Pointing upwards with little confidence
                    new_state.orientation_matrix = Eigen::Matrix3f::Identity() * 1e-10;
                    new_state.orientation_matrix(2, 2) = 0;
                }
                new_state.position = observation.position;
                new_state.covariance = observation.covariance;
                new_state.type = observation.type;
                if (observation.type == object_type::HAZMAT)
                {
                    new_state.distribution = observation.distribution;
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

std::string current_time()
{
    using sysclock_t = std::chrono::system_clock;
    std::time_t now = sysclock_t::to_time_t(sysclock_t::now());

    char buf[16] = {0};
    std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&now));

    return std::string(buf);
}

std::ostream &operator<<(std::ostream &os, const state_3d &self)
{
    static const std::array<std::string, 13> names = {

        "inhalation-hazard",
        "infectious-substance",
        "explosive",
        "non-flammable-gas",
        "organic-peroxide",
        "flammable",
        "radioactive",
        "spontaneously-combustible",
        "oxygen",
        "dangerous",
        "flammable-solid",
        "corrosive",
        "poison",

    };
    std::unordered_map<object_type, std::string> type_to_str{
        {object_type::HAZMAT, "hazmat"},
        {object_type::QR, "qr code"},
        {object_type::DOOR, "door"},
        {object_type::FIRE_EXTINGUISHER, "fire extinguisher"},
        {object_type::PERSON, "person"},
    };
    std::string text = self.QR_txt;
    if (self.type == object_type::HAZMAT)
    {
        Eigen::Index id;
        self.distribution.maxCoeff(&id);
        text = names[id];
    }
    auto prev = os.precision(2);
    {
        os << self.id << ',';
        os << current_time() << ',';
        os << text << ',';
        os << self.position.x() << ',';
        os << self.position.y() << ',';
        os << self.position.z() << ',';
        os << "Arbie" << ',';
        os << "T" << ',';
        os << type_to_str[self.type];
    }
    os.precision(prev);
    return os;
}