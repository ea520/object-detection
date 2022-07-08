#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <mlpack/core.hpp>
#include <mlpack/methods/dbscan/dbscan.hpp>
#include <algorithm>
#include <chrono>
#include <stdint.h>
#include <object_detection/string_arr.h>
#include <object_detection/detection_arr.h>
#include <visualization_msgs/Marker.h>
bool rviz;
std::array<std::array<float, 3>, 15> colour_map = {
    std::array<float, 3>{0.902, 0.098, 0.294},
    std::array<float, 3>{0.235, 0.706, 0.294},
    std::array<float, 3>{1.000, 0.882, 0.098},
    std::array<float, 3>{0.263, 0.388, 0.847},
    std::array<float, 3>{0.961, 0.510, 0.192},
    std::array<float, 3>{0.569, 0.118, 0.706},
    std::array<float, 3>{0.275, 0.941, 0.941},
    std::array<float, 3>{0.941, 0.196, 0.902},
    std::array<float, 3>{0.737, 0.965, 0.047},
    std::array<float, 3>{0.980, 0.745, 0.745},
    std::array<float, 3>{0.000, 0.502, 0.502},
    std::array<float, 3>{0.604, 0.388, 0.141},
    std::array<float, 3>{1.000, 0.980, 0.784},
    std::array<float, 3>{0.502, 0.000, 0.000},
    std::array<float, 3>{0.667, 1.000, 0.765},
};

struct detection
{
    detection(float x = 0., float y = 0., float z = 0., float score = 0., u_int32_t id = -1) : x(x), y(y), z(z), score(score), id(id){};
    float x, y, z, score;
    uint32_t id;
    inline void print() const
    {
        printf("%25s <%5.2f, %5.2f, %5.2f> conf: %5.2f\n", classes[id], x, y, z, score);
    }

    static constexpr const char *classes[] = {
        "person",
        "fire extinguisher",
        "door",
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
        "corrosive"};
};

// int arr[] = {X15(1)} is equivalent to int arr[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1}
#define X12(x) x, x, x, x, x, x, x, x, x, x, x, x

constexpr size_t obj_class_count = 15;       // The number of classes e.g. person, door, explosive sign etc.
constexpr double epsilon = 2.e-2;            // The size of a cluster (key points in a cluster have MIN_POINTS points within a distance of epsilon)
constexpr size_t min_points = 10;            // The minimum number of surrounding points for a point to be classed as a key point
constexpr size_t max_detection_count = 50;   // The maximum number of points per object before points from earlier times get deleted
constexpr size_t point_count_threshold = 20; // The minimum number of points in a cluster before the cluster is said to be part of an object
constexpr double clustering_freq = 0.5;      // Hz
ros::Publisher detection_pub;                // Publish the detections in the required csv format but without the first 2 columns

static_assert(obj_class_count == 15, "YOU CAN DELETE THIS LINE IF YOU'RE SURE YOU'RE INITIALISING THE ARRAY CORRECTLY");
std::array<mlpack::dbscan::DBSCAN<>, obj_class_count> models = {
    mlpack::dbscan::DBSCAN(0.5, min_points),           // person key points within (10 cm)
    mlpack::dbscan::DBSCAN(0.1, min_points),           // fire extinguisher
    mlpack::dbscan::DBSCAN(0.5, min_points),           // door (key points within 20 cm)
    X12(mlpack::dbscan::DBSCAN<>(epsilon, min_points)) // The use of the X15 macro relies on obj_class_count = 15
};

std::array<std::vector<std::array<double, 3>>, obj_class_count> detections;
std::array<detection, obj_class_count> filtered_detections;
std::array<std::vector<std::array<double, 3>>, obj_class_count> positions;
ros::Publisher marker_pub;
bool isclose(const std::array<double, 3> &point, std::vector<std::array<double, 3>> &points, std::array<double, 3> *&min_ptr, double distance = 0.25)
{
    if (points.size() == 0)
    {
        min_ptr = nullptr;
        return false;
    }
    min_ptr = &(*std::min_element(points.begin(), points.end(),
                                  [point](const std::array<double, 3> &p1, const std::array<double, 3> &p2)
                                  {
                                      double dist1 = std::pow(point[0] - p1[0], 2) +
                                                     std::pow(point[1] - p1[1], 2) +
                                                     std::pow(point[2] - p1[2], 2);
                                      double dist2 = std::pow(point[0] - p2[0], 2) +
                                                     std::pow(point[1] - p2[1], 2) +
                                                     std::pow(point[2] - p2[2], 2);
                                      return dist1 < dist2;
                                  } // Find the closest point
                                  ));
    const std::array<double, 3> &min = *min_ptr;
    return std::pow(point[0] - min[0], 2) + std::pow(point[1] - min[1], 2) + std::pow(point[2] - min[2], 2) < distance * distance;
}

auto start = std::chrono::steady_clock::now();
void publish_object_detections_csv(const object_detection::detection_arr &msg)
{
    object_detection::string_arr out_msg;
    for (const auto &det : msg.data)
    {
        auto &detection = detections[det.type];
        detection.push_back({det.x, det.y, det.z});
        if (detection.size() > max_detection_count)
            detection.erase(detection.begin());
    }
    std::chrono::duration<double> elapsed_seconds = std::chrono::steady_clock::now() - start;
    if (elapsed_seconds.count() < 1. / clustering_freq)
        return;

    start = std::chrono::steady_clock::now();
    for (size_t i = 0; i < obj_class_count; i++) // loop over person, door etc. for clustering
    {
        if (detections[i].empty())
            continue; // no points have been found in this class so clustering isn't necessary, go to the next object type

        arma::mat data_set = arma::mat(detections[i][0].data(), 3, detections[i].size(), false);
        arma::Row<size_t> assignments;
        arma::mat centroids;
        size_t numCluster = models[i].Cluster(data_set, assignments, centroids);
        if (numCluster == 0)
            continue; // All the data is noise

        for (size_t j = 0; j < numCluster; j++) // loop through the clusters of points.
        {
            // Assignments contains an array e.g. [0,1,1,0] which means points at index 0 and 3 form a cluster and points at index 1 and 2 do.
            size_t count = std::count(assignments.begin(), assignments.end(), j);                 // The number of points in the jth cluster
            std::array<double, 3> centroid = {centroids(0, j), centroids(1, j), centroids(2, j)}; // Store the location of the jth centroid in an array
            std::array<double, 3> *min_ptr = nullptr;
            if (count > point_count_threshold && !isclose(centroid, positions[i], min_ptr))
            {
                // There are enough points in this cluster and no object of this time has previously been found here
                bool ishazmat = i >= 3;

                // Make a string in the csv format required
                auto format = "%s,%.3f,%.3f,%.3f,Arbie,T,%s";
                auto size = std::snprintf(nullptr, 0, format, ishazmat ? detection::classes[i] : "", centroid[0], centroid[1], centroid[2], ishazmat ? "Hazmat" : detection::classes[i]); // get the size of the output string
                std::string output(size, '\0');                                                                                                                                           // make a buffer for the output string
                std::sprintf(&output[0], format, ishazmat ? detection::classes[i] : "", centroid[0], centroid[1], centroid[2], ishazmat ? "Hazmat" : detection::classes[i]);              // formatted text into the output string
                // publish the string
                printf("%s\n", output.c_str());
                out_msg.data.push_back(output);
                positions[i].push_back(centroid);
            }
            else if (min_ptr)
            {
                // There are enough points to form a detection but this centroid is close to one that has already been seen
                // In this case, maybe the object has moved slighlty or maybe more data means the new centroid position is more accurate
                // Replace the previous centroid's data with the new one
                *min_ptr = centroid;
            }
        }
    }
    if (out_msg.data.size())
        detection_pub.publish(out_msg);

    if (rviz)
    {
        auto stamp = ros::Time::now();
        for (size_t i = 0; i < detections.size(); i++)
        {
            const auto &dets = detections[i];
            if (dets.size() == 0)
                continue;
            visualization_msgs::Marker points;
            points.header.frame_id = "/my_frame";
            points.header.stamp = stamp;
            points.ns = detection::classes[i];
            points.action = visualization_msgs::Marker::ADD;
            points.pose.orientation.w = 1.0;

            points.id = 0;

            points.type = visualization_msgs::Marker::SPHERE_LIST;

            points.scale.x = 5e-3;
            points.scale.y = 5e-3;
            points.scale.z = 5e-3;

            auto colour = colour_map[i];
            std_msgs::ColorRGBA c;
            c.r = colour[0];
            c.g = colour[1];
            c.b = colour[2];
            c.a = 1.0;

            for (const auto &det : dets)
            {
                geometry_msgs::Point p; // rviz coordinate system
                p.x = -det[1];
                p.y = det[0];
                p.z = det[2];
                points.points.push_back(p);
                points.colors.push_back(c);
            }
            marker_pub.publish(points);
        }
        for (size_t i = 0; i < positions.size(); i++)
        {
            visualization_msgs::Marker text_marker;
            text_marker.header.frame_id = "/my_frame";
            text_marker.header.stamp = stamp;
            text_marker.ns = detection::classes[i];
            text_marker.action = visualization_msgs::Marker::ADD;
            text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            text_marker.scale.z = 0.08;
            text_marker.text = detection::classes[i];

            std_msgs::ColorRGBA col;
            col.r = colour_map[i][0];
            col.g = colour_map[i][1];
            col.b = colour_map[i][2];
            col.a = 1.0;
            text_marker.color = col;

            geometry_msgs::Point p;
            int marker_id = 1;
            for (size_t j = 0; j < positions[i].size(); j++)
            {
                text_marker.id = marker_id++;
                p.x = -positions[i][j][1];
                p.y = positions[i][j][0];
                p.z = positions[i][j][2] + 0.05; // 10 cm above
                text_marker.pose.position = p;
                marker_pub.publish(text_marker);
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DBSCANner");
    {
        ros::NodeHandle nh("~");
        nh.param("rviz", rviz, false);
    }
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("detections3d", 30, publish_object_detections_csv);
    std::cerr << (rviz ? "" : "NOT ") << "VISUALISING MARKERS" << std::endl;
    detection_pub = n.advertise<object_detection::string_arr>("/detection/text_csv", 20);
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 100);
    ros::spin();
}