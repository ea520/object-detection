#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <thread>
#include <librealsense2/rs.hpp>
#include <ros/package.h>
#include "yolo.hpp"
#include "util.hpp"
#include "QR.hpp"
#include "tracker.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <boost/circular_buffer.hpp>
#include <boost/timer/timer.hpp>
#include <std_msgs/String.h>
#include <numeric>
#include <fstream>
// to store camera properies e.g. focal length
rs2_intrinsics intrinsics;

// process at most 10 images per sec
static constexpr double target_rate = 8.;

// for detecting non-QR objects
yolo_net net;
QR_scanner scanner;
tracker_t tracker;

image_transport::Publisher colour_pub;
ros::Publisher vis_pub, text_pub;

// Datatype to store the most recent 40 cpu percentages
boost::circular_buffer<float> inference_times(40);

void camera_callback(const sensor_msgs::ImageConstPtr &colour_img, const sensor_msgs::ImageConstPtr &depth_img, const nav_msgs::OdometryConstPtr &odom)
{
    // Limit the rate of inference 30 fps is probably un-necessary
    ros::Rate rate(target_rate);

    // Convert images to opencv format
    cv::Mat frame = util::toCVMat(colour_img);
    cv::Mat depth_frame = util::toCVMat(depth_img);

    // Find all the objects, their positions and sizes
    std::vector<object2d> yolo_output, QR_output;
    auto detect_yolo = [&yolo_output, &frame]()
    {
        yolo_output = net.detect(frame);
    };
    auto detect_qr = [&QR_output, &frame]()
    {
        QR_output = scanner.detect(frame);
    };
    {
        // Do the detection of objects and QRs on 2 separate threads
        auto t0 = std::chrono::high_resolution_clock::now();
        boost::timer::cpu_timer _timer;
        _timer.start();
        std::thread t1(detect_qr);
        std::thread t2(detect_yolo);
        t1.join();
        t2.join();
        _timer.stop();
        inference_times.push_back(_timer.elapsed().user * 1e-9f);
    }
    // Combine QRs and objects into one array
    auto output_2d = yolo_output;
    output_2d.insert(output_2d.end(), QR_output.begin(), QR_output.end());

    // Add the depth and surface normals of each object
    auto with_depths = get_2d_with_depths(output_2d, depth_frame, intrinsics);

    // Convert the camera position and rotation to the eqivalents in the maths library
    Eigen::Vector3f camera_pos(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    Eigen::Quaternionf camera_rotation(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);

    // Pretty sure the matrix is always a multiple of the identity
    Eigen::Matrix3f sigma = Eigen::Matrix3f::Identity() * odom->pose.covariance[0];

    camera_pose_t camera_pose{camera_pos, camera_rotation, sigma};

    // Use the camera's pose and covariance to get noisy observations of the objects
    std::vector<observation_3d> observations = get_3d(with_depths, camera_pose, intrinsics);

    // Add these observations to the object tracker. This will improve estimates of the
    // objects' positions and find out if any of the observations correspond to previously unseen objects
    tracker.update(observations);

    // Get all of the objects that the tracker is aware of. Some may not be in the camera frame.
    auto tracked_objs = tracker.get_objects();
    std::sort(tracked_objs.begin(), tracked_objs.end());
    // Send a message for rviz to render the objects
    visualization_msgs::MarkerArray markerarr;
    for (const auto &obj : tracked_objs)
    {
        markerarr.markers.push_back(obj.get_object_marker(camera_rotation));
        markerarr.markers.push_back(obj.get_covariance_marker());
        if (obj.type == object_type::QR)
            markerarr.markers.push_back(obj.get_QR_text_marker());
    }

    // Takes the average of an array
    auto average = [](auto arr) -> float
    {
        auto sum = std::accumulate(arr.begin(), arr.end(), 0.f);
        return sum / arr.size();
    };

    float average_inference_time = average(inference_times);
    constexpr static size_t buffsize = 200; // more than enough
    char buff[buffsize];

    // Create a string with the relevant information then publish the string
    snprintf(buff, buffsize, "Inference (user) time %5.0f ms | Target rate: %.0f Hz",
             1000.f * average_inference_time, 1. / rate.expectedCycleTime().toSec());
    std_msgs::String text_msg;
    text_msg.data = buff;

    // Draw the bounding boxes onto the image
    draw_boxes(output_2d, frame);

    colour_pub.publish(util::toImageMsg(frame));
    text_pub.publish(text_msg);
    vis_pub.publish(markerarr);
    rate.sleep();
}
int main(int argc, char **argv)
{
    bool GPU;
    ros::init(argc, argv, "detect");
    {
        ros::NodeHandle nh("~");
        nh.param("gpu", GPU, false);
        std::cerr << (GPU ? "USING GPU" : "USING CPU") << std::endl;
    }
    ros::NodeHandle nh;
    // Initialise the monitor for CPU usage
    cpu_monitor::init();

    // Get the camera parameters
    auto camera_info_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/d400/aligned_depth_to_color/camera_info", nh, ros::Duration(30));
    intrinsics.width = camera_info_msg->width;
    intrinsics.height = camera_info_msg->height;
    intrinsics.ppx = camera_info_msg->K[2];
    intrinsics.ppy = camera_info_msg->K[5];
    intrinsics.fx = camera_info_msg->K[0];
    intrinsics.fy = camera_info_msg->K[4];
    if (camera_info_msg->distortion_model == "plumb_bob")
        intrinsics.model = RS2_DISTORTION_BROWN_CONRADY;
    else
        intrinsics.model = RS2_DISTORTION_NONE;

    const auto &D = camera_info_msg->D;
    assert(D.size() == sizeof(intrinsics.coeffs) / sizeof(*intrinsics.coeffs));
    for (size_t i = 0; i < sizeof(intrinsics.coeffs) / sizeof(*intrinsics.coeffs); i++)
        intrinsics.coeffs[i] = (float)D[i];

    // Subscribe to the relevant topics
    message_filters::Subscriber<sensor_msgs::Image> colour_sub(nh, "/d400/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/d400/aligned_depth_to_color/image_raw", 1);
    message_filters::Subscriber<nav_msgs::Odometry> gyro_sub(nh, "/t265/odom/sample", 10);
    {
        auto path = ros::package::getPath("object_detection") + "/detections.csv";
        std::ofstream output_file(path);
        using sysclock_t = std::chrono::system_clock;
        std::time_t now = sysclock_t::to_time_t(sysclock_t::now());
        char buf[16] = {0};
        std::strftime(buf, sizeof(buf), "%H:%M:%S", std::localtime(&now));
        auto time = std::string(buf);
        std::strftime(buf, sizeof(buf), "%Y-%m-%d", std::localtime(&now));
        auto date = std::string(buf);
        output_file << "pois" << std::endl;
        output_file << "1.2" << std::endl;
        output_file << date << std::endl;
        output_file << time << std::endl;
        output_file << "<MISSION>" << std::endl;
        output_file << "id,time,text,x,y,z,robot,mode,type" << std::endl;
    }
    // Setup the subscriber to sync the tracking and depth camera messages
    using sync_type = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, nav_msgs::Odometry>;
    message_filters::Synchronizer<sync_type> sync(sync_type(50), colour_sub, depth_sub, gyro_sub);
    sync.registerCallback(boost::bind(&camera_callback, _1, _2, _3));

    // Find the path to the YOLO model and make the net
    std::string path = ros::package::getPath("object_detection") + "/../../resources/yolo-model/";
    std::string bin_path{path + "best.bin"}, xml_path{path + "best.xml"};
    std::string txt_path = path + "classes.txt";
    int target = GPU ? cv::dnn::DNN_TARGET_OPENCL_FP16 : cv::dnn::DNN_TARGET_CPU;
    net = yolo_net(bin_path, xml_path, txt_path, target);

    // Publisher to show the objects in rviz
    vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 0);
    // Publisher show resource use
    text_pub = nh.advertise<std_msgs::String>("object_detection_cpu_info", 0);

    // Publisher for the objects with bounding boxes
    image_transport::ImageTransport it(nh);
    colour_pub = it.advertise("camera/detections", 3);

    ros::spin();
}
