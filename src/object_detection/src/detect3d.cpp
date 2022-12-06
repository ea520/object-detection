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
static const std::array<cv::Scalar, 22> colors = {
    cv::Scalar(230, 25, 75),
    cv::Scalar(60, 180, 75),
    cv::Scalar(255, 225, 25),
    cv::Scalar(0, 130, 200),
    cv::Scalar(245, 130, 48),
    cv::Scalar(145, 30, 180),
    cv::Scalar(70, 240, 240),
    cv::Scalar(240, 50, 230),
    cv::Scalar(210, 245, 60),
    cv::Scalar(250, 190, 212),
    cv::Scalar(0, 128, 128),
    cv::Scalar(220, 190, 255),
    cv::Scalar(170, 110, 40),
    cv::Scalar(255, 250, 200),
    cv::Scalar(128, 0, 0),
    cv::Scalar(170, 255, 195),
    cv::Scalar(128, 128, 0),
    cv::Scalar(255, 215, 180),
    cv::Scalar(0, 0, 128),
    cv::Scalar(128, 128, 128),
    cv::Scalar(255, 255, 255),
    cv::Scalar(0, 0, 0)};

rs2_intrinsics intrinsics;
bool show_boxes = true;

yolo_net net;
QR_scanner scanner;
image_transport::Publisher colour_pub;
ros::Publisher vis_pub;
util::cpu_timer_t cpu_timer;
tracker_t tracker;
void camera_callback(const sensor_msgs::ImageConstPtr &colour_img, const sensor_msgs::ImageConstPtr &depth_img, const nav_msgs::OdometryConstPtr &odom)
{
    ros::Rate rate(10.);
    rate.reset();
    cv::Mat frame = util::toCVMat(colour_img);
    cv::Mat depth_frame = util::toCVMat(depth_img);
    std::vector<object2d> yolo_output, QR_output;
    auto detect_yolo = [&]()
    {
        yolo_output = net.detect(frame);
    };
    auto detect_qr = [&]()
    {
        QR_output = scanner.detect(frame);
    };
    std::thread t2(detect_yolo);
    std::thread t1(detect_qr);
    std::chrono::high_resolution_clock::duration inference_time;
    {
        util::scope_timer t(&inference_time);
        t1.join();
        t2.join();
    }
    using namespace std::chrono;
    auto output_2d = yolo_output;
    output_2d.insert(output_2d.end(), QR_output.begin(), QR_output.end());
    auto with_depths = get_2d_with_depths(output_2d, depth_frame);
    Eigen::Vector3f camera_pos(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    Eigen::Quaternionf camera_rotation(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
    // Pretty sure the matrix is always a multiple of the identity
    Eigen::Matrix3f sigma = Eigen::Matrix3f::Identity() * odom->pose.covariance[0];
    camera_pose_t camera_pose{camera_pos, camera_rotation, sigma};
    std::vector<observation_3d> points = get_3d(with_depths, camera_pose);
    tracker.update(points);
    auto tracked_objs = tracker.get_objects();
    visualization_msgs::MarkerArray markerarr;
    for (const auto &obj : tracked_objs)
    {
        markerarr.markers.push_back(obj.get_object_marker());
        markerarr.markers.push_back(obj.get_covariance_marker());
        if (obj.type == object_type::QR)
            markerarr.markers.push_back(obj.get_QR_text_marker());
    }
    vis_pub.publish(markerarr);
    draw_boxes(output_2d, frame);
    colour_pub.publish(util::toImageMsg(frame));
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

    auto camera_info_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/d400/aligned_depth_to_color/camera_info", nh, ros::Duration(10));
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

    message_filters::Subscriber<sensor_msgs::Image> colour_sub(nh, "/d400/color/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/d400/aligned_depth_to_color/image_raw", 1);
    message_filters::Subscriber<nav_msgs::Odometry> gyro_sub(nh, "/t265/odom/sample", 10);

    using sync_type = message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, nav_msgs::Odometry>;
    message_filters::Synchronizer<sync_type> sync(sync_type(10), colour_sub, depth_sub, gyro_sub);

    sync.registerCallback(boost::bind(&camera_callback, _1, _2, _3));
    std::string path = ros::package::getPath("object_detection") + "/../../resources/";
    std::string bin_path{path + "best.bin"}, xml_path{path + "best.xml"};
    std::string txt_path = path + "classes.txt";
    std::cout << "BIN PATH: " << bin_path << std::endl;

    int target = GPU ? cv::dnn::DNN_TARGET_OPENCL_FP16 : cv::dnn::DNN_TARGET_CPU;

    net = yolo_net(bin_path, xml_path, txt_path, target, 0.8);
    if (show_boxes)
    {
        image_transport::ImageTransport it(nh);
        colour_pub = it.advertise("camera/detections", 3);
    }
    vis_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker", 0);
    ros::spin();
}
