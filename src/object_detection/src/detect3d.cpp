#include <fstream>
#include <zbar.h>
#include <opencv2/dnn/dnn.hpp>
#include <opencv2/core.hpp>
#include <chrono>
#include <algorithm>
#include <numeric>
#include <vector>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <thread>
#include <librealsense2/rs.hpp>
#include <ros/package.h>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/time_synchronizer.h>
#include <thread>
#include <mutex>
#include <object_detection/string_arr.h>
#include <object_detection/detection_arr.h>
#include "yolo.hpp"
#include "util.hpp"
#include <message_filters/sync_policies/approximate_time.h>

rs2_intrinsics intrinsics;
bool show_boxes = true;

yolo::yolo_net net;
zbar::ImageScanner scanner;
image_transport::Publisher colour_pub;
ros::Publisher detection_pub, qr_pub;

util::circular_buffer<short, 20> inference_times;
util::circular_buffer<short, 100> cpu_times;
util::Rate rate(std::chrono::milliseconds(100));
util::cpu_timer_t cpu_timer;
const matrix3d depth_camera_rotation = {
    {0.0016619, 0.07326312, 0.99731126},
    {-0.99975752, -0.0217769, 0.00326572},
    {0.0219576, -0.99707486, 0.07320917}};
const vec3d depth_camera_displacement(-0.01162073, 0.04516913, 0.0230702);

void camera_callback(const sensor_msgs::ImageConstPtr &colour_img, const sensor_msgs::ImageConstPtr &depth_img, const nav_msgs::OdometryConstPtr &odom)
{
    auto tracker_rotation = matrix3d(odom->pose.pose.orientation);
    auto tracker_pos = vec3d(odom->pose.pose.position);

    cv::Mat frame = util::toCVMat(colour_img);
    cv::Mat depth_frame = util::toCVMat(depth_img);
    std::vector<yolo::Detection> output;

    cv::Mat grey;
    cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);
    zbar::Image image(grey.cols, grey.rows, "Y800", (uchar *)grey.data, grey.cols * grey.rows);

    auto detect_yolo = [&]()
    {
        output = net.detect(frame);
    };
    auto detect_qr = [&]()
    { scanner.scan(image); };
    std::thread t2(detect_yolo);
    std::thread t1(detect_qr);
    std::chrono::high_resolution_clock::duration inference_time;
    {
        util::scope_timer t(&inference_time);
        t1.join();
        t2.join();
    }
    using namespace std::chrono;
    inference_times.add_value(duration_cast<milliseconds>(inference_time).count());
    object_detection::detection_arr out_msg;

    auto get_world_point = [&](float x, float y) -> vec3d
    {
        float depth = (float)depth_frame.at<u_int16_t>((int)y, (int)x);
        if (depth < 10 || depth > 3000) // only sensitive to a certain depth range
            return vec3d{0., 0., 0.};
        float camera_point[3], pixel[] = {x, y};
        rs2_deproject_pixel_to_point(camera_point, &intrinsics, pixel, depth * 1e-3); // depth in m
        vec3d camera_point_vec(camera_point);
        vec3d world_point = depth_camera_rotation.dot(camera_point_vec);

        world_point += depth_camera_displacement;
        // This would be the point relative to the tracking camera if it hadn't rotated or translated
        world_point = tracker_rotation.dot(world_point);
        // This is the point relative to the tracking camera if it hadn't translated

        world_point.x += tracker_pos.x;
        world_point.y += tracker_pos.y;
        world_point.z += tracker_pos.z;
        return world_point;
    };
    static std::vector<std::string> known_codes;
    object_detection::string_arr qr_detection_text;
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
    {
        const std::string text = symbol->get_data();

        std::vector<int> xs, ys;
        for (int i = 0; i < symbol->get_location_size(); i++)
        {
            xs.push_back(symbol->get_location_x(i));
            ys.push_back(symbol->get_location_y(i));
        }

        int x1 = *std::min_element(xs.begin(), xs.end());
        int y1 = *std::min_element(ys.begin(), ys.end());
        int x2 = *std::max_element(xs.begin(), xs.end());
        int y2 = *std::max_element(ys.begin(), ys.end());
        if (x1 == 0 || y1 == 0 || x2 == grey.cols || y2 == grey.rows)
            continue;
        if (show_boxes)
        {
            cv::rectangle(frame, cv::Rect(cv::Point2i(x1, y1), cv::Point2i(x2, y2)), {255, 0, 0}, 3);
            cv::putText(frame, symbol->get_data(), {x1, y1}, cv::FONT_HERSHEY_PLAIN, 1.5, {0, 0, 255}, 2);
        }

        if (std::find(known_codes.begin(), known_codes.end(), symbol->get_data()) == known_codes.end())
        {
            auto world_point = get_world_point((x1 + x2) * 0.5f, (y1 + y2) * 0.5f);
            if (world_point.x == 0. && world_point.y == 0. && world_point.z == 0.)
                continue;
            auto format = "%s,%.3f,%.3f,%.3f,Arbie,T,%s";
            auto size = std::snprintf(nullptr, 0, format, symbol->get_data().c_str(), world_point.x, world_point.y, world_point.z, "QRCode"); // get the size of the output string
            std::string output(size, '\0');                                                                                                   // make a buffer for the output string
            std::sprintf(&output[0], format, symbol->get_data().c_str(), world_point.x, world_point.y, world_point.z, "QRCode");              // formatted text into the output string
            // publish the string
            printf("%s\n", output.c_str());
            qr_detection_text.data.push_back(output);
            known_codes.push_back(symbol->get_data());
        }
    }
    if (qr_detection_text.data.size())
        qr_pub.publish(qr_detection_text);

    int detections = output.size();

    // get depth image
    if (show_boxes)
    {
        yolo::draw_boxes(output, frame, net);
    }
    for (int i = 0; i < detections; ++i)
    {

        auto detection = output[i];
        auto box = detection.box;
        auto classId = detection.class_id;

        if (box.x == 0 || box.y == 0 || box.x + box.width == frame.cols || box.y + box.height == frame.rows)
            continue;

        auto world_point = get_world_point(box.x - box.width / 2, box.y - box.height / 2);
        // std::cerr << world_point << std::endl;
        if (world_point.x == 0. && world_point.y == 0. && world_point.z == 0.)
            continue;

        object_detection::detection d;
        d.x = world_point.x;
        d.y = world_point.y;
        d.z = world_point.z;
        d.conf = detection.confidence;
        d.type = classId;
        out_msg.data.push_back(d);
    }
    if (out_msg.data.size())
        detection_pub.publish(out_msg);
    if (show_boxes)
    {

        cv::copyMakeBorder(frame, frame, 40, 0, 0, 0, cv::BORDER_CONSTANT, {0, 0, 0});
        cpu_times.add_value(cpu_timer.getCPUPercentage());
        cv::putText(frame, std::string("CPU ") + std::to_string(cpu_times.get_average()) + "%", {10, 30}, cv::FONT_HERSHEY_DUPLEX, 0.7, {100, 200, 0}, 1);

        cv::putText(frame,
                    std::string("Inference time: ") + std::to_string(inference_times.get_average()) + "ms",
                    {390, 30}, cv::FONT_HERSHEY_DUPLEX, 0.7, {100, 200, 0}, 1);
        cv::putText(frame, std::string("Target FPS: ") + std::to_string(1'000'000 / rate.get_target().count()), {190, 30}, cv::FONT_HERSHEY_DUPLEX, 0.7,
                    rate.wait() ? cv::Scalar{50, 50, 200} : cv::Scalar{100, 200, 0}, 1);
        sensor_msgs::Image colour_msg = util::toImageMsg(frame);
        colour_pub.publish(colour_msg);
    }
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
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

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
    std::string txt_path = ros::package::getPath("object_detection") + "classes.txt";
    std::cout << "BIN PATH: " << bin_path << std::endl;

    int target = GPU ? cv::dnn::DNN_TARGET_OPENCL_FP16 : cv::dnn::DNN_TARGET_CPU;

    net = yolo::yolo_net(bin_path, xml_path, txt_path, target, 0.8);

    if (show_boxes)
    {
        image_transport::ImageTransport it(nh);
        colour_pub = it.advertise("camera/detections", 3);
    }
    detection_pub = nh.advertise<object_detection::detection_arr>("detections3d", 3);
    qr_pub = nh.advertise<object_detection::string_arr>("/detection/text_csv", 3);
    ros::spin();
}
