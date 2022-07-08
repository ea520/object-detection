#include <fstream>
#include <zbar.h>
#include <image_transport/image_transport.h>
#include <librealsense2/rs.hpp>
#include <message_filters/subscriber.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "util.h"
#include "vec3d.h"
#include <chrono>
#include <ros/package.h>
zbar::ImageScanner scanner;
rs2_intrinsics intrinsics;
std::ofstream csv_file;

void camera_callback(const sensor_msgs::ImageConstPtr &colour_img, const sensor_msgs::ImageConstPtr &depth_img, const nav_msgs::OdometryConstPtr &odom)
{
    const geometry_msgs::Quaternion &tracker_rotation = odom->pose.pose.orientation;
    const geometry_msgs::Point &tracker_pos = odom->pose.pose.position;
    cv::Mat frame = util::toCVMat(colour_img);
    cv::Mat depth_frame = util::toCVMat(depth_img);
    cv::Mat grey;
    cv::cvtColor(frame, grey, cv::COLOR_BGR2GRAY);

    zbar::Image image(grey.cols, grey.rows, "Y800", (uchar *)grey.data, grey.cols * grey.rows);
    scanner.scan(image);

    auto get_world_point = [depth_frame](float x, float y) -> geometry_msgs::Point
    {
        float pixel[] = {std::roundf(x), std::roundf(y)};
        float depth = (float)depth_frame.at<u_int16_t>((int)pixel[1], (int)pixel[0]);
        if (depth < 200 || depth > 1500)
        {
            geometry_msgs::Point p;
            p.x = p.y = p.z = 0.;
            return p;
        }
        static int i;
        std::cout << i++ << " " << depth << std::endl;
        float camera_point[3];
        rs2_deproject_pixel_to_point(camera_point, &intrinsics, pixel, depth * 1e-3);
        geometry_msgs::Point output;
        output.x = camera_point[0];
        output.y = camera_point[1];
        output.z = camera_point[2];
        return output;
    };
    for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
    {
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
        float x = (x1 + x2) * 0.5f, y = (y1 + y2) * 0.5f;
        vec3d world_point = (vec3d)get_world_point(x, y);
        if (world_point.x != 0. && world_point.y != 0. && world_point.z != 0.)
        {
            // Assume errors in depth are proportional to the depth
            // Assume also that the main source of error is the deprojection
            // i.e. the tracker position and orientation are well known

            // If the QR code is at the centre of the camera, there is no
            // error perpendicular to the camera
            // If the QR code is at edge of the field of view, the error is largest

            // If the error in the depth is dz
            // And the angle to the z axis is theta
            // The error in the distance is to the camera
            // is dz sec(theta)
            //
            std::ostringstream ss;
            ss
                << world_point.x << ","
                << world_point.y << ","
                << world_point.z << ","
                << tracker_pos.x << ","
                << tracker_pos.y << ","
                << tracker_pos.z << ","
                << tracker_rotation.x << ","
                << tracker_rotation.y << ","
                << tracker_rotation.z << ","
                << tracker_rotation.w << std::endl;
            // std::cout << ss.str();
            csv_file << ss.str();
            csv_file.flush();
        }
        using namespace std::chrono_literals;
        std::this_thread::sleep_for(500ms);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collect");
    ros::NodeHandle nh;
    std::string path = ros::package::getPath("depth_calibration");

    csv_file = std::ofstream(path + "/outputs.csv", std::ofstream::out);
    std::cout << path << std::endl;
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);

    auto camera_info_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>("/d400/aligned_depth_to_color/camera_info", nh, ros::Duration(10));
    intrinsics.width = camera_info_msg->width;
    intrinsics.height = camera_info_msg->height;
    intrinsics.ppx = camera_info_msg->K[2];
    intrinsics.ppy = camera_info_msg->K[5];
    intrinsics.fx = camera_info_msg->K[0];
    intrinsics.fy = camera_info_msg->K[4];
    if (camera_info_msg->distortion_model == "plumb_bob")
    {
        intrinsics.model = RS2_DISTORTION_BROWN_CONRADY;
        std::cerr << "PLUMB BOB MODEL but using None anyway" << std::endl;
        std::cerr << *camera_info_msg << std::endl;
    }
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
    ros::spin();
}