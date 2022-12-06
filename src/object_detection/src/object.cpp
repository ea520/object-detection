#include "object.hpp"
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <Eigen/Geometry>
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

Eigen::Vector3f get_normal(std::vector<Eigen::Vector3f> points, float *mse)
{
    // It is assumed the depth is on the z axis
    // This then does least squares on the z error
    // Fitting the data to the plane ax + by = z + d
    // Subtracting the means from x,y and z  a<x> + b<y> = <z>
    // a can be found with least squares
    if (points.size() < 10)
        throw std::runtime_error("Not enough points to calculate plane (need at least 10)");

    // Reinterpret the data as an EIGEN matrix
    typedef Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Mat3x;
    Mat3x mat = Eigen::Map<Mat3x>(&points[0][0], points.size(), 3);

    // Subtract the average x value from each point's x coord etc.
    mat.rowwise() -= mat.colwise().mean();

    // Solve Ax = b for the least squares
    // With good sampling, x and y would be independent so their covariance
    // should be 0 making the matrix diagonal (trivial to invert)
    // It turns out, just doing the inversion isn't much harder computationally
    // and it allows for strange sampling of points
    Eigen::Matrix2f A;
    A(0, 0) = mat.col(0).dot(mat.col(0));
    A(1, 0) = A(0, 1) = mat.col(0).dot(mat.col(1));
    A(1, 1) = mat.col(1).dot(mat.col(1));
    Eigen::Vector2f b{
        mat.col(0).dot(mat.col(2)),
        mat.col(1).dot(mat.col(2)),
    };

    Eigen::Vector3f n;
    n(2) = -1.; // Normal points somewhat towards the camera
    n.head<2>() = A.ldlt().solve(b);
    auto errors = mat * n;
    // std::cout << errors * 100.f << std::endl;
    *mse = errors.dot(errors) / errors.rows();
    return n.normalized();
}

void draw_boxes(const std::vector<object2d> &detections, cv::Mat &frame)
{
    for (size_t i = 0; i < detections.size(); ++i)
    {

        const auto &detection = detections[i];
        const cv::Point2i top_left{detection.u - detection.w / 2, detection.v - detection.w / 2};
        const cv::Point2i bottom_right{detection.u + detection.w / 2, detection.v + detection.w / 2};
        const cv::Rect box{top_left, bottom_right};
        auto classId = (std::hash<std::string>{}(detection.text) + (size_t)detection.type) % colors.size();
        const auto &color = colors[classId];
        char buff[6];
        snprintf(buff, sizeof(buff), " %.2f", detection.confidence);

        cv::rectangle(frame, box, color, 3);
        cv::rectangle(frame, cv::Point(box.x, box.y - 10), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
        cv::Scalar textcolor;
        textcolor[0] = color[0] < 128 ? 255 : 0;
        textcolor[1] = color[1] < 128 ? 255 : 0;
        textcolor[2] = color[2] < 128 ? 255 : 0;
        cv::putText(frame, detection.text + buff, cv::Point(box.x, box.y), cv::FONT_HERSHEY_PLAIN, 1.0, textcolor);
    }
}
// TODO: Change this
static constexpr rs2_intrinsics intrinsics{
    640,
    480,
    322.505524f,
    249.334457f,
    616.423279f,
    616.756714f,
    RS2_DISTORTION_INVERSE_BROWN_CONRADY,
    {0.f, 0.f, 0.f, 0.f, 0.f}

};

float mse_tol(object_type type)
{
    switch (type)
    {
    case object_type::HAZMAT:
    case object_type::QR:
        return 0.03 * 0.03;
    case object_type::DOOR:
        return 0.03 * 0.03;
    default:
        return INFINITY;
    }
}
float angle_tol(object_type type)
{
    switch (type)
    {
    case object_type::DOOR:
        return M_PI * 2. / 180.;
    default:
        return INFINITY;
    }
}

inline Eigen::Vector3f deproject(std::array<float, 2> uv, float depth)
{
    Eigen::Vector3f ret;
    rs2_deproject_pixel_to_point((float *)&ret, &intrinsics, uv.data(), depth);
    return ret;
}

std::vector<object2d_with_depth> get_2d_with_depths(const std::vector<object2d> &detections, cv::Mat &depth)
{
    std::vector<object2d_with_depth> ret;
    for (const auto &detection : detections)
    {
        object2d_with_depth det;
        det.depth = depth.at<uint16_t>(detection.v, detection.u) / 1000.; // Convert mm to metres
        if (det.depth < 0.1 || det.depth > 2.0)                           // only usable between 10cm and 2.0 m
            continue;
        det.u = detection.u;
        det.v = detection.v;
        det.confidence = detection.confidence;
        det.text = detection.text;
        det.type = detection.type;
        std::vector<Eigen::Vector3f> points;
        static constexpr size_t N = 64 * 64;
        // smaller bounding box;
        int width = 0.8 * detection.w;
        int height = 0.8 * detection.h;
        int u = det.u - width / 2;  // left
        int v = det.v - height / 2; // top
        size_t iterations = 0;
        // Find N points. Give up is the map is full of holes
        while (points.size() < N && iterations++ < 2 * N)
        {
            int x = u + rand() % width;
            int y = v + rand() % height;
            float dist = depth.at<uint16_t>(y, x) / 1000.;
            if (dist < 0.1f)
                continue;
            float positionsf[3];
            float coords[] = {(float)x, (float)y};
            rs2_deproject_pixel_to_point(positionsf, &intrinsics, coords, dist);
            points.emplace_back(positionsf[0], positionsf[1], positionsf[2]);
        }
        if (points.size() == N)
        {
            float mse;
            det.normal = get_normal(points, &mse);
            if (mse < mse_tol(det.type))
                ret.push_back(det);
        }
    }
    return ret;
}

inline constexpr float get_object_variance(object_type type)
{
    switch (type)
    {
    case object_type::QR:
        return .01 * .01; // Can locate the centre of the QR code to within 1cm
    case object_type::HAZMAT:
        return .01 * .01; //
    case object_type::DOOR:
        return .4 * .4; //
    case object_type::PERSON:
        return .4 * .4; //
    case object_type::FIRE_EXTINGUISHER:
        return .4 * .4; //
    default:
        return NAN;
    }
}

inline Eigen::Matrix3f get_jacobian(std::array<float, 2> uv, float depth)
{
    Eigen::Vector3f ddu = deproject({uv[0] + 0.5f, uv[1]}, depth) - deproject({uv[0] - 0.5f, uv[1]}, depth);
    Eigen::Vector3f ddv = deproject({uv[0], uv[1] + 0.5f}, depth) - deproject({uv[0], uv[1] - 0.5f}, depth);
    const float epsilon = 0.05f * depth; // use 1 cm for the derivative
    Eigen::Vector3f ddd = (deproject({uv[0], uv[1]}, depth + epsilon / 2) - deproject({uv[0], uv[1]}, depth - epsilon / 2)) / epsilon;
    Eigen::Matrix3f ret;
    ret.col(0) << ddu;
    ret.col(1) << ddv;
    ret.col(2) << ddd;
    return ret;
}
Eigen::Matrix3f get_rotation()
{
    Eigen::Matrix3f P = Eigen::Matrix3f::Zero();
    P.row(0) << 0.0162226, 0.0753285, 0.997027;
    P.row(1) << -0.999617, -0.0211404, 0.017862;
    P.row(2) << 0.022423, -0.996935, 0.0749567;
    return P;
}

std::vector<observation_3d> get_3d(const std::vector<object2d_with_depth> &detections, const camera_pose_t &camera_pose)
{
    std::vector<observation_3d> ret;
    static const Eigen::Matrix3f Q0 = get_rotation();
    static const Eigen::Vector3f t0 = Eigen::Vector3f{0.02104983, 0.05304557, 0.05261957};
    const Eigen::Matrix3f Q{camera_pose.rotation};

    for (const auto &det : detections)
    {
        observation_3d obj;
        Eigen::Vector3f pos_wrt_camera = deproject({(float)det.u, (float)det.v}, det.depth + 0.1); // Add 10 cm to get a point somewhere inside the fire extinguisher rather than the surface
        obj.position = Q * (Q0 * pos_wrt_camera + t0) + camera_pose.translation;
        obj.covariance = Eigen::Matrix3f::Identity() * get_object_variance(det.type) + camera_pose.covariance; // Just the position covariance, not the angle
        obj.type = det.type;
        obj.text = det.text;
        obj.confidence = det.confidence;
        obj.normal = Q * Q0 * det.normal;
        auto normal = obj.normal;
        auto normal_horizontal = normal;
        normal_horizontal.z() = 0;
        normal_horizontal.normalize();
        float angle = acosf(normal_horizontal.dot(normal));
        if (angle < angle_tol(det.type))
            ret.push_back(obj);
    }
    return ret;
}
