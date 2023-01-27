#include "object.hpp"
#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <Eigen/Dense>

// 22 distinct colours
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

// Draw the bounding boxes to the image
void draw_boxes(const std::vector<object2d> &detections, cv::Mat &frame)
{
    for (size_t i = 0; i < detections.size(); ++i)
    {

        const auto &detection = detections[i];
        const cv::Point2i top_left{detection.u - detection.w / 2, detection.v - detection.h / 2};
        const cv::Point2i bottom_right{detection.u + detection.w / 2, detection.v + detection.h / 2};
        const cv::Rect box{top_left, bottom_right};
        // Objects of the same type and text should have the same colour. This hash function deterministically converts text to a number
        size_t color_index = (std::hash<std::string>{}(detection.text) + (size_t)detection.type) % colors.size();
        const auto &color = colors[color_index];

        // Create a string for the confidence
        char buff[6] = "\0";
        if (detection.type != object_type::QR)
            snprintf(buff, sizeof(buff), " %.2f", detection.confidence);

        // draw the bounding box
        cv::rectangle(frame, box, color, 3);

        // Draw a filled box. The text will go over this
        cv::rectangle(frame, cv::Point(box.x, box.y - 10), cv::Point(box.x + box.width, box.y), color, cv::FILLED);

        // Make the text colour dissimilar to the background colour
        cv::Scalar textcolor;
        textcolor[0] = color[0] < 128 ? 255 : 0;
        textcolor[1] = color[1] < 128 ? 255 : 0;
        textcolor[2] = color[2] < 128 ? 255 : 0;

        // Write the text
        cv::putText(frame, detection.text + buff, cv::Point(box.x, box.y), cv::FONT_HERSHEY_PLAIN, 1.0, textcolor);
    }
}

Eigen::Matrix3f get_normal_matrix(std::vector<Eigen::Vector3f> normals)
{
    // This creates a covariance matrix
    if (normals.size() < 10)
        throw std::runtime_error("Not enough points to calculate plane (need at least 10)");

    // Reinterpret the data as an EIGEN matrix
    typedef Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Mat3x;
    Mat3x mat = Eigen::Map<Mat3x>(&normals[0][0], normals.size(), 3);
    // Subtract the average x value from each point's x coord etc.
    mat.rowwise() -= mat.colwise().mean();
    return mat.transpose() * mat;
}

Eigen::Matrix3f get_axis_matrix(std::vector<Eigen::Vector3f> normals)
{
    if (normals.size() < 1)
        throw std::runtime_error("Not enough points to calculate plane (need at least 1)");

    // Reinterpret the data as an EIGEN matrix
    typedef Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Mat3x;
    Mat3x mat = Eigen::Map<Mat3x>(&normals[0][0], normals.size(), 3);

    return mat.transpose() * mat;
}

Eigen::Vector3f covar_to_orientation(const Eigen::Matrix3f &covar)
{
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> es(covar);
    Eigen::Index idx;
    es.eigenvalues().minCoeff(&idx); // sqrt of sum of square errors
    return es.eigenvectors().col(idx);
}

// For horizontal objects, what angle to the horizontal is tolerable
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

// Find the coordinates of pixel (u,v) at depth d relative to the camera
inline Eigen::Vector3f deproject(std::array<float, 2> uv, float depth, const rs2_intrinsics &intrinsics)
{
    Eigen::Vector3f ret;
    rs2_deproject_pixel_to_point((float *)&ret, &intrinsics, uv.data(), depth);
    return ret;
}

std::vector<Eigen::Vector3f> sample_depth_map(cv::Mat &depth, int u, int v, int w, int h, size_t N, const rs2_intrinsics &intrinsics)
{
    std::vector<Eigen::Vector3f> points;
    size_t iterations = 0;
    while (points.size() < N && iterations++ < 2 * N)
    {
        int x = u + rand() % w;
        int y = v + rand() % h;
        float dist = depth.at<uint16_t>(y, x) / 1000.;
        if (dist < 0.1f)
            continue;
        float positionsf[3];
        float coords[] = {(float)x, (float)y};
        rs2_deproject_pixel_to_point(positionsf, &intrinsics, coords, dist);
        points.emplace_back(positionsf[0], positionsf[1], positionsf[2]);
    }
    return points;
}

// Find the normals relative to the camera and depths for the objects
std::vector<object2d_with_depth> get_2d_with_depths(const std::vector<object2d> &detections, cv::Mat &depth, const rs2_intrinsics &intrinsics)
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
        det.distribution = detection.distribution;
        if (det.type == object_type::DOOR || det.type == object_type::HAZMAT || det.type == object_type::QR)
        {
            static constexpr size_t N = 64 * 64;
            // smaller bounding box;
            int width = 0.8 * detection.w;
            int height = 0.8 * detection.h;
            int u = det.u - width / 2;  // left
            int v = det.v - height / 2; // top
            std::vector<Eigen::Vector3f> points = sample_depth_map(depth, u, v, width, height, N, intrinsics);
            if (points.size() == N)
            {
                det.orientation_matrix = get_normal_matrix(points);
                ret.push_back(det);
            }
        }
        else if (det.type == object_type::FIRE_EXTINGUISHER)
        {
            // smaller bounding box;
            int width = 0.3 * detection.w;
            int height = 0.3 * detection.h;
            int u0 = det.u - width / 2;  // left
            int v0 = det.v - height / 2; // top
            int w = width / 2;
            int h = height / 2;
            static constexpr size_t N = 16 * 16;
            std::vector<Eigen::Vector3f> normals;
            // Take 4 normals near the centre
            // Find a matrix used to calculate the axis
            for (int i = 0; i < 2; i++)
                for (int j = 0; j < 2; j++)
                {
                    int u = u0 + i * w;
                    int v = v0 + j * h;
                    std::vector<Eigen::Vector3f> points = sample_depth_map(depth, u, v, w, h, N, intrinsics);
                    if (points.size() > N / 2)
                    {
                        normals.push_back(covar_to_orientation(get_normal_matrix(points)));
                    }
                }
            if (normals.size())
            {
                det.orientation_matrix = get_axis_matrix(normals);
            }
            else
                det.orientation_matrix *= NAN;
            ret.push_back(det);
        }
        else if (det.type == object_type::PERSON)
        {
            det.orientation_matrix = Eigen::Matrix3f::Identity();
            ret.push_back(det);
        }
    }
    return ret;
}

// X is parallel to the surface normal
// Z is approximately vertical if the normal is horizontal
// Y is perpendicular to X and Z
inline Eigen::DiagonalMatrix<float, 3> get_object_covariance(object_type type)
{
    float sx, sy, sz;
    switch (type)
    {
    case object_type::QR:
        sx = sy = sz = .01; // Can locate the centre of the QR code to within 1cm
        break;
    case object_type::HAZMAT:
        sx = sy = sz = .01; //
        break;
    case object_type::DOOR:
        sx = 0.1;
        sy = 0.2;
        sz = 1.;

        break;
    case object_type::PERSON:
        sx = sy = sz = .4; //
        break;
    case object_type::FIRE_EXTINGUISHER:
        sx = sy = sz = .4; //
        break;
    default:
        sx = sy = sz = NAN; //
        break;
    }
    return Eigen::DiagonalMatrix<float, 3>(sx * sx, sy * sy, sz * sz);
}

// The rotation of the camera relative to the world coordinates
Eigen::Matrix3f get_extrinsic_rotation()
{
    Eigen::Matrix3f P = Eigen::Matrix3f::Zero();
    P.row(0) << 0.0162226, 0.0753285, 0.997027;
    P.row(1) << -0.999617, -0.0211404, 0.017862;
    P.row(2) << 0.022423, -0.996935, 0.0749567;
    return P;
}

// Get the points in 3D space relative to a stationary reference frame
std::vector<observation_3d> get_3d(const std::vector<object2d_with_depth> &detections, const camera_pose_t &camera_pose, const rs2_intrinsics &intrinsics)
{
    // The return value
    std::vector<observation_3d> ret;

    // The coodinate transform from the depth camera to the tracking camera
    static const Eigen::Matrix3f Q0 = get_extrinsic_rotation();
    static const Eigen::Vector3f t0 = Eigen::Vector3f{0.02104983, 0.05304557, 0.05261957};

    // The camera rotation as a rotation matrix
    const Eigen::Matrix3f Q{camera_pose.rotation};

    for (const auto &det : detections)
    {
        observation_3d obj;

        // Get the position of the objects relative to the depth camera
        Eigen::Vector3f pos_wrt_camera = deproject({(float)det.u, (float)det.v}, det.depth, intrinsics);

        // Find the corresponding point relative to a stationary reference frame
        obj.position = Q * (Q0 * pos_wrt_camera + t0) + camera_pose.translation;

        auto M = (Q * Q0);
        obj.orientation_matrix = M * det.orientation_matrix * M.transpose(); // really the axis not the covar

        // Find a rotation matrix that takes (1,0,0) to this normal
        Eigen::Vector3f n = covar_to_orientation(obj.orientation_matrix);
        auto axis = Eigen::Vector3f::UnitX().cross(n);
        axis.normalize();
        float theta = acosf(Eigen::Vector3f::UnitX().dot(n));
        Eigen::Matrix3f normal_rotation(Eigen::AngleAxisf(theta, axis));
        Eigen::DiagonalMatrix<float, 3> Lambda = get_object_covariance(det.type);

        // The covariance matrix is roated accordngly
        auto object_covariance = normal_rotation * Lambda * normal_rotation.transpose();

        // The total noise is the camera's translation error plus the fact that the pixel you chose won't be exaclty at the object's centre
        obj.covariance = object_covariance + camera_pose.covariance;
        obj.type = det.type;
        obj.text = det.text;
        obj.confidence = det.confidence;
        obj.distribution = det.distribution;

        if (fabs(covar_to_orientation(obj.orientation_matrix).z()) < angle_tol(det.type))
            ret.push_back(obj);
    }
    return ret;
}
