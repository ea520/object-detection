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

Eigen::Vector3f get_normal(std::vector<Eigen::Vector3f> points, float *mse = nullptr)
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
    if (mse)
        *mse = errors.dot(errors) / errors.rows();
    return n.normalized();
}

Eigen::Vector3f get_axis(std::vector<Eigen::Vector3f> points)
{
    // Least squares fit for (nx,ny,nz) * (ax,-1,az) = 0 aka (nx,nz) * (ax,az) = ny
    if (points.size() < 10)
        throw std::runtime_error("Not enough points to calculate plane (need at least 10)");
    typedef Eigen::Matrix<float, Eigen::Dynamic, 3, Eigen::RowMajor> Mat3x;
    Mat3x mat = Eigen::Map<Mat3x>(&points[0][0], points.size(), 3);
    Eigen::Matrix2f A;
    A(0, 0) = mat.col(0).dot(mat.col(0));
    A(1, 0) = A(0, 1) = mat.col(0).dot(mat.col(2));
    A(1, 1) = mat.col(1).dot(mat.col(2));
    Eigen::Vector2f b{
        mat.col(0).dot(mat.col(1)),
        mat.col(2).dot(mat.col(1)),
    };
    Eigen::Vector3f axis;
    axis(1) = -1.; // Normal points somewhat towards the camera
    axis.head<2>() = A.ldlt().solve(b);
    return axis.normalized();
}

// Draw the bounding boxes to the image
void draw_boxes(const std::vector<object2d> &detections, cv::Mat &frame)
{
    for (size_t i = 0; i < detections.size(); ++i)
    {

        const auto &detection = detections[i];
        const cv::Point2i top_left{detection.u - detection.w / 2, detection.v - detection.w / 2};
        const cv::Point2i bottom_right{detection.u + detection.w / 2, detection.v + detection.w / 2};
        const cv::Rect box{top_left, bottom_right};
        auto classId = (std::hash<std::string>{}(detection.text) + (size_t)detection.type) % colors.size();
        if (detection.text == "")
            classId = rand();
        const auto &color = colors[classId];
        // Get a string for the confidence
        char buff[6] = "\0";
        if (detection.type != object_type::QR)
            snprintf(buff, sizeof(buff), " %.2f", detection.confidence);

        // draw the bounding box
        cv::rectangle(frame, box, color, 3);

        // Draw a filled box where the text will go
        cv::rectangle(frame, cv::Point(box.x, box.y - 10), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
        // Make the text collor dissimilar to the background colour
        cv::Scalar textcolor;
        textcolor[0] = color[0] < 128 ? 255 : 0;
        textcolor[1] = color[1] < 128 ? 255 : 0;
        textcolor[2] = color[2] < 128 ? 255 : 0;
        // Write the text
        cv::putText(frame, detection.text + buff, cv::Point(box.x, box.y), cv::FONT_HERSHEY_PLAIN, 1.0, textcolor);
    }
}

// For planar objects, how big can the average square error be for it to be considered a plane?
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
            size_t iterations = 0;
            std::vector<Eigen::Vector3f> points = sample_depth_map(depth, u, v, width, height, N, intrinsics);
            if (points.size() == N)
            {
                float mse;
                det.normal = get_normal(points, &mse);
                if (mse < mse_tol(det.type))
                    ret.push_back(det);
            }
        }
        else if (det.type == object_type::FIRE_EXTINGUISHER)
        {
            // smaller bounding box;
            int width = 0.2 * detection.w;
            int height = 0.2 * detection.h;
            int u0 = det.u - width / 2;  // left
            int v0 = det.v - height / 2; // top
            int w = width / 4;
            int h = height / 4;
            static constexpr size_t N = 16 * 16;
            std::vector<Eigen::Vector3f> normals;
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                {
                    int u = u0 + i * w;
                    int v = v0 + j * h;
                    std::vector<Eigen::Vector3f> points = sample_depth_map(depth, u, v, w, h, N, intrinsics);
                    if (points.size() == N)
                    {
                        normals.push_back(get_normal(points));
                    }
                }
            if (normals.size() > 10)
            {
                det.normal = get_axis(normals);
            }
            else
                det.normal *= NAN;
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

        // Rotate the normal with the new reference frame
        obj.normal = Q * Q0 * det.normal;
        // Find a rotation matrix that takes (1,0,0) to this normal
        auto n = obj.normal.normalized();
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

        if (fabs(obj.normal.z()) < angle_tol(det.type))
            ret.push_back(obj);
    }
    return ret;
}
