#pragma once
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <thread>
#include <nav_msgs/Odometry.h>

namespace util
{

    /// @brief Convert OpenCV image to to a ROS image message
    /// @param image
    /// @return the image as ROS format
    sensor_msgs::Image toImageMsg(const cv::Mat &image);

    // convert a ros immage message to an opencv image

    /// @brief Convert a ROS image message to an OpenCV image
    /// @param img
    /// @return
    cv::Mat toCVMat(const sensor_msgs::ImageConstPtr &img);

    /// @brief Struct for finding the CPU usage of the current programme
    struct cpu_timer_t
    {
        cpu_timer_t();
        /// @brief Get the CPU usage as a percentage (maximum of 100% * #threads)
        /// @return The CPU usage as a percentage
        int getCPUPercentage();

    private:
        clock_t lastCPU, lastSysCPU, lastUserCPU;
    };
    /// @brief Struct for pausing code for a certain time
    struct Rate
    {
        /// @brief Constructor
        /// @param us The time in microseconds between the when the constructor is called and when functin `wait` returns
        Rate(std::chrono::microseconds us) : us(us), t0(std::chrono::high_resolution_clock::now()) {}
        /// @brief Get the time in microseconds given in the constructor
        /// @return
        std::chrono::microseconds get_target() const { return us; }
        /// @brief Pause execution for the remaining time if there is any time left
        /// @return Whether there was any time remaining
        bool wait();

    private:
        std::chrono::microseconds us;
        std::chrono::high_resolution_clock::time_point t0;
    };
    /// @brief Struct for timing how long until its destructor is called
    struct scope_timer
    {
        /// @brief Constructor
        /// @param output A variable to save the time it took for the desturctor to be called
        scope_timer(std::chrono::high_resolution_clock::duration *output) : t0(std::chrono::high_resolution_clock::now()), output(output) {}
        ~scope_timer() { *output = std::chrono::high_resolution_clock::now() - t0; }

    private:
        std::chrono::high_resolution_clock::time_point t0;
        std::chrono::high_resolution_clock::duration *output;
    };
    /// @brief Struct for a circular buffer: When the buffer is full, the oldest entry gets removed.
    /// @tparam T The integer type of the buffer (char, long, unsigned etc.)
    /// @tparam N The maximum size of the buffer
    template <typename T, int N>
    struct circular_buffer
    {
        /// @brief Add a value to the buffer (if the buffer is full, also remove the earliest value)
        /// @param value
        void add_value(T value)
        {
            arr[position++] = value;
            if (position >= N)
            {
                position = 0;
                full = true;
            }
        }
        /// @brief Get the average value of the elements in the buffer
        /// @return The average
        T get_average()
        {
            long long sum = 0;
            if (full)
            {
                for (T val : arr)
                    sum += val;
                return sum / N;
            }
            else if (position > 0)
            {
                for (size_t i = 0; i < position; i++)
                    sum += arr[i];
                return sum / position;
            }
            else
            {
                return -1;
            }
        }

    private:
        size_t position = 0;
        bool full = false;
        std::array<T, N> arr{};
    };
}

/// @brief Basic 3D vector operations
struct vec3d
{
    constexpr vec3d(double x = 0., double y = 0., double z = 0.) : x(x), y(y), z(z) {}
    explicit constexpr vec3d(const geometry_msgs::Point &p) : x(p.x), y(p.y), z(p.z) {}
    explicit constexpr vec3d(float p[3]) : x(p[0]), y(p[1]), z(p[2]) {}
    explicit operator geometry_msgs::Point() const
    {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    constexpr vec3d operator+(const vec3d &v) const { return vec3d{x + v.x, y + v.y, z + v.z}; }
    void operator+=(const vec3d &v)
    {
        x += v.x;
        y += v.y;
        z += v.z;
    }
    constexpr vec3d operator+() const { return *this; }
    constexpr vec3d operator-(const vec3d &v) const { return vec3d{x - v.x, y - v.y, z - v.z}; }
    constexpr vec3d operator-() const { return vec3d{-x, -y, -z}; }
    constexpr vec3d operator*(double a) const { return vec3d{a * x, a * y, a * z}; }
    constexpr double dot(const vec3d &v) const { return x * v.x + y * v.y + z * v.z; }
    constexpr double &operator[](const size_t i)
    {
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;

        default:
            assert(0);
            return x; // just to satisfy the compiler
        }
    }
    double x, y, z;
};

struct matrix3d
{
public:
    constexpr matrix3d(const vec3d &x, const vec3d &y, const vec3d &z) : x(x), y(y), z(z) {}
    explicit constexpr matrix3d(const geometry_msgs::Quaternion &Q)
    {
        matrix3d &rot = *this;
        // make aliases so copying code from
        // https://automaticaddison.com/how-to-convert-a-quaternion-to-a-rotation-matrix/
        // is easier
        const double &q0 = Q.w, &q1 = Q.x, &q2 = Q.y, &q3 = Q.z;
        double &r00 = rot[0][0], &r01 = rot[0][1], &r02 = rot[0][2];
        double &r10 = rot[1][0], &r11 = rot[1][1], &r12 = rot[1][2];
        double &r20 = rot[2][0], &r21 = rot[2][1], &r22 = rot[2][2];

        r00 = 2 * (q0 * q0 + q1 * q1) - 1;
        r01 = 2 * (q1 * q2 - q0 * q3);
        r02 = 2 * (q1 * q3 + q0 * q2);

        r10 = 2 * (q1 * q2 + q0 * q3);
        r11 = 2 * (q0 * q0 + q2 * q2) - 1;
        r12 = 2 * (q2 * q3 - q0 * q1);

        r20 = 2 * (q1 * q3 - q0 * q2);
        r21 = 2 * (q2 * q3 + q0 * q1);
        r22 = 2 * (q0 * q0 + q3 * q3) - 1;
    }
    constexpr vec3d dot(const vec3d &v) const
    {
        return vec3d{x.dot(v), y.dot(v), z.dot(v)};
    }
    constexpr vec3d &operator[](const size_t i)
    {
        switch (i)
        {
        case 0:
            return x;
        case 1:
            return y;
        case 2:
            return z;

        default:
            std::cerr << "Index out of bounds" << std::endl;
            std::exit(EXIT_FAILURE);
            return x; // just to satisfy the compiler
        }
    }

private:
    vec3d x, y, z;
};