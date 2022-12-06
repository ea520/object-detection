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
