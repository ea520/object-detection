#include "util.hpp"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "sys/times.h"
#include "sys/vtimes.h"
namespace util
{
    sensor_msgs::Image toImageMsg(const cv::Mat &image)
    {
        sensor_msgs::Image ros_image;
        ros_image.header = std_msgs::Header();
        ros_image.height = image.rows;
        ros_image.width = image.cols;
        ros_image.encoding = "bgr8";
        int n = 1;
        ros_image.is_bigendian = *(char *)&n != 1;
        ros_image.step = image.cols * image.elemSize();
        size_t size = ros_image.step * image.rows;
        ros_image.data.resize(size);

        if (image.isContinuous())
        {
            memcpy(reinterpret_cast<char *>(&ros_image.data[0]), image.data, size);
        }
        else
        {
            // Copy by row by row
            uchar *ros_data_ptr = reinterpret_cast<uchar *>(&ros_image.data[0]);
            uchar *cv_data_ptr = image.data;
            for (int i = 0; i < image.rows; ++i)
            {
                memcpy(ros_data_ptr, cv_data_ptr, ros_image.step);
                ros_data_ptr += ros_image.step;
                cv_data_ptr += image.step;
            }
        }
        return ros_image;
    }
    // convert an encoding string to its opencv equivalent number
    // only supports rgb8, bgr8 and 16UC1
    int toCVEncoding(const std::string &encoding)
    {
        if (encoding == "bgr8")
            return CV_8UC3;
        else if (encoding == "rgb8")
            return CV_8UC3;
        else if (encoding == "16UC1")
            return CV_16UC1;
        std::cerr << "UNKNOWN ENCODING" << std::endl;
        assert(0);
        return 0; // just incase the compiler complains
    }

    cv::Mat toCVMat(const sensor_msgs::ImageConstPtr &img)
    {

        auto output = cv::Mat(img->height, img->width, toCVEncoding(img->encoding), (void *)img->data.data());
        if (img->encoding == "rgb8")
            cv::cvtColor(output, output, cv::COLOR_RGB2BGR);
        return output;
    }

    bool Rate::wait()
    {
        using namespace std::chrono;
        auto wait_time = t0 + us - high_resolution_clock::now();
        std::this_thread::sleep_for(wait_time);
        return wait_time > 0ns;
    }
    cpu_timer_t::cpu_timer_t()
    {
        FILE *file;
        struct tms timeSample;

        lastCPU = times(&timeSample);
        lastSysCPU = timeSample.tms_stime;
        lastUserCPU = timeSample.tms_utime;

        file = fopen("/proc/cpuinfo", "r");
        fclose(file);
    }
    int cpu_timer_t::getCPUPercentage()
    {
        struct tms timeSample;
        clock_t now;
        double percent;

        now = times(&timeSample);
        if (now <= lastCPU || timeSample.tms_stime < lastSysCPU ||
            timeSample.tms_utime < lastUserCPU)
        {
            // Overflow detection. Just skip this value.
            percent = -1.0;
        }
        else
        {
            percent = (timeSample.tms_stime - lastSysCPU) +
                      (timeSample.tms_utime - lastUserCPU);
            percent /= (now - lastCPU);
            percent *= 100;
        }
        lastCPU = now;
        lastSysCPU = timeSample.tms_stime;
        lastUserCPU = timeSample.tms_utime;

        return (int)percent;
    }
}