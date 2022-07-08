#include <opencv2/opencv.hpp>
#include <sensor_msgs/Image.h>
#include <thread>
namespace util
{
    // convert an CV image to to a ros image message
    sensor_msgs::Image toImageMsg(const cv::Mat &image);

    // convert an encoding string to its opencv equivalent number
    // only supports rgb8, bgr8 and 16UC1
    int toCVEncoding(const std::string &encoding);

    // convert a ros immage message to an opencv image
    cv::Mat toCVMat(const sensor_msgs::ImageConstPtr &img);

    // For checking CPU usage
    struct CPU_timer
    {
        CPU_timer()
        {
            PID = std::to_string(getpid());
            processor_count = std::thread::hardware_concurrency();
            update();
            t0 = std::chrono::high_resolution_clock::now();
        }
        void update()
        {
            using namespace std::chrono;
            auto dt = high_resolution_clock::now() - t0;
            if (dt < 1000ms)
                return;
            t0 = high_resolution_clock::now();
            total_cpu_usage1 = total_cpu_usage2;
            proc_times1 = proc_times2;
            FILE *fp = popen("cat /proc/stat", "r");
            if (fp == nullptr)
            {
                printf("Error: file doesn't exist.");
                exit(-1);
            }
            char *line = NULL;
            size_t len = 0;
            size_t nchars = getline(&line, &len, fp);
            pclose(fp);
            assert(nchars > 0);
            unsigned long long user, nice, system, idle;
            sscanf(line,
                   "%*s %llu %llu %llu %llu",
                   &user,
                   &nice,
                   &system,
                   &idle);

            fp = popen(("cat /proc/" + PID + "/stat").c_str(), "r");
            if (fp == nullptr)
            {
                printf("Error: file doesn't exist");
                exit(-1);
            }
            line = NULL;
            len = 0;
            nchars = getline(&line, &len, fp);
            pclose(fp);
            assert(nchars > 0);
            unsigned long usertime, systemtime;
            sscanf(line,
                   "%*d %*s %*c %*d" // pid,command,state,ppid

                   "%*d %*d %*d %*d %*u %*u %*u %*u %*u"

                   "%lu %lu" // usertime,systemtime

                   "%*d %*d %*d %*d %*d %*d %*u"

                   "%*u", // virtual memory size in bytes
                   &usertime, &systemtime);

            total_cpu_usage2 = user + nice + system + idle;
            proc_times2 = usertime + systemtime;
        }
        double get_usage() const
        {
            return processor_count * (proc_times2 - proc_times1) * 100 / (double)(total_cpu_usage2 - total_cpu_usage1);
        }

    private:
        std::string PID;
        unsigned long total_cpu_usage1, total_cpu_usage2;
        unsigned long proc_times1, proc_times2;
        unsigned int processor_count;
        std::chrono::high_resolution_clock::time_point t0;
    };

    /*
    records the time till the timer's destructor is called
    e.g.
    std::chrono::high_resolution_clock::duration output;
    {
        scope_timer t(&output);
        // do stuff here
    }
    size_t nano_seconds = output.count;
    */
    struct scope_timer
    {
        scope_timer(std::chrono::high_resolution_clock::duration *output) : t0(std::chrono::high_resolution_clock::now()), output(output) {}
        ~scope_timer()
        {
            *output = std::chrono::high_resolution_clock::now() - t0;
        }
        std::chrono::high_resolution_clock::time_point t0;
        std::chrono::high_resolution_clock::duration *output;
    };

    // Every smoothes the frame times
    struct average_smoothing_t
    {
        void update(std::chrono::high_resolution_clock::duration dt)
        {
            cumulative_time += dt;
            counter++;
            if (cumulative_time > averaging_period)
            {
                smoothed = cumulative_time / counter;
                counter = 0;
                cumulative_time = std::chrono::milliseconds(0);
            }
        };
        std::chrono::high_resolution_clock::duration get() const { return smoothed; };
        std::chrono::high_resolution_clock::duration cumulative_time = std::chrono::milliseconds(0);
        std::chrono::high_resolution_clock::duration smoothed = std::chrono::milliseconds(0);
        int counter;
        constexpr static std::chrono::high_resolution_clock::duration averaging_period = std::chrono::milliseconds(500);
    };
}
