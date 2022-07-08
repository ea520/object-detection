#include "util.h"
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
}