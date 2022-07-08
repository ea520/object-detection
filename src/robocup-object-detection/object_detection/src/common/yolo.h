#include <opencv2/opencv.hpp>
#include <zbar.h>
namespace yolo
{
    const int INPUT_WIDTH = 640;
    const int INPUT_HEIGHT = 512;
    const float SCORE_THRESHOLD = 0.2;
    const float NMS_THRESHOLD = 0.4;
    const float CONFIDENCE_THRESHOLD = 0.8;
    const int dimensions = 16 + 5;
    const int rows = 20160;

    static const std::array<std::string, 16> class_list = {
        "person",
        "fire extinguisher",
        "door",
        "inhalation-hazard",
        "infectious-substance",
        "explosive",
        "non-flammable-gas",
        "organic-peroxide",
        "flammable",
        "radioactive",
        "spontaneously-combustible",
        "oxygen",
        "dangerous",
        "flammable-solid",
        "corrosive",
        "poison"};
    // 16 distinct colours (apart from corrosive/poison)
    static const std::array<cv::Scalar, 16> colors = {
        cv::Scalar(255 * 0.294, 255 * 0.098, 255 * 0.902),
        cv::Scalar(255 * 0.294, 255 * 0.706, 255 * 0.235),
        cv::Scalar(255 * 0.098, 255 * 0.882, 255 * 1.000),
        cv::Scalar(255 * 0.847, 255 * 0.388, 255 * 0.263),
        cv::Scalar(255 * 0.192, 255 * 0.510, 255 * 0.961),
        cv::Scalar(255 * 0.706, 255 * 0.118, 255 * 0.569),
        cv::Scalar(255 * 0.941, 255 * 0.941, 255 * 0.275),
        cv::Scalar(255 * 0.902, 255 * 0.196, 255 * 0.941),
        cv::Scalar(255 * 0.047, 255 * 0.965, 255 * 0.737),
        cv::Scalar(255 * 0.745, 255 * 0.745, 255 * 0.980),
        cv::Scalar(255 * 0.502, 255 * 0.502, 255 * 0.000),
        cv::Scalar(255 * 0.141, 255 * 0.388, 255 * 0.604),
        cv::Scalar(255 * 0.784, 255 * 0.980, 255 * 1.000),
        cv::Scalar(255 * 0.000, 255 * 0.000, 255 * 0.502),
        cv::Scalar(255 * 0.765, 255 * 1.000, 255 * 0.667),
        cv::Scalar(255 * 0.765, 255 * 1.000, 255 * 0.667),
    };

    struct Detection
    {
        int class_id;
        float confidence;
        cv::Rect box;
    };

    struct Rate
    {
        Rate(unsigned ms) : ms(ms), t0(std::chrono::high_resolution_clock::now()) {}
        void pause();
        bool overspent() const { return m_overspent; };
        std::chrono::milliseconds ms;

    private:
        std::chrono::high_resolution_clock::time_point t0;
        bool m_overspent = false;
    };

    cv::Mat format_yolov5(const cv::Mat &source);
    void load_net(cv::dnn::Net &net, bool cpu);
    std::vector<Detection> detect(const cv::Mat &image, cv::dnn::Net &net);
    void draw_qrs(const zbar::Image &image, cv::Mat &frame);
    void draw_boxes(const std::vector<Detection> &detections, cv::Mat &frame);
}
