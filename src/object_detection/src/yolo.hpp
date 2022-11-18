/**
 * @file yolo.hpp
 * @author Elorm Avevor (ea520@cam.ac.uk)
 * @brief Yolo object detection with OpenCV and OpenVINO
 * @version 0.1
 * @date 2022-11-05
 *
 * @copyright Copyright (c) 2022
 *
 * EXAMPLE:
 * @code {.cpp}
 * yolo::yolo_net net("weights.bin", "weights.xml", "classes.txt", cv::dnn::DNN_TARGET_OPENCL_FP16);
 * auto img = cv::imread("img.png");
 * auto detections = net.detect(img);
 * yolo::draw_boxes(detections, img, net);
 * cv::imshow("Yolo Example", img);
 * cv::waitkey(0);
 * @endcode
 *
 */
#pragma once
#include <opencv2/opencv.hpp>
#include <zbar.h>
namespace yolo
{
    /// @brief Struct for an object detection
    struct Detection
    {
        /// @brief The id of the object corrensponding to the class list
        int class_id;
        /// @brief The confidence of the object
        float confidence;
        /// @brief The bounding box for the object
        cv::Rect box;
    };
    /// @brief Structure storing all the data for a YOLO neural network
    struct yolo_net
    {
        /// @brief Constructor
        /// @param bin_path Binary file with trained weights.
        /// @param xml_path XML configuration file with network's topology.
        /// @param class_list_path New line separated list of object classes (should be known at training time)
        /// @param target // e.g. cv::dnn::DNN_TARGET_OPENCL_FP16
        /// @param conf_thresh // The confidence threshold
        yolo_net(const std::string &bin_path, const std::string &xml_path, const std::string &class_list_path, int target, float conf_thresh = 0.8);

        /// @brief Run the model on an image
        /// @param image
        /// @return A list of the detected objects
        std::vector<Detection> detect(const cv::Mat &image);
        /// @brief Set the confidence threshold for detection
        /// @param thresh
        void set_conf_thresh(float thresh) { conf_thresh = thresh; }
        /// @brief Get the current value of the confidence threshold
        /// @return
        float get_conf_thresh() const { return conf_thresh; }
        /// @brief Get the list of classes
        /// @return
        const std::vector<std::string> &get_class_list() const { return class_list; }

    private:
        cv::Mat format_yolov5(const cv::Mat &source, int &xoffset, int &yoffset, float &scale_factor);
        cv::dnn::Net net;
        float conf_thresh;
        unsigned detection_count, floats_per_detection, INPUT_HEIGHT, INPUT_WIDTH;
        std::vector<std::string> class_list;
    };
    /// @brief Draw the QR codes onto the image (must be the same shape image as was used in the QR code detection)
    /// @param image
    /// @param frame
    void draw_qrs(const zbar::Image &image, cv::Mat &frame);
    /// @brief Draw the bounding boxes from the YOLO detection
    /// @param detections List of the detected objects from calling @code{.cpp} net.detect(frame) @endcode
    /// @param frame The image onto which the bounding boxes will be drawn
    /// @param net Yolo network that generated the detections
    void draw_boxes(const std::vector<Detection> &detections, cv::Mat &frame, const yolo_net &net);
}
