#include "yolo.hpp"
#include <fstream>
#include <thread>
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
constexpr float SCORE_THRESHOLD = 0.2;
constexpr float NMS_THRESHOLD = 0.4;
namespace yolo
{
    cv::Mat yolo_net::format_yolov5(const cv::Mat &source, int &xoffset, int &yoffset, float &scale_factor)
    {
        int col = source.cols;
        int row = source.rows;
        float aspect = (float)row / (float)col;

        int new_width = (int)INPUT_WIDTH;
        int new_height = int(new_width * aspect);
        cv::Mat resized;
        // assume it's landscape with an aspect ratio greater than 640:512
        cv::resize(source, resized, cv::Size(new_width, new_height), 0, 0, cv::INTER_AREA);
        cv::Mat result = cv::Mat::zeros(int(INPUT_HEIGHT), int(INPUT_WIDTH), CV_8UC3);
        scale_factor = col / new_width;
        int left = (INPUT_WIDTH - new_width) / 2;
        int top = (INPUT_HEIGHT - new_height) / 2;
        int bottom = INPUT_HEIGHT - top - new_height;
        int right = INPUT_WIDTH - left - new_width;
        xoffset = left;
        yoffset = top;
        cv::copyMakeBorder(resized, result, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(114, 114, 114));
        return result;
    }
    std::vector<std::string> load_names(const std::string &path)
    {
        std::ifstream ifs(path);
        std::vector<std::string> ret;
        for (std::string line; std::getline(ifs, line);)
        {
            ret.emplace_back(line);
        }
        return ret;
    }
    std::string output_name;
    yolo_net::yolo_net(const std::string &bin_path, const std::string &xml_path, const std::string &class_list_path, int target, float conf_thresh)
        : conf_thresh(conf_thresh)
    {
        std::ifstream bin{bin_path}, xml{xml_path};
        assert(bin.is_open() && xml.is_open());
        bin.close();
        xml.close();
        net = cv::dnn::readNetFromModelOptimizer(xml_path, bin_path);
        net.setPreferableBackend(cv::dnn::DNN_BACKEND_INFERENCE_ENGINE);
        net.setPreferableTarget(target);

        std::vector<int> inshape{};
        std::vector<std::vector<int>> in, out;

        auto input_id = 0;
        net.getLayerShapes(inshape, input_id, in, out); // get the input shape
        INPUT_HEIGHT = in[0][2];
        INPUT_WIDTH = in[0][3];

        auto names = net.getUnconnectedOutLayersNames();
        int out_id;
        if (names.size() == 1)
        {
            out_id = net.getLayerId(names[0]);
            output_name = names[0];
        }
        else
        {
            std::cout << "model output layer is ambiguuous. Could be:";
            for (auto &name : names)
                printf("%s ", name.c_str());
            std::cout << std::endl;
            std::exit(1);
        }

        net.getLayerShapes(inshape, out_id, in, out);
        assert(out.size() == 1 && out[0].size() == 3);
        detection_count = out[0][1], floats_per_detection = out[0][2];
        class_list = load_names(class_list_path);
        if (class_list.size() < floats_per_detection - 5)
        {
            std::cerr << "Warning: class list is too small. Using numbers instead." << std::endl;
            for (size_t i = 0; i < floats_per_detection - 5; i++)
                class_list.emplace_back(std::to_string(i));
        }
        if (class_list.size() > floats_per_detection - 5)
        {
            std::cerr << "Warning: class list is too big. Probably using the wrong class list." << std::endl;
        }
    }

    std::vector<Detection> yolo_net::detect(const cv::Mat &image)
    /*
     - resize the input image and add borders so it's the right input shape
     - run the YOLO inference on that image
     - the output format is an array of {x,y,w,h,conf,score1,...,score16}
     - where x,y are the coordinates of the top left of the bounding box
     - these coordinates are normalised to [0,1]
     - w,h are the width and height of the bounding box. Also normalised.
     - conf is the confidence in the prediction [0,1]
     - the detected class is the one with the highest score
     - The variable `rows` is the number of such arrays there are.
     - Perhaps the yolo output format is best shown by looking at the variable `data`
     -
    */
    {
        cv::Mat blob;
        int xoffset, yoffset;
        float scale_factor;
        auto input_image = format_yolov5(image, xoffset, yoffset, scale_factor); // make the image the right shape by scaling and adding black bars
        cv::dnn::blobFromImage(input_image, blob, 1 / 255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
        net.setInput(blob);

        cv::Mat outputs;
        // net.forward(outputs, "output"); // perform inference, get the data
        net.forward(outputs, output_name); // perform inference, get the data

        const float *start = reinterpret_cast<const float *>(outputs.data);
        const float *end = start + detection_count * floats_per_detection;
        const static int nclasses = floats_per_detection - 5;
        std::vector<int> class_ids; // class_ids[0] would be the class id corresponding to the networks 1st confident output
        std::vector<float> confidences;
        std::vector<cv::Rect> boxes;
        for (const float *ptr = start; ptr < end; ptr += floats_per_detection)
        {
            float x = ptr[0],
                  y = ptr[1],
                  w = ptr[2],
                  h = ptr[3],
                  conf = ptr[4];
            const float *scores = ptr + 5;
            assert(0. <= conf && conf <= 1.);
            if (conf < conf_thresh) // threshold the confidence
                continue;

            const float *max_score_ptr = std::max_element(scores, scores + nclasses);
            int class_id = max_score_ptr - scores;
            assert(class_id >= 0 && class_id <= nclasses);

            if (*max_score_ptr > SCORE_THRESHOLD) // threshold the max score
            {
                confidences.push_back(conf);
                class_ids.push_back(class_id);
                if (x < 1. && y < 1. && w < 1. && h < 1.)
                {
                    // very likely to be in normalised coordinates
                    x *= INPUT_WIDTH;
                    y *= INPUT_HEIGHT;
                    w *= INPUT_WIDTH;
                    h *= INPUT_HEIGHT;
                }
                int xcoord = scale_factor * (x - 0.5 * w - xoffset);
                int ycoord = scale_factor * (y - 0.5 * h - yoffset);
                cv::Point2i top_left(xcoord, ycoord);
                cv::Size2i box_size((int)(w * scale_factor), (int)(h * scale_factor));

                boxes.emplace_back(top_left, box_size); // add the corresponding box to the list
            }
        }

        std::vector<int> nms_result;
        cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);

        std::vector<Detection> output;
        output.reserve(nms_result.size());

        for (int idx : nms_result)
        {
            Detection result;
            result.class_id = class_ids[idx];
            result.confidence = confidences[idx];
            result.box = boxes[idx];
            output.push_back(result);
        }
        return output;
    }

    void draw_boxes(const std::vector<Detection> &detections, cv::Mat &frame, const yolo_net &net)
    {
        for (size_t i = 0; i < detections.size(); ++i)
        {

            const auto &detection = detections[i];
            const auto &box = detection.box;
            auto classId = detection.class_id;
            const auto &color = colors[classId % colors.size()];
            char buff[6];
            snprintf(buff, sizeof(buff), " %.2f", detection.confidence);

            cv::rectangle(frame, box, color, 3);
            cv::rectangle(frame, cv::Point(box.x, box.y - 10), cv::Point(box.x + box.width, box.y), color, cv::FILLED);
            cv::putText(frame, net.get_class_list()[classId] + buff, cv::Point(box.x, box.y), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 0));
        }
    }

    void draw_qrs(const zbar::Image &image, cv::Mat &frame)
    {
        for (zbar::Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol)
        {
            const std::string text = symbol->get_data();

            std::vector<int> xs, ys;
            for (int i = 0; i < symbol->get_location_size(); i++)
            {
                xs.push_back(symbol->get_location_x(i));
                ys.push_back(symbol->get_location_y(i));
            }

            int x1 = *std::min_element(xs.begin(), xs.end());
            int y1 = *std::min_element(ys.begin(), ys.end());
            int x2 = *std::max_element(xs.begin(), xs.end());
            int y2 = *std::max_element(ys.begin(), ys.end());

            cv::rectangle(frame, cv::Rect(cv::Point2i(x1, y1), cv::Point2i(x2, y2)), {255, 0, 0}, 3);
            cv::putText(frame, symbol->get_data(), {x1, y1}, cv::FONT_HERSHEY_PLAIN, 1.5, {0, 0, 255}, 2);
        }
    }
}