#include "yolo.hpp"
#include <fstream>
#include <thread>
#include <Eigen/Dense>
constexpr float SCORE_THRESHOLD = 0.7;
constexpr float NMS_THRESHOLD = 0.4;

// Convert the image into a 640*512 image with a grey top/bottom or left/right border depending on the input aspect ratio
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

// Get the class names from the file
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

yolo_net::yolo_net(const std::string &bin_path, const std::string &xml_path, const std::string &class_list_path, int target, float conf_thresh)
    : conf_thresh(conf_thresh)
{

    std::ifstream bin{bin_path}, xml{xml_path};
    if (!bin.is_open() || !xml.is_open())
        throw std::runtime_error("Could not open the YOLO model");
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
        std::cout << "model output layer is ambiguous. Could be:";
        for (auto &name : names)
            printf("%s ", name.c_str());
        std::cout << std::endl;
        std::exit(1);
    }

    net.getLayerShapes(inshape, out_id, in, out);
    assert(out.size() == 1 && out[0].size() == 3);
    detection_count = out[0][1], floats_per_detection = out[0][2];
    class_list = load_names(class_list_path);
    if ((int)class_list.size() < floats_per_detection - 5)
    {
        std::cerr << "Warning: class list is too small. Using numbers instead." << std::endl;
        for (int i = 0; i < floats_per_detection - 5; i++)
            class_list.emplace_back(std::to_string(i));
    }
    if ((int)class_list.size() > floats_per_detection - 5)
    {
        std::cerr << "Warning: class list is too big. Probably using the wrong class list." << std::endl;
    }
}

bool is_person(int idx) { return idx == 0; }
bool is_fire_extinguisher(int idx) { return idx == 1; }
bool is_door(int idx) { return idx == 2; }
bool is_hazmat(int idx) { return idx >= 3 && idx < 16; }

std::vector<object2d> yolo_net::detect(const cv::Mat &image)
/*
 - resize the input image and add borders so it's the right input shape
 - run the YOLO inference on that image
 - the output format is an array of {x,y,w,h,conf,score1,...,scoreN} where N is the number of of classes
 - where x,y are the coordinates of the top left of the bounding box
 - these coordinates may or may not be normalised to [0,1]
 - w,h are the width and height of the bounding box. Also normalised possibly.
 - conf is the confidence in the prediction [0,1]
 - the detected class is the one with the highest score
 -
*/
{
    // Do the inference
    cv::Mat blob;
    int xoffset, yoffset;
    float scale_factor;
    auto input_image = format_yolov5(image, xoffset, yoffset, scale_factor); // make the image the right shape by scaling and adding black bars
    cv::dnn::blobFromImage(input_image, blob, 1 / 255., cv::Size(INPUT_WIDTH, INPUT_HEIGHT), cv::Scalar(), true, false);
    net.setInput(blob);

    cv::Mat outputs;
    net.forward(outputs, output_name); // perform inference, get the data

    const float *start = reinterpret_cast<const float *>(outputs.data);
    const float *end = start + detection_count * floats_per_detection;
    const static int nclasses = floats_per_detection - 5; // {x,y,w,h,conf} corresponds to 5
    std::vector<int> class_ids;                           // class_ids[0] would be the class id corresponding to the networks 1st confident output
    std::vector<float> confidences;
    std::vector<Eigen::VectorXf> distributions;
    std::vector<cv::Rect> boxes;
    for (const float *ptr = start; ptr < end; ptr += floats_per_detection)
    {
        float x = ptr[0],
              y = ptr[1],
              w = ptr[2],
              h = ptr[3],
              conf = ptr[4];
        // scores is an array that starts 5 floats from the start
        const float *scores = ptr + 5;
        // sanity check
        assert(0. <= conf && conf <= 1.);
        if (conf < conf_thresh) // threshold the confidence
            continue;

        // find the max score
        const float *max_score_ptr = std::max_element(scores, scores + nclasses);
        Eigen::Map<const Eigen::VectorXf> cond_probs(scores, nclasses);
        // find the corresponding index in the scores array
        int class_id = max_score_ptr - scores;

        // sanity check
        assert(class_id >= 0 && class_id <= nclasses);

        // threshold the max score
        if (*max_score_ptr > SCORE_THRESHOLD)
        {
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

            // Distribution over all classes (it was calculated with 16bit floats so won't add to 1)
            Eigen::VectorXf distribution = cond_probs.array() / cond_probs.sum();
            // Distribution over the hazmats (sum is less than 1)
            distribution = distribution.tail(nclasses - 3);

            if (is_hazmat(class_id))
            {
                conf *= distribution.sum(); // The probability there is an object and it is a hazmat
            }
            else
            {
                conf *= *max_score_ptr;
            }
            distributions.push_back(distribution);
            confidences.push_back(conf);
        }
    }

    // Some objects will be double counted. If there are bounding boxes that intersect a lot, choose the ones with the higher confidence score
    std::vector<int> nms_result;
    cv::dnn::NMSBoxes(boxes, confidences, SCORE_THRESHOLD, NMS_THRESHOLD, nms_result);

    std::vector<object2d> output;
    output.reserve(nms_result.size());

    // Record the remaining bounding boxes
    for (int idx : nms_result)
    {
        object2d result;
        result.text = class_list[class_ids[idx]];
        result.confidence = confidences[idx];
        result.w = boxes[idx].width;
        result.h = boxes[idx].height;
        result.u = boxes[idx].x + result.w / 2;
        result.v = boxes[idx].y + result.h / 2;
        if (is_person(class_ids[idx]))
            result.type = object_type::PERSON;
        else if (is_fire_extinguisher(class_ids[idx]))
            result.type = object_type::FIRE_EXTINGUISHER;
        else if (is_door(class_ids[idx]))
            result.type = object_type::DOOR;
        else if (is_hazmat(class_ids[idx]))
        {
            result.type = object_type::HAZMAT;
            result.distribution = distributions[idx];
        }
        else
        {
            std::stringstream ss;
            ss << "Unknown object index (" << class_ids[idx] << ")";
            throw std::runtime_error(ss.str());
        }
        output.push_back(result);
    }
    return output;
}