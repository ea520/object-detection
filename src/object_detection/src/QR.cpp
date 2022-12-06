#include "QR.hpp"
std::vector<object2d> QR_scanner::detect(const cv::Mat &image)
{
    cv::Mat grey;
    cv::cvtColor(image, grey, cv::COLOR_BGR2GRAY);
    zbar::Image img(grey.cols, grey.rows, "Y800", (uchar *)grey.data, grey.cols * grey.rows);
    scanner.scan(img);
    std::vector<object2d> ret;
    for (zbar::Image::SymbolIterator symbol = img.symbol_begin(); symbol != img.symbol_end(); ++symbol)
    {
        const std::string text = symbol->get_data();

        std::vector<int> xs, ys;
        int x1 = INT_MAX, y1 = INT_MAX, x2 = 0, y2 = 0;
        for (int i = 0; i < symbol->get_location_size(); ++i)
        {
            int x_val = symbol->get_location_x(i);
            int y_val = symbol->get_location_y(i);
            x1 = std::min(x1, x_val);
            x2 = std::max(x2, x_val);
            y1 = std::min(y1, y_val);
            y2 = std::max(y2, y_val);
        }
        object2d obj;
        obj.u = (x1 + x2) / 2;
        obj.v = (y1 + y2) / 2;
        obj.w = x2 - x1;
        obj.h = y2 - y1;
        obj.confidence = 1.f;
        obj.text = symbol->get_data();
        obj.type = object_type::QR;
        ret.push_back(obj);
    }
    return ret;
}