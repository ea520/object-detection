#pragma once
#include <zbar.h>
#include <opencv2/opencv.hpp>
#include "object.hpp"
struct QR_scanner
{
    QR_scanner()
    {
        scanner.set_config(zbar::ZBAR_NONE, zbar::ZBAR_CFG_ENABLE, 0);
        scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    }
    std::vector<object2d> detect(const cv::Mat &image);

private:
    zbar::ImageScanner scanner;
};