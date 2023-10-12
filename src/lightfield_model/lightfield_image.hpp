#ifndef LIGHTFIELD_IMAGE_HPP
#define LIGHTFIELD_IMAGE_HPP

#include <vector>
#include <filesystem>
#include <opencv2/opencv.hpp>

namespace lfc
{
    struct LightfieldImage
    {
        const int col, row;
        const cv::Mat image;

        LightfieldImage(const cv::Mat &image, const int col, const int row) : image(image), col(col), row(row) {}
    };
}

#endif