#ifndef ENCODER_HPP
#define ENCODER_HPP

#include <vector>
#include <opencv2/opencv.hpp>

namespace lfc
{
    class Encoder
    {
    private:
    public:
        virtual EncodingResults predictImage(const cv::Mat &image, const cv::Mat &referenceImage, const std::vector<cv::Rect> &partitions) = 0;
    };
}

#endif