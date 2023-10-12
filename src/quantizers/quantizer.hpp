#ifndef QUANTIZER_HPP
#define QUANTIZER_HPP

#include <opencv2/opencv.hpp>

namespace lfc
{
    class Qunatizer
    {
    private:
        /* data */
    public:
        virtual cv::Mat quantize(const cv::Mat &image, const cv::Mat &referenceImage) const = 0;
        virtual ~Qunatizer() {}
    };
}

#endif