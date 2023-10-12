#ifndef FILTER_HPP
#define FILTER_HPP

#include <opencv2/opencv.hpp>

namespace lfc
{
    class Filter
    {
    private:
        /* data */
    public:
        virtual cv::Mat filter(const cv::Mat &inputMat) = 0;
        virtual ~Filter() {}
    };
}

#endif