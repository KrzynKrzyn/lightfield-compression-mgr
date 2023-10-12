#ifndef AVERAGE_FILTER_HPP
#define AVERAGE_FILTER_HPP

#include <opencv2/opencv.hpp>
#include "shift_vector_filter.hpp"

namespace lfc
{
    class AverageFilter : public Filter
    {
    private:
        const cv::Vec2i shiftVec;
    public:
        cv::Mat filter(const cv::Mat &inputMat) override
        {
            cv::Mat b = lfc::ShiftVectorFilter({0, 1}).filter(inputMat);
            cv::Mat a = lfc::ShiftVectorFilter({1, 0}).filter(inputMat);

            auto dtype = CV_16SC3;
            cv::Mat ret;
            cv::add(a, b, ret, cv::noArray(), dtype);
            ret /= 2;

            return ret;
        }
    };
}

#endif