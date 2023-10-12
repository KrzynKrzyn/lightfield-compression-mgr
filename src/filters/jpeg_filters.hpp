#ifndef JPEG4_FILTER_HPP
#define JPEG4_FILTER_HPP

#include <opencv2/opencv.hpp>
#include "shift_vector_filter.hpp"

namespace lfc
{
    class Jpeg4Filter : public Filter
    {
    private:
        const cv::Vec2i shiftVec;
    public:
        cv::Mat filter(const cv::Mat &inputMat) override
        {
            cv::Mat up = lfc::ShiftVectorFilter({0, 1}).filter(inputMat);
            cv::Mat left = lfc::ShiftVectorFilter({1, 0}).filter(inputMat);
            cv::Mat upleft = lfc::ShiftVectorFilter({1, 1}).filter(inputMat);

            auto dtype = CV_16SC3;
            cv::Mat ret;
            cv::add(up, left, ret, cv::noArray(), dtype);
            cv::subtract(ret, upleft, ret, cv::noArray(), dtype);

            return ret;
        }
    };

    class Jpeg5Filter : public Filter
    {
    private:
        const cv::Vec2i shiftVec;
    public:
        cv::Mat filter(const cv::Mat &inputMat) override
        {
            cv::Mat up = lfc::ShiftVectorFilter({0, 1}).filter(inputMat);
            cv::Mat left = lfc::ShiftVectorFilter({1, 0}).filter(inputMat);
            cv::Mat upleft = lfc::ShiftVectorFilter({1, 1}).filter(inputMat);

            auto dtype = CV_16SC3;
            cv::Mat ret;
            cv::subtract(up, upleft, ret, cv::noArray(), dtype);
            ret /= 2;
            cv::add(left, ret, ret, cv::noArray(), dtype);


            return ret;
        }
    };

    class Jpeg6Filter : public Filter
    {
    private:
        const cv::Vec2i shiftVec;
    public:
        cv::Mat filter(const cv::Mat &inputMat) override
        {
            cv::Mat up = lfc::ShiftVectorFilter({0, 1}).filter(inputMat);
            cv::Mat left = lfc::ShiftVectorFilter({1, 0}).filter(inputMat);
            cv::Mat upleft = lfc::ShiftVectorFilter({1, 1}).filter(inputMat);

            auto dtype = CV_16SC3;
            cv::Mat ret;
            cv::subtract(left, upleft, ret, cv::noArray(), dtype);
            ret /= 2;
            cv::add(up, ret, ret, cv::noArray(), dtype);

            return ret;
        }
    };
}

#endif