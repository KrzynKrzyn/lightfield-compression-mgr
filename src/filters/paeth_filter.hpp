#ifndef PAETH_FILTER_HPP
#define PAETH_FILTER_HPP

#include <opencv2/opencv.hpp>
#include "shift_vector_filter.hpp"

namespace lfc
{
    class PaethPredictor : public Filter
    {
    public:
        cv::Mat filter(const cv::Mat &inputMat) override
        {
            cv::Mat up = lfc::ShiftVectorFilter({0, 1}).filter(inputMat);
            cv::Mat left = lfc::ShiftVectorFilter({1, 0}).filter(inputMat);
            cv::Mat upleft = lfc::ShiftVectorFilter({1, 1}).filter(inputMat);

            cv::Mat initialPaeth;
            auto dtype = CV_MAKETYPE(CV_16S, inputMat.channels());
            cv::add(up, left, initialPaeth, cv::noArray(), dtype);
            cv::subtract(initialPaeth, upleft, initialPaeth, cv::noArray(), dtype);

            cv::Mat paethUp, paethLeft, paethUpLeft;
            cv::absdiff(initialPaeth, up, paethUp);
            cv::absdiff(initialPaeth, left, paethLeft);
            cv::absdiff(initialPaeth, upleft, paethUpLeft);

            cv::Mat leftConditionA, leftConditionB, leftCondition;
            leftConditionA = paethLeft <= paethUp;
            leftConditionB = paethLeft <= paethUpLeft;
            cv::bitwise_and(leftConditionA, leftConditionB, leftCondition);

            cv::Mat upCondition = paethUp <= paethUpLeft;

            cv::Mat upLeftCondition;
            cv::bitwise_or(leftCondition, upCondition, upLeftCondition);
            cv::bitwise_not(upLeftCondition, upLeftCondition);

            cv::Mat finalPaeth(inputMat.size(), dtype);
            finalPaeth.setTo(cv::Scalar(0, 0, 0));
            left.copyTo(finalPaeth, leftCondition);
            up.copyTo(finalPaeth, upCondition);
            upleft.copyTo(finalPaeth, upLeftCondition);

            return finalPaeth;
        }
    };
}

#endif