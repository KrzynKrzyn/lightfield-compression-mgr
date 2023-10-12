#ifndef SHIFT_VECTOR_FILTER_HPP
#define SHIFT_VECTOR_FILTER_HPP

#include <opencv2/opencv.hpp>

#include "filter.hpp"

namespace lfc
{
    class ShiftVectorFilter : public Filter
    {
    private:
        const cv::Vec2i shiftVec;
    public:
        ShiftVectorFilter(const cv::Vec2i& shiftVec): shiftVec(shiftVec) {}

        cv::Mat filter(const cv::Mat &inputMat) override
        {
            cv::Mat displacedInputMat(inputMat.size(), inputMat.type());
            displacedInputMat.setTo(cv::Scalar(0, 0, 0));

            int displacedRectWidth = inputMat.size().width - std::abs(shiftVec[0]);
            int displacedRectHeight = inputMat.size().height - std::abs(shiftVec[1]);

            int copyFromX = std::max(0, -shiftVec[0]);
            int copyFromY = std::max(0, -shiftVec[1]);

            int pasteToX = std::max(0, shiftVec[0]);
            int pasteToY = std::max(0, shiftVec[1]);

            cv::Rect copyFromRect(copyFromX, copyFromY, displacedRectWidth, displacedRectHeight);
            cv::Rect pasteToRect(pasteToX, pasteToY, displacedRectWidth, displacedRectHeight);

            cv::Mat copyFromBlock = inputMat(copyFromRect);
            cv::Mat pasteToBlock = displacedInputMat(pasteToRect);

            copyFromBlock.copyTo(pasteToBlock);

            return displacedInputMat;
        }
    };
}

#endif