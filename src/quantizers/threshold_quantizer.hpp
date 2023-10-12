#ifndef THRESHOLD_QUANTIZER_HPP
#define THRESHOLD_QUANTIZER_HPP

#include <opencv2/opencv.hpp>

#include "quantizer.hpp"

namespace lfc
{
    class ThresholdQuantizer : public Qunatizer
    {
    private:
        const int quantizationThreshold;
    public:
        ThresholdQuantizer(const int quantizationThreshold): quantizationThreshold(quantizationThreshold) {}

        cv::Mat quantize(const cv::Mat &image, const cv::Mat &referenceImage) const override
        {
            cv::Mat diffMat;
            cv::absdiff(image, referenceImage, diffMat);

            cv::Mat maskBiggerThanMaxDifference = diffMat <= quantizationThreshold;

            cv::Mat qunatizedImage;
            image.copyTo(qunatizedImage);
            referenceImage.copyTo(qunatizedImage, maskBiggerThanMaxDifference);

            return qunatizedImage;
        }
    };
}

#endif