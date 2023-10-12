#ifndef RANGE_QUANTIZER_HPP
#define RANGE_QUANTIZER_HPP

#include <opencv2/opencv.hpp>
#include <algorithm>

#include "quantizer.hpp"

namespace lfc
{
    class RangeQuantizer : public Qunatizer
    {
    private:
        const int quantizationRange;
        const int quantizationStep;

    public:
        RangeQuantizer(const int quantizationRange) : quantizationRange(quantizationRange), quantizationStep(2 * quantizationRange + 1) {}

        short quantizeDiff(const short &value) const
        {
            int qValue = value < 0 ? (value - quantizationRange) / quantizationStep : (value + quantizationRange) / quantizationStep;
            qValue *= quantizationStep;
            return qValue;
        }

        cv::Mat quantize(const cv::Mat &image, const cv::Mat &referenceImage) const override
        {
            cv::Mat quantizedImage;
            image.copyTo(quantizedImage);

            cv::Mat diffMat;
            cv::subtract(image, referenceImage, diffMat, cv::noArray(), CV_16SC3);

            cv::Mat quantizedDiffMat(diffMat.size(), diffMat.type());
            diffMat.copyTo(quantizedDiffMat);
            // quantizedDiffMat.setTo(cv::Scalar(0, 0, 0));

            for(cv::MatIterator_<cv::Vec3s> it = quantizedDiffMat.begin<cv::Vec3s>(), end = quantizedDiffMat.end<cv::Vec3s>(); it != end; ++it) {
                const cv::Vec3s& v = *it;
                *it = cv::Vec3s(quantizeDiff(v[0]), quantizeDiff(v[1]), quantizeDiff(v[2]));
            }
                
            // std::transform(diffMat.begin<short>(), diffMat.end<short>(), quantizedDiffMat.begin<short>(), this->quantizeDiff);

            cv::add(referenceImage, quantizedDiffMat, quantizedImage, cv::noArray(), image.type());

            return quantizedImage;
        }

        // cv::Mat quantize(const cv::Mat &image, const cv::Mat &referenceImage) const override
        // {
        //     cv::Mat quantizedImage;
        //     image.copyTo(quantizedImage);

        //     cv::Mat diffMat;
        //     cv::subtract(image, referenceImage, diffMat, cv::noArray(), CV_16SC3);

        //     double minVal, maxVal;
        //     cv::minMaxIdx(diffMat, &minVal, &maxVal);

        //     // std::cout << minVal << " " << maxVal << std::endl;

        //     cv::Mat quantizedDiffMat(diffMat.size(), diffMat.type());
        //     quantizedDiffMat.setTo(cv::Scalar(0,0,0));

        //     int step = quantizationRange * 2;
        //     for (int quantizedDiff = ((int)minVal / step)*step; quantizedDiff <= ((int)maxVal / step)*step; quantizedDiff += step)
        //     {
        //         // std::cout << quantizedDiff - quantizationRange << " " << quantizedDiff << " " << quantizedDiff + quantizationRange << std::endl;
        //         cv::Mat maskMin = diffMat >= (quantizedDiff - quantizationRange);
        //         cv::Mat maskMax = diffMat < (quantizedDiff + quantizationRange);
        //         cv::Mat quantizationMask;
        //         cv::bitwise_and(maskMax, maskMin, quantizationMask);
        //         quantizedDiffMat.setTo(cv::Scalar(quantizedDiff, quantizedDiff, quantizedDiff), quantizationMask);
        //     }

        //     cv::add(referenceImage, quantizedDiffMat, quantizedImage, cv::noArray(), image.type());

        //     return quantizedImage;
        // }
    };
}

#endif