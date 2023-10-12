#ifndef THRESHOLD_PAINTER_HPP
#define THRESHOLD_PAINTER_HPP

#include <opencv2/opencv.hpp>

namespace lfc::visualization
{
    class ThresholdPainter
    {
    public:
        cv::Mat paint(cv::Mat image, int threshold)
        {
            cv::Mat paintedThreshold(image.size(), CV_8UC3);

            paintedThreshold.setTo(cv::Scalar(0, 0, 0));
            paintedThreshold.setTo(cv::Scalar(255, 255, 255), image <= threshold);

            return paintedThreshold;
        }
    };
}

#endif