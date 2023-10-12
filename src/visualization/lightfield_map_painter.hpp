#ifndef LIGHTFIELD_MAP_PAINTER_HPP
#define LIGHTFIELD_MAP_PAINTER_HPP

#include <opencv2/opencv.hpp>

namespace lfc::visualization
{
    class LightFieldMapPainter
    {
    public:
        cv::Mat paint(cv::Mat bppMat, cv::Scalar colors = {255, 0, 0}, double highlightAbove = std::numeric_limits<double>::max())
        {
            cv::Mat bppToShow;
            bppMat.copyTo(bppToShow);
            bppToShow *= -1;
            bppToShow += 10;
            bppToShow *= 25.5;
            bppToShow.convertTo(bppToShow, CV_8UC1);
            bppToShow.convertTo(bppToShow, CV_8UC3);
            cv::cvtColor(bppToShow, bppToShow, cv::COLOR_GRAY2BGR);
            cv::Mat fuckignMask(bppToShow.size(), bppToShow.type());
            fuckignMask.setTo(colors);
            bppToShow.setTo(cv::Scalar(0, 0, 0), fuckignMask);

            cv::resize(bppToShow, bppToShow, cv::Size(), 64, 64, 0);
            for (int x = 0; x < bppMat.cols; ++x)
                for (int y = 0; y < bppMat.rows; ++y)
                {
                    float bpp = bppMat.at<float>(y, x);

                    if (bpp > highlightAbove) {
                        cv::rectangle(bppToShow, cv::Rect(x*64, y*64, 64, 64), cv::Scalar(0, 0, 255), 2);
                    }

                    std::stringstream stream;
                    if (bpp <= 9.9999)
                        stream << std::fixed << std::setprecision(2) << bpp;
                    else
                        stream << std::fixed << std::setprecision(1) << bpp;

                    cv::Scalar color = bpp > 2.5 ? cv::Scalar(255, 255, 255) : cv::Scalar(0, 0, 0);
                    std::string bppText = stream.str();
                    cv::putText(bppToShow, bppText, {64 * x + 4, 64 * y + 40}, cv::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.75, color, 1);
                }

            return bppToShow;
        }
    };
}

#endif