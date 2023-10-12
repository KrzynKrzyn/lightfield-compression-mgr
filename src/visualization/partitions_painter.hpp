#ifndef PARTITONS_PAINTER_HPP
#define PARTITONS_PAINTER_HPP

#include <opencv2/opencv.hpp>

namespace lfc::visualization
{
    class PartitionsPainter
    {
    public:
        cv::Mat paint(cv::Mat image, std::vector<cv::Rect> partitions)
        {
            cv::Mat partitionMat;
            image.copyTo(partitionMat);

            for (const auto &partition : partitions)
            {
                cv::rectangle(partitionMat, partition, cv::Vec3i(196, 0, 0));
            }

            return partitionMat;
        }
    };
}

#endif