#ifndef BLOCK_MATCHER_RESULTS_HPP
#define BLOCK_MATCHER_RESULTS_HPP

#include <vector>
#include <opencv2/opencv.hpp>

namespace lfc
{
    struct BlockMatchResults
    {
        cv::Rect soughtBlock;
        cv::Rect foundBlock;
        cv::Vec2i motionVector;
        double predictionError;
        double predictionTime;
        int referenceImageIndex;

        BlockMatchResults() : predictionError(std::numeric_limits<double>::max()) {}

        BlockMatchResults(cv::Rect soughtBlock, cv::Rect foundBlock, cv::Vec2i motionVector, double predictionError, double predictionTime, int referenceImageIndex = 0)
            : soughtBlock(soughtBlock),
              foundBlock(foundBlock),
              motionVector(motionVector),
              predictionError(predictionError),
              predictionTime(predictionTime),
              referenceImageIndex(referenceImageIndex) {}
    };
}

#endif