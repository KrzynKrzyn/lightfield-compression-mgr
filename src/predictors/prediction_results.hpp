#ifndef PREDICTION_RESULTS_HPP
#define PREDICTION_RESULTS_HPP

#include <vector>
#include <opencv2/opencv.hpp>

#include "src/block_matchers/block_matcher_results.hpp"

namespace lfc
{
    struct PredictionResults
    {
        cv::Mat predictedMat;
        std::vector<BlockMatchResults> blockMatchResults;

        PredictionResults() {}

        PredictionResults(const cv::Mat &predictedMat, const std::vector<BlockMatchResults> &blockMatchResults)
            : predictedMat(predictedMat), blockMatchResults(blockMatchResults) {}
    };
}

#endif