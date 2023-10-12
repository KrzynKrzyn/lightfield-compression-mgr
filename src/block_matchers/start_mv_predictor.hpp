#ifndef START_MV_PREDICTOR_HPP
#define START_MV_PREDICTOR_HPP

#include <opencv2/opencv.hpp>
#include "src/partitioners/partitions_model.hpp"
#include "src/predictors/prediction_results.hpp"

namespace lfc
{
    class StartMVPredictor
    {
        const std::vector<cv::Vec2i> comiteeNeighbours = {{-1, 0}, {0, -1}, {-1, -1}, {-2, 0}, {0, -2}};

    public:
        StartMVPredictor(const std::vector<cv::Vec2i>& comiteeNeighbours): comiteeNeighbours(comiteeNeighbours) {}

        cv::Vec2i predictMV(int partitionIndex, const PartitionModel &partitionModel, const PredictionResults &predictionResults) const
        {
            std::vector<cv::Vec2i> comiteeVecs;
            for (const auto &n : comiteeNeighbours)
            {
                const auto index = partitionModel.getNeighbouringPartitionIndex(partitionIndex, n[0], n[1]);
                if (index > 0 && index < predictionResults.blockMatchResults.size())
                {
                    comiteeVecs.push_back(predictionResults.blockMatchResults[index].motionVector);
                }
            }

            if (comiteeVecs.empty())
            {
                return cv::Vec2i(0, 0);
            }

            auto m = comiteeVecs.begin() + comiteeVecs.size() / 2;
            std::nth_element(comiteeVecs.begin(), m, comiteeVecs.end(), [](cv::Vec2i a, cv::Vec2i b)
                             { return a[0] < b[0]; });
            auto medianX = comiteeVecs[comiteeVecs.size() / 2][0];
            std::nth_element(comiteeVecs.begin(), m, comiteeVecs.end(), [](cv::Vec2i a, cv::Vec2i b)
                             { return a[1] < b[1]; });
            auto medianY = comiteeVecs[comiteeVecs.size() / 2][1];

            return {medianX, medianY};
        }
    };
}

#endif