#ifndef FROM_REFERENCE_PREDICTOR_HPP
#define FROM_REFERENCE_PREDICTOR_HPP

#include <opencv2/opencv.hpp>

#include "src/partitioners/partitions_model.hpp"
#include "src/block_matchers/block_matcher_factory.hpp"
#include "src/block_matchers/start_mv_predictor.hpp"

#include "src/block_matchers/block_matcher_heuristic.hpp"

template <>
struct std::hash<cv::Vec2i>
{
    std::size_t operator()(const cv::Vec2i &s) const noexcept
    {
        return (s[0] + 10000) + s[1];
    }
};

namespace lfc
{
    class FromReferencePredictor
    {
    private:
        BlockMatcherFactory *bmFactory;
        StartMVPredictor startMVPredictor;

        BlockMatchResults getBestBlockMatch(std::vector<BlockMatchResults> results)
        {
            BlockMatchResults bestPrediction;
            double predictionTimeSum = 0;

            for (const auto &r : results)
            {
                if (r.predictionError < bestPrediction.predictionError)
                    bestPrediction = r;
                predictionTimeSum += r.predictionTime;
            }

            bestPrediction.predictionTime = predictionTimeSum;

            return bestPrediction;
        }

        PredictionResults getBestPrediction(std::vector<PredictionResults> results, const std::vector<LightfieldImage> &referenceImages)
        {
            PredictionResults bestResults;
            bestResults.predictedMat = cv::Mat(referenceImages[0].image.size(), referenceImages[0].image.type());

            const auto predictionsCount = results[0].blockMatchResults.size();

            for (int i = 0; i < predictionsCount; ++i)
            {
                std::vector<BlockMatchResults> blockMatchResults;
                for (const auto& r : results) blockMatchResults.push_back(r.blockMatchResults[i]);

                const auto bestBlockMatch = getBestBlockMatch(blockMatchResults);
                bestResults.blockMatchResults.push_back(bestBlockMatch);

                cv::Mat bestReferenceImage = referenceImages[bestBlockMatch.referenceImageIndex].image;
                cv::Mat predictedBlock = bestReferenceImage(bestBlockMatch.foundBlock);
                cv::Mat toPaste = bestResults.predictedMat(bestBlockMatch.soughtBlock);
                predictedBlock.copyTo(toPaste);
            }

            return bestResults;
        }

    public:
        FromReferencePredictor(BlockMatcherFactory *bmFactory, const StartMVPredictor &startMVPredictor) : bmFactory(bmFactory), startMVPredictor(startMVPredictor) {}

        PredictionResults predictImage(const cv::Mat &image, const cv::Mat &referenceImage, const PartitionModel &partitionModel)
        {
            PredictionResults results;

            cv::Mat predictedFromReference(referenceImage.size(), referenceImage.type());
            predictedFromReference.setTo(cv::Scalar(0, 0, 0));

            for (int i = 0; i < partitionModel.size(); ++i)
            {
                const auto &partition = partitionModel.getPartition(i);
                const auto startPredictedMV = startMVPredictor.predictMV(i, partitionModel, results);
                const std::set<cv::Vec2i> startMVs{cv::Vec2i(0, 0), startPredictedMV};

                BlockMatcher *blockMatcher = bmFactory->createInstance(image, referenceImage, partition);
                // auto prediction = blockMatcher->matchBlock(startMVs);
                auto prediction = ((BlockMatcherHeuristic *)blockMatcher)->matchBlockPickBetterStart(startMVs);
                // auto prediction = blockMatcher->matchBlock(cv::Vec2i(0, 0));
                delete blockMatcher;

                results.blockMatchResults.push_back(prediction);

                cv::Mat predictedBlock = referenceImage(prediction.foundBlock);
                cv::Mat toPaste = predictedFromReference(partition);
                predictedBlock.copyTo(toPaste);
            }

            results.predictedMat = predictedFromReference;

            return results;
        };

        PredictionResults predictImage(const LightfieldImage &image, const std::vector<LightfieldImage> &referenceImages, const PartitionModel &partitionModel)
        {
            std::vector<PredictionResults> results;

            for (int i = 0; i < referenceImages.size(); ++i)
            {
                const auto &referenceImage = referenceImages[i];
                auto predictionResult = predictImage(image.image, referenceImage.image, partitionModel);

                for (auto &r : predictionResult.blockMatchResults) r.referenceImageIndex = i;

                results.push_back(predictionResult);
            }

            PredictionResults pickedResults = getBestPrediction(results, referenceImages);

            return pickedResults;
        };
    };
}

#endif