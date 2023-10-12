#ifndef FROM_REFERENCE_PREDICTION_PAINTER_HPP
#define FROM_REFERENCE_PREDICTION_PAINTER_HPP

// #include "src/src_2507/predictors/from_reference_predictor.hpp"
// #include "src/src_2507/motion_vector_predictors/mv_prediction_result.hpp"
#include "src/block_matchers/block_matcher_results.hpp"

namespace lfc::visualization
{
    class FromReferencePredictionPainter
    {
    public:
        std::array<cv::Mat, 5> paint(cv::Mat image, const std::vector<lfc::BlockMatchResults> &detailedResults, double maxMVLength = 64)
        {
            cv::Mat displacementMat, saeMat, partitionMat, timeMat, referenceOriginMat;
            image.copyTo(displacementMat);
            image.copyTo(saeMat);
            image.copyTo(partitionMat);
            image.copyTo(timeMat);
            image.copyTo(referenceOriginMat);

            for (const auto &result : detailedResults)
            {
                cv::rectangle(partitionMat, result.soughtBlock, cv::Vec3i(0, 0, 0));

                double predictionRating = (double) result.predictionError
                    / (double)(result.soughtBlock.width * result.soughtBlock.height);
                cv::rectangle(saeMat, result.soughtBlock, cv::Vec3i(0, (int)(predictionRating), 0), cv::FILLED);

                // auto displacementVec = result.motionVector;
                // auto vx = 128 + (displacementVec[0] * (255 / 64));
                // auto vy = 128 + (displacementVec[1] * (255 / 64));
                // cv::rectangle(displacementMat, result.soughtBlock, cv::Vec3i(vx, vx, vy), cv::FILLED);
            
                auto vx = 255 - (255/maxMVLength)*cv::norm(result.motionVector);
                cv::rectangle(displacementMat, result.soughtBlock, cv::Vec3i(vx, vx, vx), cv::FILLED);

                double predictionTime = result.predictionTime; // /16
                cv::rectangle(timeMat, result.soughtBlock, cv::Vec3i(0, (int)(predictionTime), (int)(predictionTime)), cv::FILLED);
            
                cv::rectangle(referenceOriginMat, result.soughtBlock, cv::Vec3i(result.referenceImageIndex == 0 ? 255 : 0, result.referenceImageIndex == 1 ? 255 : 0, 0), cv::FILLED);
            }

            return {partitionMat, saeMat, displacementMat, timeMat, referenceOriginMat};
        }
    };
}

#endif