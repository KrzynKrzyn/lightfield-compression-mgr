#ifndef TEST_DISPARITY_PREDICTOR_HPP
#define TEST_DISPARITY_PREDICTOR_HPP

#include <opencv2/opencv.hpp>

#include "src/lightfield_model/lightfield_image.hpp"
#include "src/lightfield_model/lightfield_model.hpp"

#include "src/partitioners/fixed_size_square_partitioner.hpp"
#include "src/predictors/from_reference_predictor.hpp"
#include "src/block_matchers/bm_factory_linear_preference.hpp"

namespace lfc
{
    class TestDisparityPredictor
    {
    private:
        double getPreferedAngle(const LightfieldImage &toCompressImg, const LightfieldImage &referenceImage) const
        {
            int xComponent = referenceImage.col - toCompressImg.col, yComponent = referenceImage.row - toCompressImg.row;
            double distance = sqrt(xComponent * xComponent + yComponent * yComponent);
            double preferedAngle = distance != 0 ? acos(xComponent / distance) : 0.0; // MINUS acos
            if (yComponent < 0)
                preferedAngle = -preferedAngle;

            return preferedAngle;
        }

        double getCentile(const std::vector<double> &mvLengths, double centile) const
        {
            int mvEsitmateIndex = (int)(centile * mvLengths.size()) - 1;
            size_t clampedmvEstimateIndex = std::clamp(mvEsitmateIndex, 0, (int)mvLengths.size() - 1);
            double mvLengthEstimate = mvLengths[clampedmvEstimateIndex];

            return mvLengthEstimate;
        }

        double getMotionVectorLengthEstimate(const PredictionResults &predictionResults, double scaleUpFactor) const
        {
            std::vector<double> mvLengths;

            for (const auto &result : predictionResults.blockMatchResults)
            {
                const auto mv = result.motionVector;
                const double d1 = (mv[0] * mv[0]), d2 = (mv[1] * mv[1]);
                const auto mvLength = sqrt(d1 + d2);
                mvLengths.push_back(mvLength);
            }

            std::sort(mvLengths.begin(), mvLengths.end());

            double mvLengthEstimate = getCentile(mvLengths, 0.8);

            return mvLengthEstimate;
        }

    public:
        double getDispartyEstimate(const LightfieldImage &toCompressImg, const LightfieldImage &referenceImage) const
        {
            // ============================================================== SCALE-DOWN ==============================================================

            int maxDims = std::max(toCompressImg.image.cols, toCompressImg.image.rows);
            int scaleToDims = 256;

            cv::Mat smallerImage, smallerReferenceImage;
            double scaleUpFactor = std::max(maxDims / scaleToDims, 1);
            double scaleDownFactor = 1 / scaleUpFactor;
            cv::resize(toCompressImg.image, smallerImage,
                       cv::Size(), scaleDownFactor, scaleDownFactor, 0);
            cv::resize(referenceImage.image, smallerReferenceImage,
                       cv::Size(), scaleDownFactor, scaleDownFactor, 0);

            // ============================================================== PARTITIONING ==============================================================

            lfc::FixedSizeSquarePartitioner partitioner(8);

            auto partitionResults = partitioner.partitionImage(smallerImage);

            // ============================================================== BLOCK-MATCHING =============================================================

            double preferedAngle = getPreferedAngle(toCompressImg, referenceImage);
            int lineDistance = std::max(smallerImage.cols, smallerImage.rows);

            lfc::StartMVPredictor startMVPredictor({{-1, 0}, {0, -1}, {-1, -1}, {-2, 0}, {0, -2}});
            lfc::BlockMatcherFactory *bmFactory = new lfc::BMFactoryLinearPreference(preferedAngle, lineDistance, 1.0, 4);
            lfc::FromReferencePredictor fromReferencePredictor(bmFactory, startMVPredictor);
            lfc::PartitionModel partitionModel(partitionResults.partitions);

            auto predictionResults = fromReferencePredictor.predictImage(smallerImage, smallerReferenceImage, partitionModel);

            delete bmFactory;

            // ============================================================== MV ESTIMATION =============================================================

            auto mvLengthEstimate = getMotionVectorLengthEstimate(predictionResults, scaleUpFactor);
            auto scaledUpMVEstimate = mvLengthEstimate * scaleUpFactor;

            // std::cout << "MV Length Estimate: " << scaledUpMVEstimate << std::endl;

            return scaledUpMVEstimate;
        }
    };

}

#endif