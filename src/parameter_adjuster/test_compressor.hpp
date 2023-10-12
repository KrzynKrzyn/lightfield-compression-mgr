#ifndef TEST_COMPRESSOR_HPP
#define TEST_COMPRESSOR_HPP

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "src/parameter_adjuster/test_disparity_predictor.hpp"
#include "src/parameter_adjuster/reference_images_scheme_predictor.hpp"

#include "src/lossless_compressor.hpp"
#include "src/regular_compressor.hpp"

namespace lfc
{
    class TestCompressor
    {
    private:
        RegularCompressor regularCompressor;

        struct CompressionParameters
        {
            std::vector<cv::Point2i> referenceImagePattern;
            double filteringThreshold; // prediction error threshold below which the filtering should be skipped
            std::pair<double, double> searchRadiusLinearParams;

            double searchRadius(double distanceToReferenceImage) {
                return (distanceToReferenceImage * searchRadiusLinearParams.first) + searchRadiusLinearParams.second;
            }
        };

        std::pair<double, double> getLinearParameters(double nearestBpp, double nearestDistance, double furthestBpp, double furthestDistance)
        {
            double a = (nearestBpp - furthestBpp) / (nearestDistance - furthestDistance);
            double b = nearestBpp - (nearestDistance * a);

            return {a, b};
        }

        std::pair<double, double> getLinearIntersection(std::pair<double, double> paramOne, std::pair<double, double> paramTwo)
        {
            double x = (paramTwo.second - paramOne.second) / (paramOne.first - paramTwo.first);
            double y = paramOne.first * x + paramOne.second;

            return {x, y};
        }

        double getDistance(const cv::Point2i& p1, const cv::Point2i& p2) const {
            int d1 = p1.x - p2.x, d2 = p1.y - p2.y;
            return sqrt(d1*d1 + d2*d2);
        }

    public:
        CompressionParameters testCompress(const LightfieldModel &lfModel, int epsilon, double theta)
        {
            cv::Point2i referenceImagePos{lfModel.cols / 2, lfModel.rows / 2};
            cv::Point2i furthestImagePos{0, lfModel.rows - 1};
            cv::Point2i nearestImagePos{lfModel.cols / 2 - 1, lfModel.rows / 2 + 1};

            if (lfModel.cols < 6 || lfModel.rows < 6) {
                referenceImagePos = {lfModel.cols - 1, 0};
                furthestImagePos = {0, lfModel.rows - 1};
                nearestImagePos = {lfModel.cols - 2, 1};
            }

            auto referenceImage = lfModel.getLightfieldImage(referenceImagePos.x, referenceImagePos.y);
            auto furthestImage = lfModel.getLightfieldImage(furthestImagePos.x, furthestImagePos.y);
            auto nearestImage = lfModel.getLightfieldImage(nearestImagePos.x, nearestImagePos.y);

            double nearestDistance = getDistance(referenceImagePos, nearestImagePos);
            double furthestDistance = getDistance(referenceImagePos, furthestImagePos);

            // PREDICT DISPARITY BEGIN
            lfc::TestDisparityPredictor disparityPreditor;
            double nearestSearchRadius = disparityPreditor.getDispartyEstimate(nearestImage, referenceImage);
            double furthestSearchRadius = disparityPreditor.getDispartyEstimate(furthestImage, referenceImage);

            auto searchRadiusLinearParams = getLinearParameters(nearestSearchRadius, nearestDistance, furthestSearchRadius, furthestDistance);
            // PREDICT DISPARITY END

            double xDistance = (furthestImage.col - nearestImage.col), yDistance = (furthestImage.row - nearestImage.row);
            double nearestFurthestDistance = sqrt((xDistance * xDistance) + (yDistance * yDistance));

            CompressionCostCell nearestCostCellFilter = regularCompressor.compress(nearestImage, {referenceImage}, 0.0, nearestSearchRadius, epsilon, theta);
            CompressionCostCell furthestCostCellFilter = regularCompressor.compress(furthestImage, {referenceImage}, 0.0, furthestSearchRadius, epsilon, theta);
            CompressionCostCell nearestCostCellNoFilter = regularCompressor.compress(nearestImage, {referenceImage}, std::numeric_limits<double>::max(), nearestSearchRadius, epsilon, theta);
            CompressionCostCell furthestCostCellNoFilter = regularCompressor.compress(furthestImage, {referenceImage}, std::numeric_limits<double>::max(), furthestSearchRadius, epsilon, theta);
            CompressionCostCell referenceCostCell = LosslessCompressor().compress(referenceImage.image);

            double nearestBppFilter = 8.0 * ((double)nearestCostCellFilter.overallMemoCost() / (double)nearestImage.image.size().area());
            double nearestBppNoFilter = 8.0 * ((double)nearestCostCellNoFilter.overallMemoCost() / (double)nearestImage.image.size().area());
            double furthestBppFilter = 8.0 * ((double)furthestCostCellFilter.overallMemoCost() / (double)nearestImage.image.size().area());
            double furthestBppNoFilter = 8.0 * ((double)furthestCostCellNoFilter.overallMemoCost() / (double)nearestImage.image.size().area());
            double referenceBpp = 8.0 * ((double)referenceCostCell.overallMemoCost() / (double)nearestImage.image.size().area());

            // std::cout << "NF  BPP: " << nearestBppFilter << std::endl;
            // std::cout << "NnF BPP: " << nearestBppNoFilter << std::endl;
            // std::cout << "FF  BPP: " << furthestBppFilter << std::endl;
            // std::cout << "FnF BPP: " << furthestBppNoFilter << std::endl;
            // std::cout << "Ref BPP: " << referenceBpp << std::endl;

            // PREDICT FILTERING THRESHOLD BEGIN
            auto filterLinearParams = getLinearParameters(nearestBppFilter, nearestDistance, furthestBppFilter, furthestDistance);
            auto noFilterLinearParams = getLinearParameters(nearestBppNoFilter, nearestDistance, furthestBppNoFilter, furthestDistance);

            auto linearIntersection = getLinearIntersection(filterLinearParams, noFilterLinearParams);
            double thresholdDistance = linearIntersection.first;

            auto errorParams = getLinearParameters(nearestCostCellNoFilter.predictionError, nearestDistance, furthestCostCellNoFilter.predictionError, furthestDistance);
            auto thresholdError = (thresholdDistance * errorParams.first + errorParams.second) / (referenceImage.image.size().area() * referenceImage.image.channels());
            // PREDICT FILTERING THRESHOLD END

            // PREDICT REFERENCE SCHEME BEGIN
            ReferenceImagesSchemePredictor risp(lfModel, noFilterLinearParams, filterLinearParams, referenceBpp);
            auto referencePointsArrangement = risp.predictReferenceScheme();
            // PREDICT REFERENCE SCHEME END

            CompressionParameters parameters;
            parameters.filteringThreshold = thresholdError;
            parameters.referenceImagePattern = std::vector<cv::Point2i>(referencePointsArrangement.begin(), referencePointsArrangement.end());
            parameters.searchRadiusLinearParams = searchRadiusLinearParams;

            return parameters;
        }
    };
}

#endif