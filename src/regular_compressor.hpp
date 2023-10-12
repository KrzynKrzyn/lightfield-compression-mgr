#ifndef REGULAR_COMPRESSOR_HPP
#define REGULAR_COMPRESSOR_HPP

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "src/lightfield_model/lightfield_model.hpp"
#include "src/lightfield_model/lightfield_image.hpp"
#include "src/lightfield_model/compression_cost_model.hpp"

#include "src/partitioners/fixed_size_square_partitioner.hpp"
#include "src/partitioners/variable_size_square_partitioner.hpp"
#include "src/partitioners/variable_size_square_bottom_up_partitioner.hpp"

#include "src/predictors/from_reference_predictor.hpp"
#include "src/predictors/prediction_results_encoder.hpp"

#include "src/block_matchers/bm_factory_perimiter_search.hpp"
#include "src/block_matchers/bm_factory_hillclimbing.hpp"
#include "src/block_matchers/bm_factory_stochastic_descend.hpp"
#include "src/block_matchers/bm_factory_simulated_annealing.hpp"
#include "src/block_matchers/bm_factory_test_zone_search.hpp"
#include "src/block_matchers/bm_factory_linear_preference.hpp"
#include "src/block_matchers/bm_factory_random_search.hpp"

#include "src/quantizers/range_quantizer.hpp"
#include "src/quantizers/threshold_quantizer.hpp"

#include "src/filters/median_edge_predictor.hpp"
#include "src/filters/paeth_filter.hpp"
#include "src/filters/shift_vector_filter.hpp"
#include "src/filters/average_filter.hpp"
#include "src/filters/jpeg_filters.hpp"
#include "src/filters/per_row_predictor.hpp"

#include "src/unravelers/image_leftright_unraveler.hpp"
#include "src/unravelers/vector_unraveler.hpp"

#include "src/encoders/run_length_encoder.hpp"
#include "src/encoders/huffman_encoder.hpp"
#include "src/encoders/arithmetic_encoder.hpp"

#include "src/visualization/partitions_painter.hpp"
#include "src/visualization/threshold_painter.hpp"
#include "src/visualization/from_reference_prediction_painter.hpp"

namespace lfc
{
    class RegularCompressor
    {
    private:
        double getDivisionThreshold(double theta) const
        {
            return std::max(0.0, 64.0 * theta);
        }

        int getStochasticIterations(double theta) const
        {
            return -std::min(0, (int)(16 * theta));
        }

        double getSearchDensity(double searchRadius) const
        {
            double searchDensity = 1.0;
            if (searchRadius > 128)
            {
                searchDensity = 128.0 / searchRadius;
            }

            return searchDensity;
        }

        double getPreferedAngle(const LightfieldImage &toCompressImg, const LightfieldImage &referenceImage) const
        {
            int xComponent = referenceImage.col - toCompressImg.col, yComponent = referenceImage.row - toCompressImg.row;
            double distance = sqrt(xComponent * xComponent + yComponent * yComponent);
            double preferedAngle = distance != 0 ? acos(xComponent / distance) : 0.0;
            if (yComponent < 0)
                preferedAngle = -preferedAngle;

            return preferedAngle;
        }

    public:
        CompressionCostCell compress(const LightfieldImage &toCompressImg, const std::vector<LightfieldImage> &referenceImages,
                                     double filteringThreshold, double searchRadius, int epsilon, double theta) const
        {
            double divisionThreshold = getDivisionThreshold(theta);
            int stochasticIterationsCount = getStochasticIterations(theta);

            CompressionCostCell costCell;
            std::chrono::steady_clock::time_point timerBegin, timerEnd;

            // ============================================================== PARTITIONING ==============================================================

            // lfc::FixedSizeSquarePartitioner partitioner(32);
            lfc::VariableSizeSquarePartitioner partitioner(32, 2, divisionThreshold);
            // lfc::VariableSizeSquareBottomUpPartitioner partitioner(32, 2, 12.0);

            timerBegin = std::chrono::high_resolution_clock::now();
            auto partitionResults = partitioner.partitionImage(toCompressImg.image);
            timerEnd = std::chrono::high_resolution_clock::now();

            costCell.partitionMemoryCost = partitionResults.serializedInstructions.size();
            costCell.partitionTimeCost = std::chrono::duration_cast<std::chrono::microseconds>(timerEnd - timerBegin).count();

            costCell.blockCount += partitionResults.partitions.size();

#if VISUALIZE
            // --- PARTITIONING VIZUALIZATION ---
            std::cout << "Block count: " << partitionResults.partitions.size() << std::endl;
            cv::imshow("Partitions", lfc::visualization::PartitionsPainter().paint(toCompressImg.image, partitionResults.partitions));
            cv::waitKey(0);
#endif

            // ============================================================== BLOCK-MATCHING =============================================================

            const auto &referenceImage = referenceImages[0];

            double preferedAngle = getPreferedAngle(toCompressImg, referenceImage);
            double searchDensity = getSearchDensity(searchRadius);

            lfc::StartMVPredictor startMVPredictor({{-1, 0}, {0, -1}, {-1, -1}, {-2, 0}, {0, -2}});
            lfc::BlockMatcherFactory *bmFactory = new lfc::BMFactoryLinearPreference(preferedAngle, searchRadius, searchDensity, stochasticIterationsCount); // length and density
            lfc::FromReferencePredictor fromReferencePredictor(bmFactory, startMVPredictor);

            timerBegin = std::chrono::high_resolution_clock::now();
            lfc::PartitionModel partitionModel(partitionResults.partitions);
            auto predictionResults = fromReferencePredictor.predictImage(toCompressImg.image, referenceImage.image, partitionModel);
            // auto predictionResults = fromReferencePredictor.predictImage(toCompressImg, referenceImages, partitionModel);
            auto encodedPredictionInstructions = lfc::PredictionResultsEncoder().encode(predictionResults);
            timerEnd = std::chrono::high_resolution_clock::now();

            costCell.blockMatchingMemoryCost = encodedPredictionInstructions.size();
            costCell.blockMatchingTimeCost = std::chrono::duration_cast<std::chrono::microseconds>(timerEnd - timerBegin).count();

            for (auto &p : predictionResults.blockMatchResults)
                costCell.predictionError += p.predictionError;

            delete bmFactory;

            auto imagePredictedFromReference = predictionResults.predictedMat;

#if VISUALIZE
            // --- MOTION VECTORS VIZUALIZATION ---
            cv::Mat motionVectorMat(500, 500, CV_8UC3);
            motionVectorMat.setTo(cv::Scalar(200, 128, 128));
            std::unordered_map<cv::Vec2i, size_t> countedVectors;
            for (const auto &bmr : predictionResults.blockMatchResults)
            {
                countedVectors[bmr.motionVector] += bmr.foundBlock.area();
            }

            size_t maxCount = 0;
            for (const auto &cmv : countedVectors)
            {
                if (cmv.second > maxCount)
                    maxCount = cmv.second;
            }

            cv::Vec2i mvMatMiddle{motionVectorMat.cols / 2, motionVectorMat.rows / 2};
            for (const auto &cmv : countedVectors)
            {
                uchar mvPixelValue = 255 - (255 * cmv.second / maxCount);
                auto point = mvMatMiddle + cmv.first;
                if (point[0] < 0 || point[1] < 0 || point[0] >= motionVectorMat.cols || point[1] >= motionVectorMat.rows)
                    continue;
                motionVectorMat.at<cv::Vec3b>(point[1], point[0]) = {mvPixelValue, mvPixelValue, mvPixelValue};
            }

            cv::Vec2i windowSize{100, 100};
            cv::Rect middle(mvMatMiddle - windowSize, mvMatMiddle + windowSize);
            cv::Mat zoomedMotionVectorMat = motionVectorMat(middle);
            cv::resize(motionVectorMat, motionVectorMat, {900, 900}, 0, 0, 0);
            cv::resize(zoomedMotionVectorMat, zoomedMotionVectorMat, {900, 900}, 0, 0, 0);
            cv::line(zoomedMotionVectorMat, {0, 452}, {900, 452}, {0, 0, 0});
            cv::line(zoomedMotionVectorMat, {452, 0}, {452, 900}, {0, 0, 0});

            // std::cout << "Max value: " << maxCount << std::endl;
            cv::imshow("Motion Vectors Mat", motionVectorMat);
            cv::imshow("Zoomed Motion Vectors Mat", zoomedMotionVectorMat);
            cv::waitKey(0);
            cv::destroyAllWindows();

            // --- BLOCK-MATCHING VIZUALIZATION ---
            lfc::visualization::FromReferencePredictionPainter fromReferencePainter;
            auto paintedDetailedResults = fromReferencePainter.paint(toCompressImg.image, predictionResults.blockMatchResults, searchRadius);
            cv::imshow("Reference", referenceImage.image);
            cv::imshow("Original", toCompressImg.image);
            cv::imshow("Predicted", imagePredictedFromReference);
            // cv::imshow("Partitions",  paintedDetailedResults[0]);
            cv::imshow("Quality", paintedDetailedResults[1]);
            cv::imshow("Motion vectors", paintedDetailedResults[2]);
            cv::imshow("Time", paintedDetailedResults[3]);
            cv::imshow("Reference origin", paintedDetailedResults[4]);
            cv::waitKey(0);
            cv::destroyAllWindows();
#endif

            // ============================================================== QUANTIZATION =============================================================

            lfc::RangeQuantizer quantizer(epsilon);
            // lfc::ThresholdQuantizer quantizer(0);

            timerBegin = std::chrono::high_resolution_clock::now();
            cv::Mat quantizedImagePredictedFromReference;
            if (epsilon > 0)
            {
                quantizedImagePredictedFromReference = quantizer.quantize(toCompressImg.image, imagePredictedFromReference);
            }
            else
            {
                quantizedImagePredictedFromReference = toCompressImg.image;
            }
            timerEnd = std::chrono::high_resolution_clock::now();

            costCell.quantizationTimeCost = std::chrono::duration_cast<std::chrono::microseconds>(timerEnd - timerBegin).count();

#if VISUALIZE
            // --- QUANTIZATION VIZUALIZATION ---
            cv::Mat quantizationDiff;
            cv::absdiff(quantizedImagePredictedFromReference, imagePredictedFromReference, quantizationDiff);
            cv::Mat paintedFlattenedRegions = lfc::visualization::ThresholdPainter().paint(quantizationDiff, 0);
            cv::imshow("Predicted regions", paintedFlattenedRegions);

            cv::imshow("Original", toCompressImg.image);
            cv::imshow("Predicted", imagePredictedFromReference);
            cv::imshow("Quantized", quantizedImagePredictedFromReference);

            cv::Mat predflatDiff;
            cv::absdiff(quantizedImagePredictedFromReference, toCompressImg.image, predflatDiff);
            // std::cout << cv::sum(cv::sum(predflatDiff))[0] << std::endl;
            cv::imshow("Quantization loss", 32 * predflatDiff);

            cv::waitKey(0);
            cv::destroyAllWindows();
#endif

            // ============================================================== FILTERING =============================================================

            cv::Mat predictionResiduals;
            cv::subtract(quantizedImagePredictedFromReference, imagePredictedFromReference, predictionResiduals, cv::noArray(), CV_16SC3);

            // lfc::MedianEdgePredictor filter;
            lfc::PerRowPredictor filter;

            timerBegin = std::chrono::high_resolution_clock::now();
            cv::Mat filteredResiduals;
            auto pixelCount = (toCompressImg.image.size().area() * toCompressImg.image.channels());
            double meanPredictionError = ((double)costCell.predictionError / pixelCount);
            if (meanPredictionError <= filteringThreshold)
            {
                filteredResiduals = predictionResiduals;
            }
            else
            {
                filteredResiduals = filter.filter(predictionResiduals);
                cv::subtract(predictionResiduals, filteredResiduals, filteredResiduals, cv::noArray(), CV_16SC3); // TODO: filter should return subtracted filtered values and not predicted values
            }
            timerEnd = std::chrono::high_resolution_clock::now();

            costCell.filteringMemoryCost = 1 + referenceImage.image.rows / 9; // one byte for useFilter flag, the rest for per row predictor estimation
            costCell.filteringTimeCost = std::chrono::duration_cast<std::chrono::microseconds>(timerEnd - timerBegin).count();

            // ============================================================== ENCODING =============================================================
            lfc::Unraveler<short> *tempUnraveler = new lfc::ImageLeftRightUnraveler<short>(filteredResiduals);
            // lfc::HuffmanEncoder<short> encoder(tempUnraveler);
            lfc::ArithmeticEncoder encoder(tempUnraveler);

            timerBegin = std::chrono::high_resolution_clock::now();
            auto encodedBytes = encoder.encode();
            timerEnd = std::chrono::high_resolution_clock::now();

            costCell.encodingMemoryCost = encoder.symbolCount.size() * 4 * 2; // 4 -> 32 bits for freq; 2 -> 16 bits for symbol;
            costCell.encodingTimeCost = std::chrono::duration_cast<std::chrono::microseconds>(timerEnd - timerBegin).count();
            costCell.encodedDataMemoryCost = encodedBytes.size();

            delete tempUnraveler;

            return costCell;
        }
    };
}

#endif