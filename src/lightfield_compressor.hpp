#ifndef LIGHT_FIELD_COMPRESSOR_HPP
#define LIGHT_FIELD_COMPRESSOR_HPP

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>
#include <filesystem>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "src/lightfield_model/lightfield_model.hpp"
#include "src/lightfield_model/compression_cost_model.hpp"

#include "src/parameter_adjuster/reference_image_picker.hpp"
#include "src/parameter_adjuster/test_compressor.hpp"

#include "src/lossless_compressor.hpp"
#include "src/regular_compressor.hpp"

#include "src/visualization/lightfield_map_painter.hpp"

namespace lfc
{
    class LightfieldCompressor
    {
    public:
        lfc::CompressionCostModel compress(const lfc::LightfieldModel &lfModel, int epsilon, double theta) const
        {
            lfc::CompressionCostModel costModel(lfModel.cols, lfModel.rows);

            lfc::RegularCompressor regularCompressor;
            lfc::LosslessCompressor losslessCompressor;
            lfc::TestCompressor testCompressor;

            auto timerBegin = std::chrono::high_resolution_clock::now();
            auto testResults = testCompressor.testCompress(lfModel, epsilon, theta);
            auto timerEnd = std::chrono::high_resolution_clock::now();
            costModel.parameterAdjustmentTimeCost = std::chrono::duration_cast<std::chrono::microseconds>(timerEnd - timerBegin).count();

            lfc::ReferenceImagePicker referenceImagePicker(lfModel.cols, lfModel.rows, testResults.referenceImagePattern);

            #if !(VISUALIZE)
            #pragma omp parallel for /*num_threads(6)*/
            #endif
            for (int i = 0; i < lfModel.rows * lfModel.cols; i += 1)
            {
                int y = i / lfModel.cols;
                int x = i % lfModel.rows;

                std::cout << "Processing: (" << x << ", " << y << ")" << std::endl;

                lfc::LightfieldImage toCompressImage = lfModel.getLightfieldImage(x, y);

                auto referenceImagePositions = referenceImagePicker.getReferenceImageBatch({x, y}, 1);
                auto firstReferenceImagePosition = referenceImagePositions[0];
                // auto referenceImagePosition = referenceImagePicker.getReferenceImage({x, y});

                if (firstReferenceImagePosition.x < 0 || firstReferenceImagePosition.y < 0)
                {
                    costModel.getCostCell(x, y) = losslessCompressor.compress(toCompressImage.image);
                }
                else
                {
                    std::vector<lfc::LightfieldImage> lfRefImages;
                    for (const auto &refPosition : referenceImagePositions)
                    {
                        lfc::LightfieldImage referenceImage = lfModel.getLightfieldImage(refPosition.x, refPosition.y);
                        lfRefImages.push_back(referenceImage);
                    }

                    double cameraDistanceX = (firstReferenceImagePosition.x - x), cameraDistanceY = (firstReferenceImagePosition.y - y);
                    double cameraDistance = sqrt(cameraDistanceX * cameraDistanceX + cameraDistanceY * cameraDistanceY);
                    double searchRadius = testResults.searchRadius(cameraDistance);
                    costModel.getCostCell(x, y) = regularCompressor.compress(toCompressImage, lfRefImages, testResults.filteringThreshold, searchRadius, epsilon, theta);
                }

                std::cout << " BPP: " << 8.0 * ((double)costModel.getCostCell(x, y).overallMemoCost() / (double)toCompressImage.image.size().area()) << std::endl;
            }

            return costModel;
        }
    };
}

#endif