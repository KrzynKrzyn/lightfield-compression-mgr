#ifndef LOSSLESS_COMPRESSOR_HPP
#define LOSSLESS_COMPRESSOR_HPP

#include "src/lightfield_model/compression_cost_model.hpp"

#include "src/filters/per_row_predictor.hpp"
#include "src/filters/median_edge_predictor.hpp"
#include "src/filters/paeth_filter.hpp"

#include "src/unravelers/image_leftright_unraveler.hpp"
#include "src/encoders/arithmetic_encoder.hpp"

namespace lfc
{
    class LosslessCompressor {
        public:
        lfc::CompressionCostCell compress(const cv::Mat& inputMat) const {
            lfc::CompressionCostCell costCell;
            std::chrono::steady_clock::time_point timerBegin, timerEnd;

            cv::Mat image;
            inputMat.convertTo(image, CV_16SC3);

            // ============================================================== FILTERING =============================================================
            // lfc::PerRowPredictor filter;
            // lfc::PaethPredictor filter;
            lfc::MedianEdgePredictor filter;

            timerBegin = std::chrono::high_resolution_clock::now();
            auto filteredImage = filter.filter(image);
            cv::subtract(image, filteredImage, filteredImage, cv::noArray(), CV_16SC3);
            timerEnd = std::chrono::high_resolution_clock::now();

            costCell.filteringMemoryCost = inputMat.rows / 9;  // for per row predictor estimation
            costCell.filteringTimeCost = std::chrono::duration_cast<std::chrono::microseconds>(timerEnd - timerBegin).count();

            // ============================================================== ENCODING =============================================================
            lfc::Unraveler<short> *tempUnraveler = new lfc::ImageLeftRightUnraveler<short>(filteredImage);
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