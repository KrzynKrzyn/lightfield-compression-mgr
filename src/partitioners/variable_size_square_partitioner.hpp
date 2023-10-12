#ifndef VARIABLE_SIZE_SQUARE_PARTITIONER_HPP
#define VARIABLE_SIZE_SQUARE_PARTITIONER_HPP

#include <vector>
#include <opencv2/opencv.hpp>

#include "partitioner.hpp"

namespace lfc
{
    class VariableSizeSquarePartitioner : public Partitioner
    {
    private:
        int baseBlockSize;
        int maxSubdivisions;
        double ratingThreshold;

        enum PartitionInstruction
        {
            nosplit,
            split
        };

        double calculatePartitionRating(const cv::Mat &mat) const
        {
            cv::Scalar mean, stddev;
            cv::meanStdDev(mat, mean, stddev);

            return stddev[0];
        }

        std::array<cv::Rect, 4> quadSplitRect(const cv::Rect &rect)
        {
            auto smallerSize = rect.size() / 2;

            cv::Point2i point00(rect.x, rect.y);
            cv::Point2i point01(rect.x + smallerSize.width, rect.y);
            cv::Point2i point10(rect.x, rect.y + smallerSize.height);
            cv::Point2i point11(rect.x + smallerSize.width, rect.y + smallerSize.height);

            cv::Rect rect00(point00, smallerSize);
            cv::Rect rect01(point01, smallerSize);
            cv::Rect rect10(point10, smallerSize);
            cv::Rect rect11(point11, smallerSize);

            return std::array<cv::Rect, 4>{rect00, rect01, rect10, rect11};
        }

        void subdivideImageBlock(const cv::Mat &mat, const cv::Rect &rect, int subdivisionsLeft, std::vector<cv::Rect> &blockAccumulator, std::vector<PartitionInstruction> &instructions)
        {
            cv::Mat region = mat(rect);
            double rating = calculatePartitionRating(region);

            if (rating < ratingThreshold || subdivisionsLeft <= 0)
            {
                if (subdivisionsLeft > 0)
                    instructions.push_back(PartitionInstruction::nosplit);
                blockAccumulator.push_back(rect);
            }
            else
            {
                instructions.push_back(PartitionInstruction::split);
                auto quadRect = quadSplitRect(rect);
                for (const auto &r : quadRect)
                {
                    subdivideImageBlock(mat, r, subdivisionsLeft - 1, blockAccumulator, instructions);
                }
            }
        }

        std::vector<uint8_t> serializeInstructions(const std::vector<PartitionInstruction> &instructions) const
        {
            const size_t bitsInByte = 8; 
            size_t byteCount = (instructions.size() + bitsInByte - 1) / bitsInByte;
            size_t paddingSize = (byteCount * bitsInByte) - instructions.size();

            std::vector<uint8_t> bytes(byteCount, 0);

            int bufferedBitsCount = 0;
            size_t byteIndex = 0;
            for (int i = 0; i < instructions.size(); ++i)
            {
                bytes[byteIndex] <<= 1;
                if(instructions[i] == PartitionInstruction::split) ++bytes[byteIndex];

                if (++bufferedBitsCount == bitsInByte) {
                    byteIndex++;
                    bufferedBitsCount = 0;
                }
            }

            if(paddingSize > 0) bytes[byteIndex] <<= paddingSize;

            return bytes;
        }

    public:
        VariableSizeSquarePartitioner(int baseBlockSize, int maxSubdivisions, double ratingThreshold)
            : baseBlockSize(baseBlockSize), maxSubdivisions(maxSubdivisions), ratingThreshold(ratingThreshold) {}

        PartitionResults partitionImage(const cv::Mat &image) override
        {
            std::vector<cv::Rect> partitions;
            std::vector<PartitionInstruction> instructions;

            int blockCols = image.cols / baseBlockSize;
            int blockRows = image.rows / baseBlockSize;

            for (int j = 0; j < blockRows; ++j)
                for (int i = 0; i < blockCols; ++i)
                {
                    cv::Rect roiRect(baseBlockSize * i, baseBlockSize * j, baseBlockSize, baseBlockSize);
                    subdivideImageBlock(image, roiRect, maxSubdivisions, partitions, instructions);
                }

            return {partitions, serializeInstructions(instructions)};
        }
    };
}

#endif