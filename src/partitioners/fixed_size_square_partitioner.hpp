#ifndef FIXED_SIZE_SQUARE_PARTITIONER_HPP
#define FIXED_SIZE_SQUARE_PARTITIONER_HPP

#include <vector>
#include <opencv2/opencv.hpp>

#include "partitioner.hpp"

namespace lfc
{
    class FixedSizeSquarePartitioner : public Partitioner
    {
    private:
        int baseBlockSize;

    public:
        FixedSizeSquarePartitioner(int baseBlockSize = 32)
            : baseBlockSize(baseBlockSize) {}

        PartitionResults partitionImage(const cv::Mat &image) override
        {
            std::vector<cv::Rect> ret;

            int blockCols = image.cols / baseBlockSize;
            int blockRows = image.rows / baseBlockSize;

            for (int j = 0; j < blockRows; ++j)
                for (int i = 0; i < blockCols; ++i)
                {
                    cv::Rect roiRect(baseBlockSize * i, baseBlockSize * j, baseBlockSize, baseBlockSize);
                    ret.push_back(roiRect);
                }

            return {ret, {}};
        }
    };
}

#endif