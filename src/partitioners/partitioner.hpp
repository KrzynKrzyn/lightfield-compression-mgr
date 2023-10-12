#ifndef PARTITIONER_HPP
#define PARTITIONER_HPP

#include <vector>
#include <opencv2/opencv.hpp>

namespace lfc
{
    struct PartitionResults
    {
        std::vector<cv::Rect> partitions;
        std::vector<uint8_t> serializedInstructions;

        PartitionResults(const std::vector<cv::Rect> &partitions, const std::vector<uint8_t> &serializedInstructions)
            : partitions(partitions), serializedInstructions(serializedInstructions) {}
    };

    class Partitioner
    {
    private:
    public:
        virtual PartitionResults partitionImage(const cv::Mat &image) = 0;
        virtual ~Partitioner() {}
    };
}

#endif