#ifndef PARTITION_MODEL_HPP
#define PARTITION_MODEL_HPP

#include <opencv2/opencv.hpp>

namespace lfc
{
    class PartitionModel
    {
        int minPartitionSize;
        int maxPartitionSize;
        int minX, maxX, minY, maxY;
        int rows, cols;

        std::vector<int> positionIndexToPartitionIndex;
        std::vector<cv::Rect> partitions;

        int colRowToPositionIndex(int col, int row) const
        {
            return (row * cols) + col;
        }

        std::pair<int, int> positionIndexToColRow(int positionIndex) const 
        {
            return {(positionIndex % rows), (positionIndex / rows)};
        }

        std::vector<std::pair<int, int>> getColsRows(cv::Rect partition) const
        {
            std::vector<std::pair<int, int>> ret;

            int colCount = partition.width / minPartitionSize;
            int rowCount = partition.height / minPartitionSize;

            int col = partition.x / minPartitionSize;
            int row = partition.y / minPartitionSize;

            for (int i = 0; i < colCount; ++i)
            {
                for (int j = 0; j < rowCount; ++j)
                {
                    ret.push_back({col + i, row + j});
                }
            }

            return ret;
        }

        std::array<int, 4> getEdgePositionIndices(cv::Rect partition) const
        {
            auto colsRows = getColsRows(partition);

            int minCol = colsRows.front().first;
            int maxCol = colsRows.back().first;
            int minRow = colsRows.front().second;
            int maxRow = colsRows.back().second;

            return {{
                colRowToPositionIndex(minCol, minRow),
                colRowToPositionIndex(maxCol, minRow),
                colRowToPositionIndex(minCol, maxRow),
                colRowToPositionIndex(maxCol, maxRow),
            }};
        }

        std::vector<int> getPositionIndices(cv::Rect partition) const
        {
            std::vector<int> ret;

            auto colsRows = getColsRows(partition);
            for (const auto &cr : colsRows)
                ret.push_back(colRowToPositionIndex(cr.first, cr.second));

            return ret;
        }

        int getNeighbouringPositionIndex(int positionIndex, int colIncrement, int rowIncrement) const 
        {
            auto colRow = positionIndexToColRow(positionIndex);
            int newCol = colRow.first + colIncrement;
            int newRow = colRow.second + rowIncrement;

            if (newCol < minX || newCol > maxX) return -1;
            if (newRow < minY || newRow > maxY) return -1;

            return colRowToPositionIndex(newCol, newRow);
        }

        int getPartitionIndex(cv::Rect partition) const
        {
            auto positionIndex = getPositionIndices(partition)[0];
            return positionIndexToPartitionIndex[positionIndex];
        }

    public:
        PartitionModel(const std::vector<cv::Rect> &partitions) : partitions(partitions)
        {
            minPartitionSize = minX = minY = std::numeric_limits<int>::max();
            maxPartitionSize = maxX = maxY = 0;
            for (const auto &p : partitions)
            {
                minPartitionSize = std::min({p.height, p.width, minPartitionSize});
                maxPartitionSize = std::max({p.height, p.width, maxPartitionSize});
                minX = std::min({p.x, minX});
                minY = std::min({p.y, minY});
                maxX = std::max({p.x + p.width, maxX});
                maxY = std::max({p.y + p.height, maxY});
            }

            cols = (maxX - minX) / minPartitionSize;
            rows = (maxY - minY) / minPartitionSize;

            positionIndexToPartitionIndex = std::vector<int>(cols * rows, -1);
            for (int i = 0; i < partitions.size(); ++i)
            {
                auto positionIndices = getPositionIndices(partitions[i]);
                for (const auto &pi : positionIndices)
                    positionIndexToPartitionIndex[pi] = i;
            }
        }

        cv::Rect getPartition(int partitionIndex) const
        {
            return partitions[partitionIndex];
        }

        int getNeighbouringPartitionIndex(int partitionIndex, int colIncrement, int rowIncrement) const
        {
            cv::Rect partition = partitions[partitionIndex];
            auto edgePositionIndices = getEdgePositionIndices(partition);
            int colComponent = colIncrement <= 0 ? 0 : 1;
            int rowComponent = rowIncrement <= 0 ? 0 : 2;
            int positionIndex = edgePositionIndices[colComponent + rowComponent];
            int neighbouringPositionIndex = getNeighbouringPositionIndex(positionIndex, colIncrement, rowIncrement);
            // std::cout << "!!!" << neighbouringPositionIndex << std::endl;

            if (neighbouringPositionIndex < 0 || neighbouringPositionIndex >= rows * cols)
                return -1;

            int neighbouringPartitionIndex = positionIndexToPartitionIndex[neighbouringPositionIndex];
            // std::cout << "???" << partitionIndex << "->" << neighbouringPartitionIndex << std::endl;
            return neighbouringPartitionIndex;
        }

        cv::Rect getNeighbouringPartition(int partitionIndex, int colIncrement, int rowIncrement) const
        {
            int neighbouringPartitionIndex = getNeighbouringPartitionIndex(partitionIndex, colIncrement, rowIncrement);
            if (neighbouringPartitionIndex < 0) return cv::Rect(0, 0, 0, 0);
            
            cv::Rect neighbouringPartition = partitions[neighbouringPartitionIndex];
            return neighbouringPartition;
        }

        cv::Rect getLeftPartition(int partitionIndex) const
        {
            return getNeighbouringPartition(partitionIndex, -1, 0);
        }

        cv::Rect getUpperPartition(int partitionIndex) const
        {
            return getNeighbouringPartition(partitionIndex, 0, -1);
        }

        cv::Rect getUpperLeftPartition(int partitionIndex) const
        {
            return getNeighbouringPartition(partitionIndex, -1, -1);
        }

        size_t size() const
        {
            return partitions.size();
        }
    };
}

#endif