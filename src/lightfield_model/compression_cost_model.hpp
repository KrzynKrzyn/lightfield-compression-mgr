#ifndef COMPRESSION_COST_MODEL_HPP
#define COMPRESSION_COST_MODEL_HPP

#include <vector>

namespace lfc
{
    struct CompressionCostCell
    {
        long long blockCount = 0;
        long long predictionError = 0;

        long long parameterAdjustmentTimeCost = 0;

        long long partitionMemoryCost = 0;
        long long partitionTimeCost = 0;

        long long blockMatchingMemoryCost = 0;
        long long blockMatchingTimeCost = 0;

        long long quantizationTimeCost = 0;

        long long filteringMemoryCost = 0;
        long long filteringTimeCost = 0;

        long long encodingMemoryCost = 0;
        long long encodingTimeCost = 0;

        long long encodedDataMemoryCost = 0;

        long long overallMemoCost() const {
            return partitionMemoryCost + blockMatchingMemoryCost + filteringMemoryCost + encodingMemoryCost + encodedDataMemoryCost;
        }

        long long overallTimeCost() const {
            return partitionTimeCost + blockMatchingTimeCost + quantizationTimeCost + filteringTimeCost + encodingTimeCost + parameterAdjustmentTimeCost;
        }
    };

    class CompressionCostModel
    {
    private:
        std::vector<CompressionCostCell> costCells;

    public:
        const int cols;
        const int rows;

        long long parameterAdjustmentTimeCost = 0; //todo

        CompressionCostModel(int cols, int rows) : cols(cols), rows(rows), costCells(cols * rows)
        {
        }

        CompressionCostCell& getCostCell(int col, int row)
        {
            return costCells[row * cols + col];
        }

        CompressionCostCell getOverallCost() const
        {
            CompressionCostCell ret;

            ret.parameterAdjustmentTimeCost = parameterAdjustmentTimeCost; // todo

            for (const auto &c : costCells)
            {
                ret.partitionTimeCost += c.partitionTimeCost;
                ret.partitionMemoryCost += c.partitionMemoryCost;

                ret.blockMatchingTimeCost += c.blockMatchingTimeCost;
                ret.blockMatchingMemoryCost += c.blockMatchingMemoryCost;

                ret.quantizationTimeCost += c.quantizationTimeCost;

                ret.filteringMemoryCost += c.filteringMemoryCost;
                ret.filteringTimeCost += c.filteringTimeCost;

                ret.encodingMemoryCost += c.encodingMemoryCost;
                ret.encodingTimeCost += c.encodingTimeCost;

                ret.encodedDataMemoryCost += c.encodedDataMemoryCost;

                ret.blockCount += c.blockCount;
                ret.predictionError += c.predictionError;
            }

            return ret;
        }
    };
}

#endif