#ifndef BM_PERIMITER_SEARCH_WITH_CUTOFF_HPP
#define BM_PERIMITER_SEARCH_WITH_CUTOFF_HPP

#include "block_matcher_heuristic.hpp"

namespace lfc
{
    class BMPerimiterSearchWithCutoff : public BlockMatcherHeuristic
    {
    private:
        const int perimiterRadius;
        const double cutoffThreshold;

        const std::array<cv::Vec2i, 4> circleVecs = {{
            {-1, 0},
            {0, 1},
            {1, 0},
            {0, -1},
        }};

        std::vector<cv::Vec2i> getScanIndices(int count)
        {
            std::vector<cv::Vec2i> ret;

            int x = 0, y = 0;
            int vecIndex = 0;
            int lineLength = 1;
            int remainingLineLength = 1;
            int remainingLineSequences = 2;
            for (int i = 0; i < count; ++i)
            {
                ret.push_back(cv::Vec2i(x, y));

                remainingLineLength--;
                if (remainingLineLength <= 0)
                {
                    vecIndex = (vecIndex + 1) % circleVecs.size();

                    remainingLineLength = lineLength;
                    remainingLineSequences--;
                    if (remainingLineSequences <= 0)
                    {
                        lineLength++;
                        remainingLineSequences = 2;
                    };
                }

                x += circleVecs[vecIndex][0];
                y += circleVecs[vecIndex][1];
            }

            return ret;
        }

    public:
        BMPerimiterSearchWithCutoff(const cv::Mat &referenceImg, const cv::Mat &predictedImg, const cv::Rect &soughtRegionRect,
                       const int perimiterRadius, const double cutoffThreshold)
            : BlockMatcherHeuristic(referenceImg, predictedImg, soughtRegionRect),
              perimiterRadius(perimiterRadius),
              cutoffThreshold(cutoffThreshold)
        {
        }

        using BlockMatcherHeuristic::matchBlock;

        BlockMatchResults matchBlock(const cv::Point2i &startingPoint) override
        {
            cv::Point2i bestPoint = startingPoint;
            int searchArea = (perimiterRadius + perimiterRadius + 1) * (perimiterRadius + perimiterRadius + 1);
            double cutoffThresholdForBlock = cutoffThreshold * soughtRegionRect.area() * soughtRegion.channels();
            for(const auto& scanIndex : getScanIndices(searchArea)) {
                cv::Point2i neighbour(startingPoint.x + scanIndex[0], startingPoint.y + scanIndex[1]);
                if (!isPointInsideLimits(neighbour)) continue;

                objectiveFunctionWithLookup(neighbour);

                if (lookupTable[neighbour] < lookupTable[bestPoint]) {
                    bestPoint = neighbour;
                }

                if (lookupTable[bestPoint] <= cutoffThresholdForBlock) break;
            }

            cv::Rect foundRegionRect(bestPoint, soughtRegionRect.size());
            cv::Vec2i foundMotionVector = cv::Vec2i(foundRegionRect.x - soughtRegionRect.x, foundRegionRect.y - soughtRegionRect.y);

            return {soughtRegionRect, foundRegionRect, foundMotionVector, lookupTable[bestPoint], (double)lookupTable.size()};
        }
    };
}

#endif