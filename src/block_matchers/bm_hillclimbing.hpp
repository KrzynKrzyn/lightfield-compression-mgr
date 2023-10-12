#ifndef BM_HILLCLIMBING_HPP
#define BM_HILLCLIMBING_HPP

#include "block_matcher_heuristic.hpp"

namespace lfc
{
    class BMHillclimbing : public BlockMatcherHeuristic
    {
    private:
        const int neighboursRadius;
    public:
        BMHillclimbing(const cv::Mat &referenceImg, const cv::Mat &predictedImg, const cv::Rect &soughtRegionRect,
                         const int neighboursRadius)
            : BlockMatcherHeuristic(referenceImg, predictedImg, soughtRegionRect),
              neighboursRadius(neighboursRadius)
        {
        }

        using BlockMatcherHeuristic::matchBlock;

        BlockMatchResults matchBlock(const cv::Point2i &startingPoint) override
        {
            cv::Point2i currentPoint = startingPoint;
            cv::Point2i previousPoint = currentPoint;

            do
            {
                const auto neighbours = getNeighbours(currentPoint, neighboursRadius);
                if (neighbours.empty()) break;

                const auto bestNeighbour = getBest(neighbours);
                
                previousPoint = currentPoint;
                currentPoint = bestNeighbour;
            } while (previousPoint != currentPoint);

            cv::Point2i bestPoint = currentPoint;
            cv::Rect foundRegionRect(bestPoint, soughtRegionRect.size());
            cv::Vec2i foundMotionVector = cv::Vec2i(foundRegionRect.x - soughtRegionRect.x, foundRegionRect.y - soughtRegionRect.y);

            return {soughtRegionRect, foundRegionRect, foundMotionVector, lookupTable[bestPoint], (double)lookupTable.size()};
        }
    };
}

#endif