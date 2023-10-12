#ifndef BM_TEST_ZONE_SEARCH_HPP
#define BM_TEST_ZONE_SEARCH_HPP

#include "block_matcher_heuristic.hpp"

namespace lfc
{
    class BMTestZoneSearch : public BlockMatcherHeuristic
    {
    private:
        const int maxStride = 64;
        const int refinementStartStride = 32;
        const int rasterSearchRadius = 10;
        const int rasterSearchStride = 2;

    protected:
        cv::Point getBestFromOutline(const cv::Point2i point, const cv::Point2i bestPoint, int radius)
        {
            const auto neighbours = getNeighboursOutlineSquare(point, radius);
            if (neighbours.empty()) return bestPoint;

            const auto bestNeighbour = getBest(neighbours);
            if (lookupTable[bestNeighbour] < lookupTable[bestPoint])
            {
                return bestNeighbour;
            }
            else
            {
                return bestPoint;
            }
        }

        cv::Point getBestFromRasterSearch(const cv::Point2i point, const cv::Point2i bestPoint)
        {
            const auto neighbours = getNeighboursGrid(point, rasterSearchRadius, rasterSearchStride);
            if (neighbours.empty()) return bestPoint;
            
            const auto bestNeighbour = getBest(neighbours);
            if (lookupTable[bestNeighbour] < lookupTable[bestPoint])
            {
                return bestNeighbour;
            }
            else
            {
                return bestPoint;
            }
        }

    public:
        BMTestZoneSearch(const cv::Mat &referenceImg, const cv::Mat &predictedImg, const cv::Rect &soughtRegionRect,
                         const int maxStride, const int refinementStartStride, const int rasterSearchRadius, const int rasterSearchStride)
            : BlockMatcherHeuristic(referenceImg, predictedImg, soughtRegionRect),
              maxStride(maxStride),
              refinementStartStride(refinementStartStride),
              rasterSearchRadius(rasterSearchRadius),
              rasterSearchStride(rasterSearchStride)
        {
        }

        using BlockMatcherHeuristic::matchBlock;

        BlockMatchResults matchBlock(const cv::Point2i &startingPoint) override
        {
            cv::Point2i bestPoint = startingPoint;
            objectiveFunctionWithLookup(startingPoint);

            for (int i = 1; i <= maxStride; i *= 2)
            {
                bestPoint = getBestFromOutline(startingPoint, bestPoint, i);
            }

            // double distance = cv::norm(startingPoint-bestPoint);
            // if (distance >= rasterSearchRadius)
            if (bestPoint != startingPoint)
            {
                bestPoint = getBestFromRasterSearch(bestPoint, bestPoint);

                for (int i = refinementStartStride; i > 0; i /= 2)
                {
                    bestPoint = getBestFromOutline(bestPoint, bestPoint, i);
                }
            }

            cv::Rect foundRegionRect(bestPoint, soughtRegionRect.size());
            cv::Vec2i foundMotionVector = cv::Vec2i(foundRegionRect.x - soughtRegionRect.x, foundRegionRect.y - soughtRegionRect.y);

            return {soughtRegionRect, foundRegionRect, foundMotionVector, lookupTable[bestPoint], (double)lookupTable.size()};
        }
    };
}

#endif