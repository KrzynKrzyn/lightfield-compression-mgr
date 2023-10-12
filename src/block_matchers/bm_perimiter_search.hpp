#ifndef BM_PERIMITER_SEARCH_HPP
#define BM_PERIMITER_SEARCH_HPP

#include "block_matcher_heuristic.hpp"

namespace lfc
{
    class BMPerimiterSearch : public BlockMatcherHeuristic
    {
    private:
        const int perimiterRadius;

    public:
        BMPerimiterSearch(const cv::Mat &referenceImg, const cv::Mat &predictedImg, const cv::Rect &soughtRegionRect,
                       const int perimiterRadius)
            : BlockMatcherHeuristic(referenceImg, predictedImg, soughtRegionRect),
              perimiterRadius(perimiterRadius)
        {
        }

        using BlockMatcherHeuristic::matchBlock;

        BlockMatchResults matchBlock(const cv::Point2i &startingPoint) override
        {
            cv::Point2i bestPoint = getBest(getNeighbours(startingPoint, perimiterRadius));

            cv::Rect foundRegionRect(bestPoint, soughtRegionRect.size());
            cv::Vec2i foundMotionVector = cv::Vec2i(foundRegionRect.x - soughtRegionRect.x, foundRegionRect.y - soughtRegionRect.y);

            return {soughtRegionRect, foundRegionRect, foundMotionVector, lookupTable[bestPoint], (double)lookupTable.size()};
        }
    };
}

#endif