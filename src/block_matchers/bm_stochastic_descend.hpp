#ifndef BM_STOCHASTIC_DESCEND_HPP
#define BM_STOCHASTIC_DESCEND_HPP

#include "block_matcher_heuristic.hpp"

namespace lfc
{
    class BMStochasticDescend : public BlockMatcherHeuristic
    {
    private:
        const int neighbourhoodRadius;

    public:
        BMStochasticDescend(const cv::Mat &referenceImg, const cv::Mat &predictedImg, const cv::Rect &soughtRegionRect,
                            const int neighbourhoodRadius)
            : BlockMatcherHeuristic(referenceImg, predictedImg, soughtRegionRect),
              neighbourhoodRadius(neighbourhoodRadius)
        {
        }

        using BlockMatcherHeuristic::matchBlock;

        BlockMatchResults matchBlock(const cv::Point2i &startingPoint) override
        {
            cv::Point2i bestPoint = startingPoint;

            auto neighbours = getShuffledNeighboursWithTabu(bestPoint, neighbourhoodRadius);
            const int maxNeighboursSize = (neighbourhoodRadius + neighbourhoodRadius + 1)*(neighbourhoodRadius + neighbourhoodRadius + 1);
            const int neighboursSizeLimit = maxNeighboursSize / 2;

            while (!neighbours.empty())
            // while (neighbours.size() > neighboursSizeLimit)
            {
                const auto randomNeighbour = popLast(neighbours);

                if (objectiveFunctionWithLookup(randomNeighbour) < objectiveFunctionWithLookup(bestPoint))
                {
                    bestPoint = randomNeighbour;
                    neighbours = getShuffledNeighboursWithTabu(bestPoint, neighbourhoodRadius);
                }
            }

            cv::Rect foundRegionRect(bestPoint, soughtRegionRect.size());
            cv::Vec2i foundMotionVector = cv::Vec2i(foundRegionRect.x - soughtRegionRect.x, foundRegionRect.y - soughtRegionRect.y);

            return {soughtRegionRect, foundRegionRect, foundMotionVector, lookupTable[bestPoint], (double)lookupTable.size()};
        }
    };
}

#endif