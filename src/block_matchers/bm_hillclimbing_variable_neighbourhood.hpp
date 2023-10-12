#ifndef BM_HILLCLIMBING_VARIABLE_NEIGHBOURHOOD_HPP
#define BM_HILLCLIMBING_VARIABLE_NEIGHBOURHOOD_HPP

#include "block_matcher_heuristic.hpp"

namespace lfc
{
    class BMHillclimbingVariableNeighbourhood : public BlockMatcherHeuristic
    {
    private:
        const int maxRadius = 9;
        const int minRadius = 1;
        const double maxScore = 64.0;
        const double minScore = 0.0;

        const double aComponent = (double)(maxRadius - minRadius) / (double)(maxScore - minScore);
        const double bComponent = (double)(maxRadius) - (maxScore*aComponent);

        int getNeighbourhoodRadius(const cv::Point2i& bestPoint) {
            auto score = lookupTable[bestPoint];
            auto errorPerPixel = score / soughtRegionRect.area();
            int neighbourhoodRadius = std::lround(aComponent * errorPerPixel + bComponent);
            int clampedRadius = std::clamp(neighbourhoodRadius, minRadius, maxRadius);

            return clampedRadius;
        }

    public:
        BMHillclimbingVariableNeighbourhood(const cv::Mat &referenceImg, const cv::Mat &predictedImg, const cv::Rect &soughtRegionRect)
            : BlockMatcherHeuristic(referenceImg, predictedImg, soughtRegionRect)
        {
        }

        using BlockMatcherHeuristic::matchBlock;

        BlockMatchResults matchBlock(const cv::Point2i &startingPoint) override
        {
            cv::Point2i bestPoint = startingPoint;
            cv::Point2i previousPoint = bestPoint;
            objectiveFunctionWithLookup(bestPoint);
            
            do
            {
                const auto neighbours = getNeighbours(bestPoint, getNeighbourhoodRadius(bestPoint));
                if (neighbours.empty()) break;

                const auto bestNeighbour = getBest(neighbours);
                previousPoint = bestPoint;
                if (lookupTable[bestNeighbour] < lookupTable[bestPoint])
                    bestPoint = bestNeighbour;
            } while (previousPoint != bestPoint);

            cv::Rect foundRegionRect(bestPoint, soughtRegionRect.size());
            cv::Vec2i foundMotionVector = cv::Vec2i(foundRegionRect.x - soughtRegionRect.x, foundRegionRect.y - soughtRegionRect.y);

            return {soughtRegionRect, foundRegionRect, foundMotionVector, lookupTable[bestPoint], (double)lookupTable.size()};
        }
    };
}

#endif