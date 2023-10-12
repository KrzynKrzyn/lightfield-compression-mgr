#ifndef BM_RANDOM_SEARCH_HPP
#define BM_RANDOM_SEARCH_HPP

#include <random>
#include "block_matcher_heuristic.hpp"

namespace lfc
{
    class BMRandomSearch : public BlockMatcherHeuristic
    {
    private:
        const double M_PI = 3.1415926535897932384626433832795028841971693993751058209;

        std::uniform_real_distribution<double> uniformDistribution = std::uniform_real_distribution<double>(0.0, 1.0);
        std::uniform_real_distribution<double> uniformDistributionMinusOneToOne = std::uniform_real_distribution<double>(-1.0, 1.0);
        std::normal_distribution<double> normalDistribution = std::normal_distribution<double>(0.0);

        const int stepsCount;
        const int neighboursBatchSize;
        const double randomSearchRadius;
        const double normalDistributionRatio;

    protected:
        std::vector<cv::Point2i> getRandomNeighbours(const cv::Point2i &point, double maxRadius, int neighboursCount, double normalDistributionRatio = 0.5)
        {
            std::set<cv::Point2i> neighbouringPoints;
            for (int i = 0; i < neighboursCount; ++i)
            {
                double theta = M_PI * 2.0 * uniformDistribution(rng);

                bool useNormalDistribution = uniformDistribution(rng) < normalDistributionRatio;
                double randomRadiusComponent = useNormalDistribution
                                                   ? std::abs(normalDistribution(rng))
                                                   : uniformDistribution(rng);

                double radius = maxRadius * sqrt(randomRadiusComponent);

                double x = point.x + radius * cos(theta);
                double y = point.y + radius * sin(theta);

                cv::Point2i newPoint(std::round(x), std::round(y));
                // if(isPointInsideLimits(newPoint)) neighbouringPoints.insert(newPoint);
                neighbouringPoints.insert(getNearestPointInsideLimits(newPoint));
            }

            std::vector<cv::Point2i> ret(neighbouringPoints.begin(), neighbouringPoints.end());
            return ret;
        }

        std::vector<cv::Point2i> getRandomNeighboursSquare(const cv::Point2i &point, double maxRadius, int neighboursCount)
        {
            std::set<cv::Point2i> neighbouringPoints;
            for (int i = 0; i < neighboursCount; ++i)
            {
                double xShift = maxRadius * uniformDistributionMinusOneToOne(rng);
                double yShift = maxRadius * uniformDistributionMinusOneToOne(rng);

                double x = point.x + xShift;
                double y = point.y + yShift;

                cv::Point2i newPoint(std::round(x), std::round(y));
                // if(isPointInsideLimits(newPoint)) neighbouringPoints.insert(newPoint);
                neighbouringPoints.insert(getNearestPointInsideLimits(newPoint));
            }

            std::vector<cv::Point2i> ret(neighbouringPoints.begin(), neighbouringPoints.end());
            return ret;
        }

    public:
        BMRandomSearch(const cv::Mat &referenceImg, const cv::Mat &predictedImg, const cv::Rect &soughtRegionRect,
                       const int stepsCount,
                       const int neighboursBatchSize,
                       const double randomSearchRadius,
                       const double normalDistributionRatio)
            : BlockMatcherHeuristic(referenceImg, predictedImg, soughtRegionRect),
              stepsCount(stepsCount),
              neighboursBatchSize(neighboursBatchSize),
              randomSearchRadius(randomSearchRadius),
              normalDistributionRatio(normalDistributionRatio)
        {
        }

        using BlockMatcherHeuristic::matchBlock;

        BlockMatchResults matchBlock(const cv::Point2i &startingPoint) override
        {
            cv::Point2i bestPoint = startingPoint;

            objectiveFunctionWithLookup(startingPoint);
            for (int i = 0; i < stepsCount; ++i)
            {
                auto neighbours = getRandomNeighbours(bestPoint, randomSearchRadius, neighboursBatchSize, normalDistributionRatio);
                if (neighbours.empty()) continue;

                cv::Point2i bestNeighbour = getBest(neighbours);
                if (lookupTable[bestNeighbour] < lookupTable[bestPoint])
                    bestPoint = bestNeighbour;
            }

            cv::Rect foundRegionRect(bestPoint, soughtRegionRect.size());
            cv::Vec2i foundMotionVector = cv::Vec2i(foundRegionRect.x - soughtRegionRect.x, foundRegionRect.y - soughtRegionRect.y);

            return {soughtRegionRect, foundRegionRect, foundMotionVector, lookupTable[bestPoint], (double)lookupTable.size()};
        }
    };
}

#endif