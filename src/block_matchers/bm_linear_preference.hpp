#ifndef BM_LINEAR_PREFERENCE_HPP
#define BM_LINEAR_PREFERENCE_HPP

#include <random>
#include "block_matcher_heuristic.hpp"

namespace lfc
{
    class BMLinearPreference : public BlockMatcherHeuristic
    {
    private:
        const double angleRadians;
        const double stride;
        const int pointCount;

        const int rasterSearchRadius;
        const int rasterSearchStride;

        const int stochasticIterations = 4;
        const int stochasticRadius = 4;
        const int stochasticBatchSize = 10;
        std::uniform_real_distribution<double> uniformDistribution = std::uniform_real_distribution<double>(-1.0, 1.0);

    protected:
        std::vector<cv::Point2i> getNeighboursLine(const cv::Point2i &point, const double angleRadians, const int countInOneDirection, const double stride) const
        {
            std::vector<cv::Point2i> ret;
            const int count = 2 * countInOneDirection;
            ret.reserve(1 + count);

            double incrementX = stride * std::cos(angleRadians);
            double incrementY = stride * std::sin(angleRadians);
            cv::Point2i previousPoint = point;
            for (int i = -countInOneDirection; i <= countInOneDirection; ++i)
            {
                cv::Point2i newPoint((int)std::round(point.x + i * incrementX), (int)std::round(point.y + i * incrementY));
                if (isPointInsideLimits(newPoint))
                    ret.push_back(newPoint);
            }

            return ret;
        }

        std::vector<cv::Point2i> getRandomNeighbours(const cv::Point2i &point, int radius, int neighboursCount)
        {
            std::set<cv::Point2i> neighbouringPoints;
            for (int i = 0; i < neighboursCount; ++i)
            {
                double xShift = radius * uniformDistribution(rng);
                double yShift = radius * uniformDistribution(rng);

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
        BMLinearPreference(const cv::Mat &referenceImg, const cv::Mat &predictedImg, const cv::Rect &soughtRegionRect,
                           const double angleRadians,
                           const double stride,
                           const int pointCount,
                           const int rasterSearchRadius,
                           const int rasterSearchStride,
                           const int stochasticIterations)
            : BlockMatcherHeuristic(referenceImg, predictedImg, soughtRegionRect),
              angleRadians(angleRadians),
              stride(stride),
              pointCount(pointCount),
              rasterSearchRadius(rasterSearchRadius),
              rasterSearchStride(rasterSearchStride),
              stochasticIterations(stochasticIterations)
        {
        }

        using BlockMatcherHeuristic::matchBlock;

        BlockMatchResults matchBlock(const cv::Point2i &startingPoint) override
        {
            auto lineNeighbours = getNeighboursLine(startingPoint, angleRadians, pointCount, stride);
            cv::Point2i bestPoint = getBest(lineNeighbours);

            auto rasterSearchNeighbours = getNeighboursGrid(bestPoint, rasterSearchRadius, rasterSearchStride);
            cv::Point2i bestRasterSearchPoint = getBest(rasterSearchNeighbours);
            if (lookupTable[bestRasterSearchPoint] < lookupTable[bestPoint])
                bestPoint = bestRasterSearchPoint;

            int iterationsWithNoChange = 0;
            for (int i = 0; i < stochasticIterations; ++i)
            {
                auto randomSearchNeighbours = getRandomNeighbours(bestPoint, stochasticRadius, stochasticBatchSize);
                if (randomSearchNeighbours.empty())
                    continue;

                auto bestRandomNeighbour = getBest(randomSearchNeighbours);
                if (lookupTable[bestRandomNeighbour] < lookupTable[bestPoint])
                {
                    iterationsWithNoChange = 0;
                    bestPoint = bestRandomNeighbour;
                }
                // else if (++iterationsWithNoChange >= 3)
                //     break;
            }

            cv::Rect foundRegionRect(bestPoint, soughtRegionRect.size());
            cv::Vec2i foundMotionVector = cv::Vec2i(foundRegionRect.x - soughtRegionRect.x, foundRegionRect.y - soughtRegionRect.y);

            return {soughtRegionRect, foundRegionRect, foundMotionVector, lookupTable[bestPoint], (double)lookupTable.size()};
        }
    };
}

#endif