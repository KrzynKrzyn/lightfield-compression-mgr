#ifndef BM_SIMULATED_ANNEALING_HPP
#define BM_SIMULATED_ANNEALING_HPP

#include "block_matcher_heuristic.hpp"

namespace lfc
{
    class BMSimulatedAnnealing : public BlockMatcherHeuristic
    {
    private:
        std::uniform_real_distribution<double> distribution = std::uniform_real_distribution<double>(0.0, 1.0);
        const int maxIterationsWithoutTemperatureChange = 128;
        const int neighbourhoodRadius = 8;
        const double temperatureConstant = 10.0;

        double getTemperature(const cv::Point2i &bestPoint, const cv::Point2i &chosenPoint)
        {
            auto temperature = exp(
                -abs(objectiveFunctionWithLookup(bestPoint) - objectiveFunctionWithLookup(chosenPoint)) /
                temperatureConstant);

            // std::cout << abs(objectiveFunctionWithLookup(bestPoint) - objectiveFunctionWithLookup(chosenPoint)) << " " << temperature << std::endl;

            return temperature;
        }

    public:
        BMSimulatedAnnealing(const cv::Mat &referenceImg, const cv::Mat &predictedImg, const cv::Rect &soughtRegionRect,
                              const int neighbourhoodRadius, const int maxIterationsWithoutTemperatureChange)
            : BlockMatcherHeuristic(referenceImg, predictedImg, soughtRegionRect),
              neighbourhoodRadius(neighbourhoodRadius), maxIterationsWithoutTemperatureChange(maxIterationsWithoutTemperatureChange)
        {
        }

        using BlockMatcherHeuristic::matchBlock;

        BlockMatchResults matchBlock(const cv::Point2i &startingPoint) override
        {
            cv::Point2i bestPoint = startingPoint;
            cv::Point2i currentPoint = startingPoint;

            auto neighbours = getShuffledNeighboursWithTabu(bestPoint, neighbourhoodRadius);

            int iterationsSinceTemperatureChange = 0;
            while (!neighbours.empty() && iterationsSinceTemperatureChange++ < maxIterationsWithoutTemperatureChange)
            {
                const auto randomNeighbour = popLast(neighbours);

                if (objectiveFunctionWithLookup(randomNeighbour) < objectiveFunctionWithLookup(bestPoint))
                {
                    currentPoint = bestPoint = randomNeighbour;
                    neighbours = getShuffledNeighboursWithTabu(currentPoint, neighbourhoodRadius);
                    iterationsSinceTemperatureChange = 0;
                }
                else if (distribution(rng) < getTemperature(currentPoint, randomNeighbour))
                {
                    currentPoint = randomNeighbour;
                    neighbours = getShuffledNeighboursWithTabu(currentPoint, neighbourhoodRadius);
                }
            }

            cv::Rect foundRegionRect(bestPoint, soughtRegionRect.size());
            cv::Vec2i foundMotionVector = cv::Vec2i(foundRegionRect.x - soughtRegionRect.x, foundRegionRect.y - soughtRegionRect.y);

            return {soughtRegionRect, foundRegionRect, foundMotionVector, lookupTable[bestPoint], (double)lookupTable.size()};
        }
    };
}

#endif