#ifndef BM_FACTORY_SIMULATED_ANNEALING_HPP
#define BM_FACTORY_SIMULATED_ANNEALING_HPP

#include "block_matcher_factory.hpp"
#include "bm_simulated_annealing.hpp"

namespace lfc
{
    class BMFactorySimulatedAnnealing : public BlockMatcherFactory
    {
    private:
        const int neighboursRadius;
        const int maxIterationsWithoutTemperatureChange;

    public:
        BMFactorySimulatedAnnealing(const int neighboursRadius, const int maxIterationsWithoutTemperatureChange)
            : neighboursRadius(neighboursRadius), maxIterationsWithoutTemperatureChange(maxIterationsWithoutTemperatureChange) {}

        BlockMatcher *createInstance(const cv::Mat &image, const cv::Mat &referenceImage, const cv::Rect &soughtRegionRect)
        {
            return new BMSimulatedAnnealing(referenceImage, image, soughtRegionRect, neighboursRadius, maxIterationsWithoutTemperatureChange);
        }
    };
}

#endif