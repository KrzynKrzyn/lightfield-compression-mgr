#ifndef BM_FACTORY_RANDOM_SEARCH_HPP
#define BM_FACTORY_RANDOM_SEARCH_HPP

#include "block_matcher_factory.hpp"
#include "bm_random_search.hpp"

namespace lfc
{
    class BMFactoryRandomSearch : public BlockMatcherFactory
    {
    private:
        const int stepsCount;
        const int neighboursBatchSize;
        const double randomSearchRadius;
        const double normalDistributionRatio;

    public:
        BMFactoryRandomSearch(const int stepsCount,
                              const int neighboursBatchSize,
                              const double randomSearchRadius,
                              const double normalDistributionRatio)
            : stepsCount(stepsCount),
              neighboursBatchSize(neighboursBatchSize),
              randomSearchRadius(randomSearchRadius),
              normalDistributionRatio(normalDistributionRatio) {}

        BlockMatcher *createInstance(const cv::Mat &image, const cv::Mat &referenceImage, const cv::Rect &soughtRegionRect)
        {
            return new BMRandomSearch(referenceImage, image, soughtRegionRect,
                                      stepsCount, neighboursBatchSize, randomSearchRadius, normalDistributionRatio);
        }
    };
}

#endif