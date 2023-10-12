#ifndef BM_FACTORY_HILLCLIMBING_HPP
#define BM_FACTORY_HILLCLIMBING_HPP

#include "block_matcher_factory.hpp"
#include "bm_hillclimbing.hpp"

namespace lfc
{
    class BMFactoryHillclimbing : public BlockMatcherFactory
    {
    private:
        const int neighboursRadius;

    public:
        BMFactoryHillclimbing(const int neighboursRadius)
            : neighboursRadius(neighboursRadius) {}

        BlockMatcher *createInstance(const cv::Mat &image, const cv::Mat &referenceImage, const cv::Rect &soughtRegionRect)
        {
            return new BMHillclimbing(referenceImage, image, soughtRegionRect, neighboursRadius);
        }
    };
}

#endif