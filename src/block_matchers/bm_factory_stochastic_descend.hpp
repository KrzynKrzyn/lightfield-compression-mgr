#ifndef BM_FACTORY_STOCHASTIC_DESCEND_HPP
#define BM_FACTORY_STOCHASTIC_DESCEND_HPP

#include "block_matcher_factory.hpp"
#include "bm_stochastic_descend.hpp"

namespace lfc
{
    class BMFactoryStochasticDescend : public BlockMatcherFactory
    {
    private:
        const int neighboursRadius;

    public:
        BMFactoryStochasticDescend(const int neighboursRadius)
            : neighboursRadius(neighboursRadius) {}

        BlockMatcher *createInstance(const cv::Mat &image, const cv::Mat &referenceImage, const cv::Rect &soughtRegionRect)
        {
            return new BMStochasticDescend(referenceImage, image, soughtRegionRect, neighboursRadius);
        }
    };
}

#endif