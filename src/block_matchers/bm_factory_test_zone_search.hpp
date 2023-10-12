#ifndef BM_FACTORY_TEST_ZONE_SEARCH_HPP
#define BM_FACTORY_TEST_ZONE_SEARCH_HPP

#include "block_matcher_factory.hpp"
#include "bm_test_zone_search.hpp"

namespace lfc
{
    class BMFactoryTestZoneSearch : public BlockMatcherFactory
    {
    private:
        const int maxStride = 64;
        const int refinementStartStride = 32;
        const int rasterSearchRadius = 10;
        const int rasterSearchStride = 2;

    public:
        BMFactoryTestZoneSearch(const int maxStride,
                                const int refinementStartStride,
                                const int rasterSearchRadius,
                                const int rasterSearchStride)
            : maxStride(maxStride),
              refinementStartStride(refinementStartStride),
              rasterSearchRadius(rasterSearchRadius),
              rasterSearchStride(rasterSearchStride) {}

        BlockMatcher *createInstance(const cv::Mat &image, const cv::Mat &referenceImage, const cv::Rect &soughtRegionRect)
        {
            return new BMTestZoneSearch(referenceImage, image, soughtRegionRect, maxStride, refinementStartStride, rasterSearchRadius, rasterSearchStride);
        }
    };
}

#endif