#ifndef BM_FACTORY_PERIMITER_SEARCH_HPP
#define BM_FACTORY_PERIMITER_SEARCH_HPP

#include "block_matcher_factory.hpp"
#include "bm_perimiter_search.hpp"
#include "bm_perimiter_search_with_cutoff.hpp"

namespace lfc
{
    class BMFactoryPerimiterSearch : public BlockMatcherFactory
    {
    private:
        const int neighboursRadius;
        const double cutoffThreshold;

    public:
        BMFactoryPerimiterSearch(const int neighboursRadius, const double cutoffThreshold = -1)
            : neighboursRadius(neighboursRadius), cutoffThreshold(cutoffThreshold) {}

        BlockMatcher *createInstance(const cv::Mat &image, const cv::Mat &referenceImage, const cv::Rect &soughtRegionRect)
        {
            if (cutoffThreshold < 0)
                return new BMPerimiterSearch(referenceImage, image, soughtRegionRect, neighboursRadius);
            else
                return new BMPerimiterSearchWithCutoff(referenceImage, image, soughtRegionRect, neighboursRadius, cutoffThreshold);
        }
    };
}

#endif