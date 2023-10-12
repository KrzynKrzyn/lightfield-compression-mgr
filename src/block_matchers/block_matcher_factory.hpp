#ifndef BLOCK_MATCHER_FACTORY_HPP
#define BLOCK_MATCHER_FACTORY_HPP

#include <vector>
#include <opencv2/opencv.hpp>

#include "block_matcher.hpp"
#include "block_matcher_results.hpp"

#include "src/lightfield_model/lightfield_image.hpp"

namespace lfc
{
    class BlockMatcherFactory
    {
    private:
    public:
        virtual BlockMatcher *createInstance(const cv::Mat &image, const cv::Mat &referenceImage, const cv::Rect &soughtRegionRect) = 0;

        virtual BlockMatcher *createInstance(const LightfieldImage &image, const LightfieldImage &referenceImage, const cv::Rect &soughtRegionRect)
        {
            return createInstance(image.image, referenceImage.image, soughtRegionRect);
        }

        virtual ~BlockMatcherFactory()
        {
        }
    };
}

#endif