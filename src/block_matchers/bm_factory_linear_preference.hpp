#ifndef BM_FACTORY_LINEAR_PREFERENCE_HPP
#define BM_FACTORY_LINEAR_PREFERENCE_HPP

#include "block_matcher_factory.hpp"
#include "bm_linear_preference.hpp"

namespace lfc
{
    class BMFactoryLinearPreference : public BlockMatcherFactory
    {
    private:
        const double angleRadians;
        const double stride;
        const int pointCount;
        const int rasterSearchRadius;
        const int rasterSearchStride;
        const int stochasticIterations;

        double calculatePreferedAngle(const LightfieldImage &image, const LightfieldImage &referenceImage) {
            int xComponent = referenceImage.col - image.col, yComponent = referenceImage.row - image.row;
            double distance = sqrt(xComponent * xComponent + yComponent * yComponent);
            double preferedAngle = distance != 0 ? acos(xComponent / distance) : 0.0; // MINUS acos
            if (yComponent < 0)
                preferedAngle = -preferedAngle;

            return preferedAngle;
        }

    public:
        using BlockMatcherFactory::createInstance;

        // count and stride
        BMFactoryLinearPreference(const double angleRadians,
                                  const double stride,
                                  const int pointCount,
                                  const int rasterSearchRadius,
                                  const int rasterSearchStride,
                                  const int stochasticIterations)
            : angleRadians(angleRadians),
              stride(stride),
              pointCount(pointCount),
              rasterSearchRadius(rasterSearchRadius),
              rasterSearchStride(rasterSearchStride),
              stochasticIterations(stochasticIterations) {}

        // length and denisty
        BMFactoryLinearPreference(const double angleRadians,
                                  const double lineLength,
                                  const double lineDensity,
                                  const int stochasticIterations)
            : angleRadians(angleRadians),
              rasterSearchRadius(1),
              rasterSearchStride(1),
              stride(1.414213562373 / lineDensity),
              pointCount(lineLength / stride),
              stochasticIterations(stochasticIterations)
        {
        }

        BlockMatcher *createInstance(const cv::Mat &image, const cv::Mat &referenceImage, const cv::Rect &soughtRegionRect)
        {
            return new BMLinearPreference(referenceImage, image, soughtRegionRect,
                                          angleRadians, stride, pointCount,
                                          rasterSearchRadius, rasterSearchStride,
                                          stochasticIterations);
        }

        BlockMatcher* createInstance(const LightfieldImage &image, const LightfieldImage &referenceImage, const cv::Rect &soughtRegionRect) override
        {
            const double preferedAngle = calculatePreferedAngle(image, referenceImage);
            return new BMLinearPreference(referenceImage.image, image.image, soughtRegionRect,
                                          preferedAngle, stride, pointCount,
                                          rasterSearchRadius, rasterSearchStride,
                                          stochasticIterations);
        }
    };
}

#endif