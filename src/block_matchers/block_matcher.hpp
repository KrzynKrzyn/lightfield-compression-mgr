#ifndef BLOCK_MATCHER_HPP
#define BLOCK_MATCHER_HPP

#include <vector>
#include <opencv2/opencv.hpp>

#include "block_matcher_results.hpp"

template <>
struct std::hash<cv::Point2i>
{
    std::size_t operator()(const cv::Point2i &s) const noexcept
    {
        return (s.x + 10000) + s.y;
    }
};

template <>
struct std::less<cv::Vec2i>
{
    bool operator()(const cv::Vec2i &v1, const cv::Vec2i &v2) const noexcept
    {
        return (v1[0] == v2[0] ? v1[1] < v2[1] : v1[0] < v2[0]);
    }
};

template <>
struct std::less<cv::Point2i>
{
    bool operator()(const cv::Point2i &p1, const cv::Point2i &p2) const noexcept
    {
        return (p1.x == p2.x ? p1.y < p2.y : p1.x < p2.x);
    }
};

namespace lfc
{
    class BlockMatcher
    {
    protected:
        const cv::Mat referenceImg;
        const cv::Mat predictedImg;

        const cv::Rect soughtRegionRect;
        const cv::Mat soughtRegion;

        const int minX, maxX, minY, maxY;

        bool isPointInsideLimits(const cv::Point2i &point) const
        {
            return point.x >= minX && point.x <= maxX && point.y >= minY && point.y <= maxY;
        }

        cv::Point2i getNearestPointInsideLimits(const cv::Point2i &point) const
        {
            int x = point.x, y = point.y;

            if (point.x < minX)
                x = minX;
            else if (point.x > maxX)
                x = maxX;

            if (point.y < minY)
                y = minY;
            else if (point.y > maxY)
                y = maxY;

            return cv::Point2i(x, y);
        }

    public:
        BlockMatcher(const cv::Mat &referenceImg, const cv::Mat &predictedImg, const cv::Rect &soughtRegionRect)
            : referenceImg(referenceImg),
              predictedImg(predictedImg),
              soughtRegionRect(soughtRegionRect),
              soughtRegion(predictedImg(soughtRegionRect)),
              minX(0), maxX(referenceImg.cols - soughtRegionRect.width),
              minY(0), maxY(referenceImg.rows - soughtRegionRect.height)
        {
        }

        virtual ~BlockMatcher() {
            
        }

        virtual BlockMatchResults matchBlock(const cv::Point2i &startingPoint) = 0;

        BlockMatchResults matchBlock(const std::set<cv::Point2i> &startingPoints)
        {
            std::vector<BlockMatchResults> predictions;
            for (const auto &p : startingPoints)
            {
                predictions.push_back(matchBlock(p));
            }

            BlockMatchResults bestPrediction = *std::min_element(
                predictions.begin(),
                predictions.end(),
                [](const auto &a, const auto &b)
                {
                    return a.predictionError < b.predictionError;
                });

            return bestPrediction;
        }

        BlockMatchResults matchBlock(const cv::Vec2i &startingVec)
        {
            auto startingPoint = cv::Point2i(soughtRegionRect.x + startingVec[0], soughtRegionRect.y + startingVec[1]);
            startingPoint = getNearestPointInsideLimits(startingPoint);
            return matchBlock(startingPoint);
        }

        BlockMatchResults matchBlock(const std::set<cv::Vec2i> &startingVecs)
        {
            std::set<cv::Point2i> startingPoints;
            for (const auto &v : startingVecs)
            {
                auto point = cv::Point2i(soughtRegionRect.x + v[0], soughtRegionRect.y + v[1]);
                auto nearestPoint = getNearestPointInsideLimits(point);
                startingPoints.insert(nearestPoint);
            }

            return matchBlock(startingPoints);
        }

        BlockMatchResults matchBlock()
        {
            return matchBlock(cv::Point2i(soughtRegionRect.x, soughtRegionRect.y));
        }
    };
}

#endif