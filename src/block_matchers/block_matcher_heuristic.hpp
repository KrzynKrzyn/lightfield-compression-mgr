#ifndef BLOCK_MATCHER_HEURISTIC_HPP
#define BLOCK_MATCHER_HEURISTIC_HPP

#include <random>
#include <opencv2/opencv.hpp>

#include "block_matcher.hpp"

namespace lfc
{
    std::default_random_engine rng(19977);

    class BlockMatcherHeuristic : public BlockMatcher
    {
    protected:
        // predicted anchor point to prediction quality
        std::unordered_map<cv::Point2i, double> lookupTable;

        double calculateSumOfSquaredError(const cv::Mat &mat1, const cv::Mat &mat2) const
        {
            cv::Mat tempMat;
            cv::absdiff(mat1, mat2, tempMat);

            auto sumOfErrors = tempMat.dot(tempMat);

            return sumOfErrors;
        }

        double calculateSumOfAbsoluteError(const cv::Mat &mat1, const cv::Mat &mat2) const
        {
            cv::Mat tempMat;
            cv::absdiff(mat1, mat2, tempMat);

            auto sumOfErrorsPerChannel = cv::sum(tempMat);
            auto sumOfErrors = cv::sum(sumOfErrorsPerChannel)[0];

            return sumOfErrors;
        }

        double objectiveFunction(const cv::Point2i &point) const
        {
            cv::Rect regionRect(point, soughtRegionRect.size());
            cv::Mat region = referenceImg(regionRect);

            return calculateSumOfAbsoluteError(soughtRegion, region);
        }

        double objectiveFunctionWithLookup(const cv::Point2i &p)
        {
            if (lookupTable.find(p) == lookupTable.end())
            {
                lookupTable[p] = objectiveFunction(p);
            }

            return lookupTable[p];
        }

        std::vector<cv::Point2i> getNeighbours(const cv::Point2i &point, const int radius) const
        {
            std::vector<cv::Point2i> ret;
            ret.reserve((1 + radius + radius) * (1 + radius + radius));

            for (int x = -radius; x <= radius; x++)
            {
                for (int y = -radius; y <= radius; y++)
                {
                    cv::Point2i newPoint(point.x + x, point.y + y);
                    if (isPointInsideLimits(newPoint))
                        ret.push_back(newPoint);
                }
            }

            return ret;
        }

        std::vector<cv::Point2i> getNeighboursOutlineSquare(const cv::Point2i &point, const int radius) const
        {
            std::vector<cv::Point2i> ret;
            ret.reserve(8);

            const std::vector<cv::Vec2i> vecs = {
                {radius, radius},
                {radius, 0},
                {radius, -radius},
                {0, radius},
                {0, -radius},
                {-radius, radius},
                {-radius, 0},
                {-radius, -radius},
            };

            for (const auto &v : vecs)
            {
                cv::Point2i newPoint(point.x + v[0], point.y + v[1]);
                if (isPointInsideLimits(newPoint))
                    ret.push_back(newPoint);
            }

            return ret;
        }

        std::vector<cv::Point2i> getNeighboursGrid(const cv::Point2i &point, const int radius, const int stride) const
        {
            std::vector<cv::Point2i> ret;
            ret.reserve((1 + radius + radius) * (1 + radius + radius) / (stride * stride));

            for (int x = -radius; x <= radius; x += stride)
            {
                for (int y = -radius; y <= radius; y += stride)
                {
                    cv::Point2i newPoint(point.x + x, point.y + y);
                    if (isPointInsideLimits(newPoint))
                        ret.push_back(newPoint);
                }
            }

            return ret;
        }

        std::vector<cv::Point2i> getNeighboursWithTabu(const cv::Point2i &point, const int radius) const
        {
            std::vector<cv::Point2i> ret;
            ret.reserve((1 + radius + radius) * (1 + radius + radius));
            for (const auto &n : getNeighbours(point, radius))
            {
                if (lookupTable.find(n) == lookupTable.end())
                {
                    ret.push_back(n);
                }
            }

            return ret;
        }

        std::vector<cv::Point2i> getShuffledNeighbours(const cv::Point2i &point, const int radius) const
        {
            std::vector<cv::Point2i> ret = getNeighbours(point, radius);
            std::shuffle(ret.begin(), ret.end(), rng);
            return ret;
        }

        std::vector<cv::Point2i> getShuffledNeighboursWithTabu(const cv::Point2i &point, const int radius) const
        {
            std::vector<cv::Point2i> ret = getNeighboursWithTabu(point, radius);
            std::shuffle(ret.begin(), ret.end(), rng);
            return ret;
        }

        cv::Point2i getBest(const std::vector<cv::Point2i> &neighbours)
        {
            if (neighbours.size() == 1) objectiveFunctionWithLookup(neighbours.back());

            return *std::min_element(
                neighbours.begin(),
                neighbours.end(),
                [&](const auto &a, const auto &b)
                {
                    return objectiveFunctionWithLookup(a) < objectiveFunctionWithLookup(b);
                });
        }

        cv::Point2i popLast(std::vector<cv::Point2i> &neighbours)
        {
            auto ret = neighbours.back();
            neighbours.pop_back();
            return ret;
        }

    public:
        using BlockMatcher::BlockMatcher;
        using BlockMatcher::matchBlock;

        virtual ~BlockMatcherHeuristic() {
            
        }

        BlockMatchResults matchBlockPickBetterStart(const std::set<cv::Vec2i> &startingVecs)
        {
            std::vector<cv::Point2i> startingPoints;
            for (const auto &v : startingVecs)
            {
                auto point = cv::Point2i(soughtRegionRect.x + v[0], soughtRegionRect.y + v[1]);
                auto nearestPoint = getNearestPointInsideLimits(point);
                startingPoints.push_back(nearestPoint);
            }

            cv::Point2i bestStartingPoint = getBest(startingPoints);

            return matchBlock(bestStartingPoint);
        }

        cv::Mat drawLookup() const {
            
        }
    };
}

#endif