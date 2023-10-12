#ifndef REFERENCE_IMAGES_SCHEME_PREDICTOR_HPP
#define REFERENCE_IMAGES_SCHEME_PREDICTOR_HPP

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include "src/parameter_adjuster/test_disparity_predictor.hpp"
#include "src/lossless_compressor.hpp"
#include "src/regular_compressor.hpp"

namespace lfc
{
    class ReferenceImagesSchemePredictor
    {
    private:
        const LightfieldModel lfModel;
        const std::pair<double, double> bppLinearParamsNoFilter;
        const std::pair<double, double> bppLinearParamsFilter;
        const double referenceImageBpp;

        double predictedImageBpp(double distanceFromReference)
        {
            double bppNoFilter = bppLinearParamsNoFilter.first * distanceFromReference + bppLinearParamsNoFilter.second;
            double bppFilter = bppLinearParamsFilter.first * distanceFromReference + bppLinearParamsFilter.second;

            return std::min(bppFilter, bppNoFilter);
        }

        double getDistanceToReferenceImage(cv::Point2i p, std::set<cv::Point2i> solution) const
        {
            double nearestDistance = 1000000;
            for (const auto &rp : solution)
            {
                int xDist = p.x - rp.x, yDist = p.y - rp.y;
                double distance = sqrt(xDist * xDist + yDist * yDist);
                if (distance < nearestDistance)
                {
                    nearestDistance = distance;
                }
            }

            return nearestDistance;
        }

        double solutionScore(std::set<cv::Point2i> solution)
        {
            cv::Mat bppMat(cv::Size(lfModel.cols, lfModel.rows), CV_32FC1);
            bppMat.setTo(0.0);

            for (const auto &s : solution)
            {
                bppMat.at<float>(s.x, s.y) = referenceImageBpp;
            }

            for (int x = 0; x < bppMat.cols; ++x)
                for (int y = 0; y < bppMat.rows; ++y)
                {
                    auto &bpp = bppMat.at<float>(x, y);
                    if (bpp == 0)
                        bpp = predictedImageBpp(getDistanceToReferenceImage({x, y}, solution));
                }

            return cv::sum(bppMat)[0];
        }

        bool canInsertCoordinate(std::set<cv::Point2i> coords, cv::Point2i newCoordinate) const
        {
            if (
                newCoordinate.x < 0 || newCoordinate.x >= lfModel.cols ||
                newCoordinate.y < 0 || newCoordinate.y >= lfModel.rows)
                return false;
            return coords.find(newCoordinate) == coords.end();
        }

        std::vector<cv::Point2i> neighbouringPoints(cv::Point2i p) const
        {
            return {
                {p.x + 1, p.y + 1},
                {p.x, p.y + 1},
                {p.x - 1, p.y + 1},
                {p.x + 1, p.y},
                {p.x - 1, p.y},
                {p.x + 1, p.y - 1},
                {p.x, p.y - 1},
                {p.x - 1, p.y - 1},
            };
        }

        std::vector<std::set<cv::Point2i>> getNeighbouringSolutions(std::set<cv::Point2i> solution)
        {
            std::vector<std::set<cv::Point2i>> newSolutions;
            for (auto &refPoint : solution)
            {
                std::set<cv::Point2i> solutionWithoutCurrentRefPoint(solution);
                solutionWithoutCurrentRefPoint.erase(refPoint);

                for (auto &possibleNewPoint : neighbouringPoints(refPoint))
                {
                    if (canInsertCoordinate(solutionWithoutCurrentRefPoint, possibleNewPoint))
                    {
                        newSolutions.push_back(std::set<cv::Point2i>((solutionWithoutCurrentRefPoint)));
                        newSolutions.back().insert(possibleNewPoint);
                    }
                }
            }

            return newSolutions;
        }

        std::pair<std::set<cv::Point2i>, double> getBest(std::vector<std::set<cv::Point2i>> neighbours)
        {
            double bestScore = std::numeric_limits<double>::max();
            int bestSolutionIndex = -1;
            for (int i = 0; i < neighbours.size(); ++i)
            {
                const auto &n = neighbours[i];
                double score = solutionScore(n);
                if (score < bestScore)
                {
                    bestSolutionIndex = i;
                    bestScore = score;
                }
            }

            return {neighbours[bestSolutionIndex], bestScore};
        }

    public:
        ReferenceImagesSchemePredictor(const LightfieldModel &lfModel,
                                       const std::pair<double, double> &bppLinearParamsNoFilter,
                                       const std::pair<double, double> &bppLinearParamsFilter,
                                       const double &referenceImageBpp)
            : lfModel(lfModel),
              bppLinearParamsNoFilter(bppLinearParamsNoFilter),
              bppLinearParamsFilter(bppLinearParamsFilter),
              referenceImageBpp(referenceImageBpp)
        {
        }

        std::set<cv::Point2i> hillclimbing(std::set<cv::Point2i> start)
        {
            std::set<cv::Point2i> current = start;
            std::set<cv::Point2i> previous = current;

            double bestScore = std::numeric_limits<double>::max();
            do
            {
                const auto neighbours = getNeighbouringSolutions(current);
                if (neighbours.empty())
                    break;

                const auto best = getBest(neighbours);

                if (best.second >= bestScore)
                    break;
                else
                    bestScore = best.second;

                previous = current;
                current = best.first;
            } while (previous != current);

            return current;
        }

        std::set<cv::Point2i> initSolution(int count)
        {
            std::set<cv::Point2i> ret;

            for (int x = 0; x < lfModel.cols;)
                for (int y = 0; y < lfModel.rows; x += 1, y += 1)
                {
                    ret.insert({x, y});
                    if (ret.size() >= count)
                        return ret;
                }

            return ret;
        }

        std::set<cv::Point2i> predictReferenceScheme()
        {
            std::set<cv::Point2i> bestSolutionAllTime{{0, 0}};
            double bestScoreAllTime = solutionScore(bestSolutionAllTime);
            double bestScoreIteration = bestScoreAllTime;

            int referenceImageCount = 0;
            while (++referenceImageCount < 20)
            {
                auto init = initSolution(referenceImageCount);
                auto hillclimbedSolution = hillclimbing(init);

                bestScoreIteration = solutionScore(hillclimbedSolution);
                if (bestScoreIteration < bestScoreAllTime)
                {
                    bestSolutionAllTime = hillclimbedSolution;
                    bestScoreAllTime = bestScoreIteration;
                }
                else
                {
                    break;
                }
            }

            return bestSolutionAllTime;
        }
    };
}

#endif