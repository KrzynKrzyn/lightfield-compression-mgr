#ifndef REFERENCE_IMAGE_PICKER_HPP
#define REFERENCE_IMAGE_PICKER_HPP

#include <stdexcept>
#include "src\lightfield_model\lightfield_model.hpp"

namespace lfc
{
    class ReferenceImagePicker
    {
    private:
        int cols, rows;
        int referenceImagesCount;
        std::vector<cv::Point2i> referenceImagePoints;

        std::vector<cv::Point2i> generateReferenceImages() const
        {
            switch (referenceImagesCount)
            {
            case 1:
                return {{cols / 2, rows / 2}};
            case 2:
                return {{cols / 4, rows / 4}, {3 * cols / 4, 3 * rows / 4}};
            case 4:
                return {{cols / 4, rows / 4}, {cols / 4, 3 * rows / 4}, {3 * cols / 4, rows / 4}, {3 * cols / 4, 3 * rows / 4}};
            case 5:
                return {{cols / 2, rows / 2}, {cols / 4 - 1, rows / 4 - 1}, {cols / 4 - 1, 3 * rows / 4 + 1}, {3 * cols / 4 + 1, rows / 4 - 1}, {3 * cols / 4 + 1, 3 * rows / 4 + 1}};
            case -4:
                return {{0, 0}, {cols - 1, 0}, {0, rows - 1}, {cols - 1, rows - 1}};
            default:
                throw std::logic_error("Not implemented error");
            }
        }

    public:
        ReferenceImagePicker(int cols, int rows, int referenceImagesCount)
            : cols(cols), rows(rows), referenceImagesCount(referenceImagesCount), referenceImagePoints(generateReferenceImages())
        {
        }

        ReferenceImagePicker(int cols, int rows, std::vector<cv::Point2i> referenceImagePoints)
            : cols(cols), rows(rows), referenceImagesCount(referenceImagePoints.size()), referenceImagePoints(referenceImagePoints)
        {
        }

        cv::Point2i getReferenceImage(cv::Point2i p) const
        {
            double nearestDistance = 1000000;
            cv::Point2i nearestPoint;
            for (const auto &rp : referenceImagePoints)
            {
                int xDist = p.x - rp.x, yDist = p.y - rp.y;
                double distance = sqrt(xDist * xDist + yDist * yDist);
                if (distance < nearestDistance)
                {
                    nearestDistance = distance;
                    nearestPoint = rp;
                }
            }

            if (p == nearestPoint)
                return {-1, -1};
            else
                return nearestPoint;
        }

        std::vector<cv::Point2i> getReferenceImageBatch(cv::Point2i p, size_t batchSize) const
        {
            double nearestDistance = 1000000;
            auto sortedReferenceImagePoints = referenceImagePoints;
            std::sort(sortedReferenceImagePoints.begin(), sortedReferenceImagePoints.end(), [&](const cv::Point2i &a, const cv::Point2i &b)
                      {
                int xDistA = p.x - a.x, yDistA = p.y - a.y;
                double distanceA = sqrt(xDistA * xDistA + yDistA * yDistA);

                int xDistB = p.x - b.x, yDistB = p.y - b.y;
                double distanceB = sqrt(xDistB * xDistB + yDistB * yDistB);

                return distanceA < distanceB; });

            if (p == sortedReferenceImagePoints[0])
                return {{-1, -1}};
            else
            {
                int trueBatchSize = std::min(batchSize, sortedReferenceImagePoints.size());
                return std::vector<cv::Point2i>(sortedReferenceImagePoints.begin(), sortedReferenceImagePoints.begin() + trueBatchSize);
            }
        }
    };
}

#endif