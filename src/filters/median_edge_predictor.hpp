#ifndef MEDIAN_EDGE_PREDICTOR_HPP
#define MEDIAN_EDGE_PREDICTOR_HPP

#include <opencv2/opencv.hpp>
#include "shift_vector_filter.hpp"

namespace lfc
{
    class MedianEdgePredictor : public Filter
    {
    private:
        cv::Mat getMatWithMinValues(const cv::Mat &a, const cv::Mat &b) const
        {
            cv::Mat aMin = a < b;
            cv::Mat ret;
            b.copyTo(ret);
            a.copyTo(ret, aMin);

            return ret;
        }

        cv::Mat getMatWithMaxValues(const cv::Mat &a, const cv::Mat &b) const
        {
            cv::Mat aMax = a > b;
            cv::Mat ret;
            b.copyTo(ret);
            a.copyTo(ret, aMax);

            return ret;
        }

    protected:
        cv::Mat getMEDPrediction(const cv::Mat &image, int dtype = -1) const
        {
            cv::Mat b = lfc::ShiftVectorFilter({0, 1}).filter(image);
            cv::Mat a = lfc::ShiftVectorFilter({1, 0}).filter(image);
            cv::Mat c = lfc::ShiftVectorFilter({1, 1}).filter(image);

            cv::Mat cMax1 = c >= a;
            cv::Mat cMax2 = c >= b;
            cv::Mat cMax;
            cv::bitwise_and(cMax1, cMax2, cMax);

            cv::Mat cMin1 = c <= a;
            cv::Mat cMin2 = c <= b;
            cv::Mat cMin;
            cv::bitwise_and(cMin1, cMin2, cMin);

            cv::Mat ret(c.size(), dtype);
            cv::add(a, b, ret, cv::noArray(), dtype);
            cv::subtract(ret, c, ret, cv::noArray(), dtype);

            cv::Mat maxValues = getMatWithMaxValues(a, b);
            maxValues.convertTo(maxValues, dtype);
            maxValues.copyTo(ret, cMin);

            cv::Mat minValues = getMatWithMinValues(a, b);
            minValues.convertTo(minValues, dtype);
            minValues.copyTo(ret, cMax);

            return ret;
        }

    public:
        cv::Mat filter(const cv::Mat &inputMat) override
        {
            return getMEDPrediction(inputMat, CV_16SC3);
        }
    };
}

#endif