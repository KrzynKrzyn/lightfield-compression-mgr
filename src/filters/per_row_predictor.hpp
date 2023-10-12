#ifndef PER_ROW_PREDICTOR_HPP
#define PER_ROW_PREDICTOR_HPP

#include <opencv2/opencv.hpp>
#include "shift_vector_filter.hpp"

#include "filtering_results_encoder.hpp"

namespace lfc
{
    class PerRowPredictor : public Filter
    {
    private:
        cv::Mat getAveragePrediction(const cv::Mat &up, const cv::Mat &left, const cv::Mat &upleft, int dtype = -1) const
        {
            cv::Mat ret;
            cv::add(left, up, ret, cv::noArray(), dtype);
            ret /= 2;

            return ret;
        }

        cv::Mat getJpegls4Prediction(const cv::Mat &up, const cv::Mat &left, const cv::Mat &upleft, int dtype = -1) const
        {
            cv::Mat ret;
            cv::add(up, left, ret, cv::noArray(), dtype);
            cv::subtract(ret, upleft, ret, cv::noArray(), dtype);

            return ret;
        }

        cv::Mat getJpegls5Prediction(const cv::Mat &up, const cv::Mat &left, const cv::Mat &upleft, int dtype = -1) const
        {
            cv::Mat ret;
            cv::subtract(up, upleft, ret, cv::noArray(), dtype);
            ret /= 2;
            cv::add(left, ret, ret, cv::noArray(), dtype);

            return ret;
        }

        cv::Mat getJpegls6Prediction(const cv::Mat &up, const cv::Mat &left, const cv::Mat &upleft, int dtype = -1) const
        {
            cv::Mat ret;
            cv::subtract(left, upleft, ret, cv::noArray(), dtype);
            ret /= 2;
            cv::add(up, ret, ret, cv::noArray(), dtype);

            return ret;
        }

        std::vector<cv::Mat> preparePossiblePredictions(const cv::Mat &image, int dtype = -1) const
        {
            cv::Mat predictNone(image.size(), image.type());
            predictNone.setTo(cv::Scalar(0, 0, 0));

            cv::Mat predictUp = lfc::ShiftVectorFilter({0, 1}).filter(image);
            cv::Mat predictLeft = lfc::ShiftVectorFilter({1, 0}).filter(image);
            cv::Mat predictUpLeft = lfc::ShiftVectorFilter({1, 1}).filter(image);

            // lfc::PaethPredictor paethPredictor;
            // cv::Mat predictPaeth = paethPredictor.predict(image);

            // cv::Mat predictAverage = getAveragePrediction(predictUp, predictLeft, predictUpLeft, dtype);
            // cv::Mat predictJpeg4 = getJpegls4Prediction(predictUp, predictLeft, predictUpLeft, dtype);
            // cv::Mat predictJpeg5 = getJpegls5Prediction(predictUp, predictLeft, predictUpLeft, dtype);
            // cv::Mat predictJpeg6 = getJpegls6Prediction(predictUp, predictLeft, predictUpLeft, dtype);

            // return {predictNone, predictUp, predictLeft, predictUpLeft, predictAverage, predictJpeg4, predictJpeg5, predictJpeg6};
            return {predictNone, predictUp, predictLeft, predictUpLeft};
        }

        int calculateSumOfAbsoluteError(const cv::Mat &mat1, const cv::Mat &mat2) const
        {
            cv::Mat tempMat;
            cv::absdiff(mat1, mat2, tempMat);

            auto sumOfErrorsPerChannel = cv::sum(tempMat);
            auto sumOfErrors = cv::sum(sumOfErrorsPerChannel)[0];

            return (int)sumOfErrors;
        }

        std::tuple<cv::Mat, std::vector<int>> getPerRowPrediction(const cv::Mat &image, int dtype = -1) const
        {
            std::vector<cv::Mat> possiblePredictions = preparePossiblePredictions(image, dtype);
            std::vector<int> pickedPredictions;
            pickedPredictions.reserve(image.rows);

            cv::Mat finalPrediction(image.size(), dtype);

            for (int i = 0; i < image.rows; ++i)
            {
                cv::Mat currentRow = image.row(i);

                int minSAE = std::numeric_limits<int>::max();
                int pickedPredictionNumber = -1;
                for (int j = 0; j < possiblePredictions.size(); ++j)
                {
                    cv::Mat predictedRow = possiblePredictions[j].row(i);
                    auto sae = calculateSumOfAbsoluteError(currentRow, predictedRow);

                    if (sae < minSAE)
                    {
                        minSAE = sae;
                        pickedPredictionNumber = j;
                    }
                }

                pickedPredictions.push_back(pickedPredictionNumber);

                cv::Mat pickedPrediction = possiblePredictions[pickedPredictionNumber].row(i);
                pickedPrediction.copyTo(finalPrediction.row(i));
            }

            return {finalPrediction, pickedPredictions};
        }

    public:
        std::tuple<cv::Mat, std::vector<int>> predict(const cv::Mat &inputMat)
        {
            return getPerRowPrediction(inputMat, CV_16SC3);
        }

        cv::Mat filter(const cv::Mat &inputMat) override
        {
            auto filteringResults = predict(inputMat);
            // std::cout << "F size: " << lfc::FilteringResultsEncoder().encode(std::get<std::vector<int>>(filteringResults)).size() << std::endl;
            return std::get<cv::Mat>(filteringResults);
        }
    };
}

#endif