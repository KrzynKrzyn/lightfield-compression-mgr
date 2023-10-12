#ifndef IMAGE_LEFTRIGHT_CHANNELS_UNRAVELER_HPP
#define IMAGE_LEFTRIGHT_CHANNELS_UNRAVELER_HPP

#include <opencv2/opencv.hpp>
#include "unraveler.hpp"

namespace lfc
{
    template <class T>
    class ImageLeftRightChannelsUnraveler : public Unraveler<T>
    {
    private:
        const std::vector<T> unraveledImage;
        int valueIndex = 0;

        std::vector<T> matToVector(const cv::Mat &mat)
        {
            std::vector<T> ret;
            if (mat.isContinuous())
            {
                ret.assign((T *)mat.data, (T *)mat.data + mat.total() * mat.channels());
            }
            else
            {
                for (int i = 0; i < mat.rows; ++i)
                {
                    ret.insert(ret.end(), mat.ptr<T>(i), mat.ptr<T>(i) + mat.cols * mat.channels());
                }
            }

            return ret;
        }

        std::vector<T> matToVectorSeparateChannels(const cv::Mat &mat)
        {
            std::vector<cv::Mat> splitMat;
            cv::split(mat, splitMat);

            std::vector<T> ret;

            for (const auto& x : splitMat) 
            {
                auto vectorMat = matToVector(x);
                ret.insert( ret.end(), vectorMat.begin(), vectorMat.end() );
            }

            return ret;
        }

    public:
        ImageLeftRightChannelsUnraveler(const cv::Mat &mat) : unraveledImage(matToVectorSeparateChannels(mat))
        {
        }

        T nextValue() override
        {
            return unraveledImage[valueIndex++];
        }

        void reset() override
        {
            valueIndex = 0;
        }

        bool isEndReached() override {
            return valueIndex >= unraveledImage.size();
        }
    };
}

#endif