#ifndef IMAGE_ZIKZAK_UNRAVELER_HPP
#define IMAGE_ZIKZAK_UNRAVELER_HPP

#include <opencv2/opencv.hpp>
#include "unraveler.hpp"

namespace lfc
{
    template <class T>
    class ImageZikZakUnraveler : public Unraveler<T>
    {
    private:
        const cv::Mat image;
        int index = 0;

        void nextDiagonal() {
            
        }

        void nextDiagonalValue() {

        }

        void previousDiagonalValue() {

        }

    public:
        ImageZikZakUnraveler(const cv::Mat &mat) : image(mat))
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