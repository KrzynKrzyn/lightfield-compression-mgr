#ifndef VECTOR_UNRAVELER_HPP
#define VECTOR_UNRAVELER_HPP

#include <opencv2/opencv.hpp>
#include "unraveler.hpp"

namespace lfc
{
    template <class T>
    class VectorUnraveler : public Unraveler<T>
    {
    private:
        const std::vector<T> vector;
        int valueIndex = 0;

    public:
        VectorUnraveler(const std::vector<T> &vector) : vector(vector)
        {
        }

        T nextValue() override
        {
            return vector[valueIndex++];
        }

        void reset() override
        {
            valueIndex = 0;
        }

        bool isEndReached() override {
            return valueIndex >= vector.size();
        }
    };
}

#endif