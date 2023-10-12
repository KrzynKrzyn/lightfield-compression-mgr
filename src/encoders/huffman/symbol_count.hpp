#ifndef UNRAVELER_SCANNER_SYMBOL_COUNT_HPP
#define UNRAVELER_SCANNER_SYMBOL_COUNT_HPP

#include <unordered_map>

#include "../../unravelers/unraveler.hpp"

namespace lfc::huffman
{
    template <class T>
    std::unordered_map<T, size_t> generateSymbolCount(Unraveler<T> &unraveler)
    {
        std::unordered_map<T, size_t> symbolCount;
        unraveler.reset();
        while (!unraveler.isEndReached())
        {
            ++symbolCount[unraveler.nextValue()];
        }
        return symbolCount;
    }
}

#endif