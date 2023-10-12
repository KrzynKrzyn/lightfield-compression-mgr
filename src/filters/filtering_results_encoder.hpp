#ifndef FILTERING_RESULTS_ENCODER_HPP
#define FILTERING_RESULTS_ENCODER_HPP

#include "src/unravelers/vector_unraveler.hpp"
#include "src/encoders/arithmetic_encoder.hpp"

namespace lfc
{
    class FilteringResultsEncoder
    {
    private:
    public:
        std::vector<uint8_t> encode(std::vector<int> pickedFilters)
        {
            std::vector<uint8_t> ret;

            lfc::Unraveler<int>* resultsUnraveler = new lfc::VectorUnraveler<int>(pickedFilters);
            lfc::ArithmeticEncoder<int> encoder(resultsUnraveler);
            auto encodedResults = encoder.encode();

            delete resultsUnraveler;

            for (int i=0; i<(encoder.symbolCount.size() *2 * 4); ++i) encodedResults.push_back(0);

            return encodedResults;
        }
    };
}

#endif