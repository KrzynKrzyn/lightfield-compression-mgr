#ifndef PREDICTION_RESULTS_ENCODER_HPP
#define PREDICTION_RESULTS_ENCODER_HPP

#include "prediction_results.hpp"

#include "src/unravelers/vector_unraveler.hpp"
#include "src/encoders/arithmetic_encoder.hpp"

namespace lfc
{
    class PredictionResultsEncoder
    {
    private:
    public:
        std::vector<uint8_t> encode(PredictionResults results)
        {
            std::vector<uint8_t> ret;

            std::vector<int> serializedMVs;
            for (const auto& r : results.blockMatchResults) {
                const auto &mv = r.motionVector;
                serializedMVs.push_back(mv[0]);
                serializedMVs.push_back(mv[1]);
            }

            lfc::Unraveler<int>* mvUnraveler = new lfc::VectorUnraveler<int>(serializedMVs);
            lfc::ArithmeticEncoder<int> mvEncoder(mvUnraveler);
            auto encodedMVs = mvEncoder.encode();

            delete mvUnraveler;

            for (int i=0; i<(mvEncoder.symbolCount.size() *2 * 4); ++i) encodedMVs.push_back(0);  // TODO: true save, not placeholder

            return encodedMVs;
        }
    };
}

#endif