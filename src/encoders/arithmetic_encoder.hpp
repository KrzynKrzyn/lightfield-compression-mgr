#ifndef ARITHMETIC_ENCODER_HPP
#define ARITHMETIC_ENCODER_HPP

// #include <unordered_map>
// #include <queue>
#include <string>
#include <bitset>
#include <iostream>

#include "huffman/tree_node.hpp"
#include "huffman/symbol_count.hpp"
#include "huffman/tree_generator.hpp"
#include "huffman/code_generator.hpp"

namespace lfc
{
    class SymbolProbability
    {
        public:
        const uint32_t low, high;

        SymbolProbability(uint32_t low, uint32_t high) : low(low), high(high) {}
    };

    template <class T>
    class ArithmeticEncoder
    {
    public:
        Unraveler<T> *unraveler;

        std::unordered_map<T, size_t> symbolCount;
        std::unordered_map<T, int> symbolToIndex;
        std::vector<SymbolProbability> symbolProbabilities;
        size_t cumulativeCount;

        void generateSymbolProbabilities()
        {
            cumulativeCount = 0;
            for (const auto &sc : symbolCount)
            {
                uint32_t low = cumulativeCount;
                uint32_t high = cumulativeCount += sc.second;

                symbolToIndex[sc.first] = symbolProbabilities.size();
                symbolProbabilities.push_back({low, high});
            }
        }

    public:
        ArithmeticEncoder(Unraveler<T> *unraveler) : unraveler(unraveler)
        {
            symbolCount = huffman::generateSymbolCount(*unraveler);
            generateSymbolProbabilities();
        }

        // std::vector<unsigned char> encodedBytes;
        // std::string bufferedCodes = "";
        // void put_bit_plus_pending(bool bit, int &pending_bits)
        // {
        //     bufferedCodes += bit ? "1" : "0";
        //     for(int i=0; i<pending_bits; ++i) bufferedCodes += bit ? "0" : "1";
        //     // bufferedCodes += std::string(bit ? "0" : "1", pending_bits);
        //     pending_bits = 0;

        //     while (bufferedCodes.size() >= 8)
        //     {
        //         std::string encodedByteString = bufferedCodes.substr(0, 8);
        //         bufferedCodes = bufferedCodes.substr(8);

        //         std::bitset<8> encodedByteBitset(encodedByteString);
        //         unsigned char encodedByte = (encodedByteBitset.to_ulong() & 0xFF);
        //         encodedBytes.push_back(encodedByte);
        //     }
        // }
        
        std::vector<unsigned char> encodedBytes;
        char bufferedByte = 0;
        int bufferedBitsCount = 0;
        const int bitsInByte = 8;

        void reset_bit_counting() {
            bufferedByte = 0;
            bufferedBitsCount = 0;
        }

        void put_bit(bool bit) {
            if(bit) bufferedByte++;
            bufferedByte <<= 1;
            bufferedBitsCount++;

            if (bufferedBitsCount == bitsInByte) {
                encodedBytes.push_back(bufferedByte);
                reset_bit_counting();
            }
        }

        void put_bit_plus_pending(bool bit, int& pendingBits) {
            put_bit(bit);
            for(int i=0; i<pendingBits; ++i) put_bit(!bit);
            pendingBits = 0;
        }

        const uint32_t ONE_FOURTH = 0x40000000;
        const uint32_t ONE_HALF = 0x80000000;
        const uint32_t THREE_FOURTHS = 0xC0000000;
        const uint32_t MAX_CODE = 0xFFFFFFFF;

        std::vector<unsigned char> encode()
        {
            encodedBytes.clear();
            reset_bit_counting();
            // bufferedCodes = "";

            int pending_bits = 0;
            uint64_t low = 0;
            uint64_t high = MAX_CODE;

            unraveler->reset();
            while (!unraveler->isEndReached())
            {
                T e = unraveler->nextValue();

                SymbolProbability p = symbolProbabilities[symbolToIndex[e]];

                // std::cout << e << ": " << p.low << " " << p.high << " " << cumulativeCount << std::endl;
                // std::cout << bufferedBitsCount << " " << encodedBytes.size() << std::endl;

                uint64_t range = high - low + 1;
                high = low + (range * p.high / cumulativeCount) - 1;
                low = low + (range * p.low / cumulativeCount);
                for (;;)
                {
                    if (high < ONE_HALF)
                        put_bit_plus_pending(0, pending_bits);
                    else if (low >= ONE_HALF)
                        put_bit_plus_pending(1, pending_bits);
                    else if (low >= ONE_FOURTH && high < THREE_FOURTHS)
                    {
                        pending_bits++;
                        low -= ONE_FOURTH;
                        high -= ONE_FOURTH;
                    }
                    else
                        break;
                    high <<= 1;
                    high++;
                    low <<= 1;
                    high &= MAX_CODE;
                    low &= MAX_CODE;
                }
            }

            pending_bits++;
            if (low < ONE_FOURTH)
                put_bit_plus_pending(0, pending_bits);
            else
                put_bit_plus_pending(1, pending_bits);

            return encodedBytes;
        }
    };
}

#endif