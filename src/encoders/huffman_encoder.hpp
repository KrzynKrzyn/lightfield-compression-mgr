#ifndef HUFFMAN_ENCODER_HPP
#define HUFFMAN_ENCODER_HPP

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
    template<class T>
    class HuffmanEncoder
    {
    private:
    public:
        Unraveler<T>* unraveler;

        std::unordered_map<T, size_t> symbolCount;
        std::unordered_map<T, std::string> huffmanCodes;

    public:
        HuffmanEncoder(Unraveler<T>* unraveler): unraveler(unraveler) {
            symbolCount = huffman::generateSymbolCount(*unraveler);

            huffman::TreeGenerator<T> treeGenerator;
            huffman::Node<T> huffmanTree = treeGenerator.generateTree(symbolCount);

            huffman::CodeGenerator<T> codeGenerator;
            huffmanCodes = codeGenerator.generateHuffmanCodes(huffmanTree);
        }

        size_t estimateEncodedSizeInBits()
        {
            size_t encodedBitsLength = 0;
            for (const auto &e : huffmanCodes)
            {
                const auto &symbol = e.first;
                const auto &huffmanCode = e.second;
                size_t numberOfBits = symbolCount[symbol] * huffmanCode.size();
                encodedBitsLength += numberOfBits;
            }

            return encodedBitsLength;
        }

        std::vector<unsigned char> encode()
        {
            std::vector<unsigned char> encodedBytes;
            std::string bufferedCodes = "";
            int i = 0;
            // std::cout << "To push: " << bytes.size() << std::endl;
            // for (const auto &e : bytes)

            unraveler->reset();
            while(!unraveler->isEndReached())
            {
                T e = unraveler->nextValue();
                ++i;
                bufferedCodes += huffmanCodes[e];

                while (bufferedCodes.size() >= 8)
                {
                    std::string encodedByteString = bufferedCodes.substr(0, 8);
                    bufferedCodes = bufferedCodes.substr(8);

                    // todo: repetition
                    std::bitset<8> encodedByteBitset(encodedByteString);
                    unsigned char encodedByte = (encodedByteBitset.to_ulong() & 0xFF);
                    encodedBytes.push_back(encodedByte);
                }

                // if (i % 1000000 == 0)
                //     std::cout << "And push! " << i << std::endl;
            }

            if (!bufferedCodes.empty())
            {
                size_t paddingSize = 8 - bufferedCodes.size();
                bufferedCodes += std::string(paddingSize, '0');

                // todo: repetition
                std::bitset<8> encodedByteBitset(bufferedCodes);
                unsigned char encodedByte = (encodedByteBitset.to_ulong() & 0xFF);
                encodedBytes.push_back(encodedByte);
            }

            return encodedBytes;
        }
    };
}

#endif