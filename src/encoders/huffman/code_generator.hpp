#ifndef HUFFMAN_CODE_GENERATOR_HPP
#define HUFFMAN_CODE_GENERATOR_HPP

#include "tree_node.hpp"

namespace lfc::huffman
{
    template <class T>
    class CodeGenerator
    {
    private:
        void _fillCodes(const Node<T> &node, const std::string &code, std::unordered_map<T, std::string> &huffmanCodes)
        {
            if (node.children.empty())
            {
                huffmanCodes[node.value] = code;
                return;
            }

            _fillCodes(node.children[0], code + "0", huffmanCodes);
            _fillCodes(node.children[1], code + "1", huffmanCodes);
        }

    public:
        std::unordered_map<T, std::string> generateHuffmanCodes(const Node<T> &root)
        {
            if (root.children.empty())
                return {{root.value, "0"}};

            std::unordered_map<T, std::string> huffmanCodes;

            _fillCodes(root.children[0], "0", huffmanCodes);
            _fillCodes(root.children[1], "1", huffmanCodes);

            return huffmanCodes;
        }
    };
}

#endif