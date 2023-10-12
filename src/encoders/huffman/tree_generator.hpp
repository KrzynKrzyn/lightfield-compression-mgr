#ifndef HUFFMAN_TREE_GENERATOR_HPP
#define HUFFMAN_TREE_GENERATOR_HPP

#include "tree_node.hpp"

namespace lfc::huffman
{
    template <class T>
    class TreeGenerator
    {
    public:
        Node<T> generateTree(const std::unordered_map<T, size_t> &symbolCount)
        {
            std::priority_queue<Node<T>, std::vector<Node<T>>, NodeProbabilityComparator> nodeQueue; // todo comparator

            for (const auto &e : symbolCount)
            {
                double probability = static_cast<double>(e.second) / static_cast<double>(1); // todo kna
                nodeQueue.push(Node<T>(e.first, probability));
            }

            while (nodeQueue.size() > 1)
            {
                auto e1 = nodeQueue.top();
                nodeQueue.pop();
                auto e2 = nodeQueue.top();
                nodeQueue.pop();

                Node<T> newNode({e1, e2}, e1.probability + e2.probability);
                nodeQueue.push(newNode);
            }

            return nodeQueue.top();
        }
    };
}

#endif