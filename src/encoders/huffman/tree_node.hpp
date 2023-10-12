#ifndef HUFFMAN_TREE_NODE_HPP
#define HUFFMAN_TREE_NODE_HPP

namespace lfc::huffman
{
    template <class T>
    struct Node
    {
        T value;
        double probability;
        std::vector<Node<T>> children;

        Node(T value, double probability) : value(value), probability(probability) {}
        Node(std::vector<Node<T>> children, double probability) : children(children), probability(probability) {}
    };

    class NodeProbabilityComparator
    {
    public:
        template <class T>
        bool operator()(Node<T> n1, Node<T> n2)
        {
            return n1.probability > n2.probability;
        }
    };
}

#endif