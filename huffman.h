#pragma once

#include <vector>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>

// HuffmanTree decoder for DHT section.
class HuffmanTree {
    struct Node {
        uint8_t val;
        std::shared_ptr<Node> left = nullptr;
        std::shared_ptr<Node> right = nullptr;
        bool is_term = false;
        bool is_blocked = false;
    };

public:
    // code_lengths is the array of size no more than 16 with number of
    // terminated nodes in the Huffman tree.
    // values are the values of the terminated nodes in the consecutive
    // level order.
    void Build(const std::vector<uint8_t>& code_lengths, const std::vector<uint8_t>& values);

    // Moves the state of the huffman tree by |bit|. If the node is terminated,
    // returns true and overwrites |value|. If it is intermediate, returns false
    // and value is unmodified.
    bool Move(bool bit, int& value);

private:
    void RecursiveBuild(std::shared_ptr<Node>& cur_node, uint8_t cur_depth, uint8_t depth,
                        uint8_t& id_vals, uint8_t& ised_vals,
                        const std::vector<uint8_t>& code_lengths,
                        const std::vector<uint8_t>& values);

    std::shared_ptr<Node> root_ = std::make_shared<Node>();
    std::shared_ptr<Node> cur_node_ = nullptr;
};
