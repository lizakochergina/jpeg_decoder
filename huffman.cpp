#include "huffman.h"

void HuffmanTree::Build(const std::vector<uint8_t> &code_lengths,
                        const std::vector<uint8_t> &values) {
    if (code_lengths.size() > 16) {
        throw std::invalid_argument("size of vector of code lenghts is too big");
    }

    uint8_t cnt_of_vals = 0;
    for (auto cnt : code_lengths) {
        cnt_of_vals += cnt;
    }
    if (cnt_of_vals != values.size()) {
        throw std::invalid_argument("incorrect size of vector of values");
    }

    root_->left.reset();
    root_->right.reset();
    root_->is_term = false;
    root_->is_blocked = false;

    uint8_t id_vals = 0;
    for (int depth = 0; depth != code_lengths.size(); ++depth) {
        uint8_t used_vals = 0;
        RecursiveBuild(root_, 0, depth + 1, id_vals, used_vals, code_lengths, values);
        if (used_vals != code_lengths[depth]) {
            throw std::invalid_argument("wrong arguments");
        }
    }

    cur_node_ = root_;
}

bool HuffmanTree::Move(bool bit, int &value) {
    if (!cur_node_) {
        throw std::invalid_argument("no more branches");
    }

    if (bit) {
        if (!cur_node_->right) {
            throw std::invalid_argument("no more branches");
        } else {
            cur_node_ = cur_node_->right;
            if (cur_node_->is_term) {
                value = cur_node_->val;
                cur_node_ = root_;
                return true;
            } else {
                return false;
            }
        }
    } else {
        if (!cur_node_->left) {
            throw std::invalid_argument("no more branches");
        } else {
            cur_node_ = cur_node_->left;
            if (cur_node_->is_term) {
                value = cur_node_->val;
                cur_node_ = root_;
                return true;
            } else {
                return false;
            }
        }
    }
}
void HuffmanTree::RecursiveBuild(std::shared_ptr<Node> &cur_node, uint8_t cur_depth, uint8_t depth,
                                 uint8_t &id_vals, uint8_t &used_vals,
                                 const std::vector<uint8_t> &code_lengths,
                                 const std::vector<uint8_t> &values) {

    if (id_vals == values.size() || used_vals >= code_lengths[depth - 1]) {
        return;
    }

    if (cur_depth == depth && !cur_node->is_term) {
        cur_node->val = values[id_vals];
        cur_node->is_term = true;
        cur_node->is_blocked = true;
        ++id_vals;
        ++used_vals;
        return;
    }

    if (cur_depth < depth) {
        if (!cur_node->left) {
            cur_node->left = std::make_shared<Node>();
            RecursiveBuild(cur_node->left, cur_depth + 1, depth, id_vals, used_vals, code_lengths,
                           values);
        } else if (!cur_node->left->is_term && !cur_node->left->is_blocked) {
            RecursiveBuild(cur_node->left, cur_depth + 1, depth, id_vals, used_vals, code_lengths,
                           values);
        }

        if (!cur_node->right) {
            cur_node->right = std::make_shared<Node>();
            RecursiveBuild(cur_node->right, cur_depth + 1, depth, id_vals, used_vals, code_lengths,
                           values);
        } else if (!cur_node->right->is_term && !cur_node->right->is_blocked) {
            RecursiveBuild(cur_node->right, cur_depth + 1, depth, id_vals, used_vals, code_lengths,
                           values);
        }

        cur_node->is_blocked =
            (cur_node->left && (cur_node->left->is_term || cur_node->left->is_blocked)) &&
            (cur_node->right && (cur_node->right->is_term || cur_node->right->is_blocked));
    }
}
