#pragma once

#include <cstddef>  // for size_t
#include <vector>

#include "aabb.hpp"
#include "vec3.hpp"

struct BVH {
 public:
  struct Node {
    AABB aabb;
    Node *left, *right;
    // Indices of primitives
    size_t start, count;

    bool is_leaf() const
    {
      return left == nullptr && right == nullptr;
    }
  };

 private:
  Node *root = nullptr;
  Node *preallocated_nodes = nullptr;
  size_t number_of_preallocated_nodes = 0;
  size_t number_of_allocated_nodes = 0;
  std::vector<size_t> indices;
  size_t count_leaf_primitives() const;
  Node *create_node(const std::vector<AABB> &bounding_boxes, size_t start, size_t count);
  // This method is intended to be used with the same bounding boxes that were used to build the
  // BVH
  bool are_bounding_boxes_valid(const std::vector<AABB> &bounding_boxes) const;

 public:
  explicit BVH(const std::vector<AABB> &bounding_boxes);
  ~BVH();
  const Node *get_root() const;
  // Get correct primitive index from an index stored in a BVH node
  size_t get_primitive_index(size_t index) const;
};
