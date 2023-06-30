#pragma once

#include <cstddef>  // for size_t
#include <vector>

#include "vec3.hpp"

struct BVH {
 public:
  struct AABB {
    Vec3 min, max;

    AABB operator+(const AABB &other) const;
    void operator+=(const AABB &other);
    Vec3 get_corner(int index) const;
    Vec3 calc_center() const;
    bool is_close(const AABB &other, float tolerance) const;
    bool contains(const AABB &other) const;
  };

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
  std::vector<size_t> indices;
  size_t count_leaf_primitives() const;
  Node *create_node(const std::vector<AABB> &bounding_boxes, size_t start, size_t count);

 public:
  explicit BVH(const std::vector<AABB> &bounding_boxes);
  const Node *get_root() const;
  void validate_bounding_boxes(const std::vector<AABB> &bounding_boxes) const;
};