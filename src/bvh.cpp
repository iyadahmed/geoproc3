#include <cassert>
#include <cstdlib>
#include <stack>

#include "bvh.hpp"

BVH::AABB BVH::AABB::operator+(const AABB &other) const
{
  return AABB{min.min(other.min), max.max(other.max)};
}

void BVH::AABB::operator+=(const AABB &other)
{
  min = min.min(other.min);
  max = max.max(other.max);
}

Vec3 BVH::AABB::get_corner(int index) const
{
  assert(index >= 0 && index < 8);
  // Thanks Copilot! :)
  return Vec3{index & 1 ? max.x : min.x, index & 2 ? max.y : min.y, index & 4 ? max.z : min.z};
}

Vec3 BVH::AABB::calc_center() const
{
  return (min + max) / 2;
}

bool BVH::AABB::is_close(const AABB &other, float tolerance) const
{
  return min.is_close(other.min, tolerance) && max.is_close(other.max, tolerance);
}

bool BVH::AABB::contains(const AABB &other) const
{
  return other.min >= min && other.max <= max;
}

BVH::Node *BVH::create_node(const std::vector<AABB> &bounding_boxes, size_t start, size_t count)
{
  AABB aabb = bounding_boxes[indices[start]];
  for (size_t i = start + 1; i < start + count; i++) {
    aabb += bounding_boxes[indices[i]];
  }
  Node *node = preallocated_nodes + number_of_allocated_nodes;
  node->aabb = aabb;
  node->left = nullptr;
  node->right = nullptr;
  node->start = start;
  node->count = count;
  number_of_allocated_nodes += 1;
  return node;
}

size_t BVH::count_leaf_primitives() const
{
  size_t count = 0;
  std::stack<Node *> stack;
  stack.push(root);
  while (!stack.empty()) {
    Node *node = stack.top();
    stack.pop();
    if (node->is_leaf()) {
      count += node->count;
    }
    else {
      stack.push(node->left);
      stack.push(node->right);
    }
  }
  return count;
}

static void calculate_split_axis_and_position(const std::vector<Vec3> &bounding_boxes_centers,
                                              const std::vector<size_t> &indices,
                                              size_t start,
                                              size_t count,
                                              int *split_axis,
                                              float *split_position)
{

  Vec3 mean(0.0f);
  Vec3 mean_of_squares(0.0f);
  float count_reciprocal = 1.0f / (float)count;
  for (size_t i = start; i < start + count; i++) {
    Vec3 centroid = bounding_boxes_centers[indices[i]];
    mean += centroid * count_reciprocal;
    mean_of_squares += centroid * centroid * count_reciprocal;
  }
  Vec3 variance = mean_of_squares - mean * mean;
  *split_axis = 0;
  if (variance.y > variance.x) {
    *split_axis = 1;
  }
  if (variance.z > variance[*split_axis]) {
    *split_axis = 2;
  }
  *split_position = mean[*split_axis];
}

static size_t partition_indices(const std::vector<Vec3> &bounding_boxes_centers,
                                std::vector<size_t> &indices,
                                size_t start,
                                size_t count,
                                int split_axis,
                                float split_position)
{
#define predicate(i) (bounding_boxes_centers[indices[i]][split_axis] < split_position)

  size_t first = start;
  size_t last = start + count - 1;
  for (; first <= last; first++) {
    if (!predicate(first)) {
      break;
    }
  }
  if (first == last) {
    return first;
  }
  for (size_t i = first; i <= last; i++) {
    if (predicate(i)) {
      std::swap(indices[first], indices[i]);
      first++;
    }
  }
#undef predicate
  return first;
}

BVH::BVH(const std::vector<AABB> &bounding_boxes)
{
  indices.reserve(bounding_boxes.size());
  for (size_t i = 0; i < bounding_boxes.size(); i++) {
    indices.push_back(i);
  }

  // Pre-allocate uninitialized memory for nodes
  // actual needed count is 2 * bounding_boxes.size() - 1
  // but we allocate one more to avoid checking for underflow
  preallocated_nodes = (Node *)malloc(sizeof(Node) * (2 * bounding_boxes.size()));

  root = create_node(bounding_boxes, 0, bounding_boxes.size());

  std::vector<Vec3> bounding_boxes_centers;
  bounding_boxes_centers.reserve(bounding_boxes.size());
  for (const AABB &aabb : bounding_boxes) {
    bounding_boxes_centers.push_back(aabb.calc_center());
  }

  std::stack<Node *> stack;
  stack.push(root);

  while (!stack.empty()) {
    Node *node = stack.top();
    stack.pop();

    if (node->count <= 2) {
      continue;
    }

    size_t start = node->start;
    size_t count = node->count;

    int split_axis;
    float split_pos;
    calculate_split_axis_and_position(
        bounding_boxes_centers, indices, start, count, &split_axis, &split_pos);
    size_t first = partition_indices(
        bounding_boxes_centers, indices, start, count, split_axis, split_pos);

    if (first == start || first == start + count) {
      continue;
    }

    node->left = create_node(bounding_boxes, start, first - start);
    node->right = create_node(bounding_boxes, first, count - first + start);
    stack.push(node->left);
    stack.push(node->right);
  }

  assert(count_leaf_primitives() == bounding_boxes.size());
  assert(are_bounding_boxes_valid(bounding_boxes));
}

BVH::~BVH()
{
  free(preallocated_nodes);
}

const BVH::Node *BVH::get_root() const
{
  return root;
}

bool BVH::are_bounding_boxes_valid(const std::vector<AABB> &bounding_boxes) const
{
  std::stack<const Node *> stack;
  stack.push(root);
  while (!stack.empty()) {
    const Node *node = stack.top();
    stack.pop();
    if (node->is_leaf()) {
      AABB aabb = bounding_boxes[indices[node->start]];
      for (size_t i = node->start + 1; i < node->start + node->count; i++) {
        aabb += bounding_boxes[indices[i]];
      }
      if (!aabb.is_close(node->aabb, 0.0001f)) {
        return false;
      }
    }
    else {
      if (!node->aabb.is_close(node->left->aabb + node->right->aabb, 0.0001f)) {
        return false;
      }
      stack.push(node->left);
      stack.push(node->right);
    }
  }

  return true;
}

size_t BVH::get_primitive_index(size_t index) const
{
  return indices[index];
}
