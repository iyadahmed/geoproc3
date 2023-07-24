#include <chrono>
#include <iostream>
#include <stack>

#include "bvh.hpp"
#include "stl_io.hpp"

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cerr << "Expected arguments: /path/to/mesh.stl" << std::endl;
    return 1;
  }
  std::vector<Triangle> triangles = read_binary_stl(argv[1]);
  std::vector<BVH::AABB> bounding_boxes;
  bounding_boxes.reserve(triangles.size());
  for (const Triangle &t : triangles)
    bounding_boxes.push_back(t.calc_aabb());

  auto t1 = std::chrono::high_resolution_clock::now();
  BVH bvh(bounding_boxes);
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "BVH construction time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << "ms"
            << std::endl;

  // Traverse BVH
  size_t n = 0;
  std::stack<const BVH::Node *> stack;
  stack.push(bvh.get_root());
  while (!stack.empty()) {
    const BVH::Node *node = stack.top();
    stack.pop();
    if (node == nullptr)
      continue;
    else if (node->is_leaf())
      n++;
    else {
      stack.push(node->left);
      stack.push(node->right);
    }
  }
  std::cout << n << std::endl;
  return 0;
}
