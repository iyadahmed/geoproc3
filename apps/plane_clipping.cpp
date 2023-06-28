#include <array>
#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <stack>

#include "bvh.hpp"
#include "vec3.hpp"

constexpr int STL_BINARY_HEADER_SIZE = 80;

struct Triangle {
  std::array<Vec3, 3> vertices;

  BVH::AABB calc_aabb() const
  {
    Vec3 min = vertices[0].min(vertices[1]).min(vertices[2]);
    Vec3 max = vertices[0].max(vertices[1]).max(vertices[2]);
    return BVH::AABB{min, max};
  }
};

struct Plane {
  Vec3 normal;
  float distance;
};

enum class AABB_Plane_Intersection_Type {
  INTERSECTING,  // Some AABB corners are above the plane, some are below
  ABOVE,         // AABB is entirely above plane
  BELOW,         // AABB is entirely below plane
};

static bool is_above_plane(const Vec3 &point, const Plane &plane)
{
  return (point.dot(plane.normal) - plane.distance) > 0;
}

static AABB_Plane_Intersection_Type intersect(const BVH::AABB &aabb, const Plane &plane)
{
  int count = 0;
  for (int i = 0; i < 8; i++) {
    if (is_above_plane(aabb.get_corner(i), plane)) {
      count++;
    }
  }
  if (count == 8) {
    return AABB_Plane_Intersection_Type::ABOVE;
  }
  else if (count == 0) {
    return AABB_Plane_Intersection_Type::BELOW;
  }
  else {
    return AABB_Plane_Intersection_Type::INTERSECTING;
  }
}

static void copy_triangles(const std::vector<Triangle> &input,
                           const BVH &bvh,
                           std::vector<Triangle> &output)
{
}

static void plane_clipping(const std::vector<Triangle> &input,
                           const BVH &bvh,
                           std::vector<Triangle> &output)
{
  std::stack<const BVH::Node *> stack;
  stack.push(bvh.get_root());
  while (!stack.empty()) {
    const BVH::Node *node = stack.top();
    stack.pop();
  }
}

static size_t max_leaf_count(const BVH &bvh)
{
  size_t result = 0;
  std::stack<const BVH::Node *> stack;
  stack.push(bvh.get_root());
  while (!stack.empty()) {
    const BVH::Node *node = stack.top();
    stack.pop();
    if (node->is_leaf()) {
      result = std::max(result, node->count);
    }
    else {
      stack.push(node->left);
      stack.push(node->right);
    }
  }
  return result;
}

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cout << "Expected arguments: /path/to/mesh.stl" << std::endl;
    return 0;
  }
  const char *mesh_path = argv[1];

  std::ifstream mesh_file;
  // Raise exceptions on failure to open or read file
  mesh_file.exceptions(std::ifstream::badbit | std::ifstream::failbit);
  mesh_file.open(mesh_path, std::ios::binary);

  // Skip header
  mesh_file.seekg(STL_BINARY_HEADER_SIZE, std::ios::cur);

  uint32_t num_triangles = 0;
  mesh_file.read((char *)(&num_triangles), sizeof(uint32_t));

  std::cout << "Number of triangles reported by file: " << num_triangles << std::endl;

  std::vector<BVH::AABB> bounding_boxes;
  bounding_boxes.reserve(num_triangles);

  for (uint32_t i = 0; i < num_triangles; i++) {
    // Skip normal vector
    mesh_file.seekg(sizeof(Vec3), std::ios::cur);
    Triangle triangle = {Vec3(0.0f), Vec3(0.0f), Vec3(0.0f)};
    mesh_file.read((char *)(&triangle), sizeof(Triangle));
    // Skip attribute byte count
    mesh_file.seekg(sizeof(uint16_t), std::ios::cur);

    bounding_boxes.push_back(triangle.calc_aabb());
  }

  auto start = std::chrono::high_resolution_clock::now();
  BVH bvh(bounding_boxes);
  auto end = std::chrono::high_resolution_clock::now();
  std::cout << "BVH construction time: "
            << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << "ms"
            << std::endl;

  std::cout << max_leaf_count(bvh) << std::endl;

  bvh.validate_bounding_boxes(bounding_boxes);
  return 0;
}
