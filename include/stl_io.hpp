#pragma once

#include <array>
#include <fstream>
#include <vector>

#include "bvh.hpp"

struct Triangle {
  std::array<Vec3, 3> vertices;

  Triangle(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2) : vertices{v0, v1, v2} {}

  BVH::AABB calc_aabb() const
  {
    Vec3 min = vertices[0].min(vertices[1]).min(vertices[2]);
    Vec3 max = vertices[0].max(vertices[1]).max(vertices[2]);
    return BVH::AABB{min, max};
  }
};

std::vector<Triangle> read_binary_stl(const char *filepath);
void write_binary_stl(const char *filepath, const std::vector<Triangle> &triangles);
