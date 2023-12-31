#pragma once

#include <array>

#include "aabb.hpp"
#include "vec3.hpp"

struct Triangle {
  std::array<Vec3, 3> vertices;

  Triangle(const Vec3 &v0, const Vec3 &v1, const Vec3 &v2) : vertices{v0, v1, v2} {}

  AABB calc_aabb() const
  {
    Vec3 min = vertices[0].min(vertices[1]).min(vertices[2]);
    Vec3 max = vertices[0].max(vertices[1]).max(vertices[2]);
    return AABB{min, max};
  }
};
