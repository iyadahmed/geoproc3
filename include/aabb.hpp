#pragma once

#include "vec3.hpp"

struct AABB {
  Vec3 min;
  Vec3 max;

  AABB operator+(const AABB &other) const;
  void operator+=(const AABB &other);
  Vec3 get_corner(int index) const;
  Vec3 calc_center() const;
  bool is_close(const AABB &other, float tolerance) const;
  bool contains(const AABB &other) const;
};

inline AABB AABB::operator+(const AABB &other) const
{
  return AABB{min.min(other.min), max.max(other.max)};
}

inline void AABB::operator+=(const AABB &other)
{
  min = min.min(other.min);
  max = max.max(other.max);
}

inline Vec3 AABB::get_corner(int index) const
{
  assert(index >= 0 && index < 8);
  // Thanks Copilot! :)
  return Vec3{index & 1 ? max.x : min.x, index & 2 ? max.y : min.y, index & 4 ? max.z : min.z};
}

inline Vec3 AABB::calc_center() const
{
  return (min + max) / 2;
}

inline bool AABB::is_close(const AABB &other, float tolerance) const
{
  return min.is_close(other.min, tolerance) && max.is_close(other.max, tolerance);
}

inline bool AABB::contains(const AABB &other) const
{
  return other.min >= min && other.max <= max;
}
