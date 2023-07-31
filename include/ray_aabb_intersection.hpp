#pragma once

#include "aabb.hpp"
#include "ray.hpp"

inline bool intersect_ray_aabb(const Ray &ray, const AABB &aabb)
{
  Vec3 t_upper = (aabb.max - ray.origin) / ray.direction;
  Vec3 t_lower = (aabb.min - ray.origin) / ray.direction;
  Vec3 t_min_v = t_upper.min(t_lower);
  Vec3 t_max_v = t_upper.max(t_lower);

  float t_min = t_min_v.max_elem();
  float t_max = t_max_v.min_elem();

  return t_max > t_min;
}
