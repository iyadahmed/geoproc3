#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "stl_io.hpp"
#include "vec3.hpp"

struct Plane {
  Vec3 normal;
  Vec3 point;
};

struct Ray {
  Vec3 origin;
  Vec3 direction;
};

static float distance(const Vec3 &point, const Plane &plane)
{
  return (point - plane.point).dot(plane.normal);
}

static std::vector<Vec3> clip_polygon(const std::vector<Vec3> &polygon, const Plane &plane)
{
  std::vector<Vec3> result;
  for (size_t i = 0; i < polygon.size(); i++) {
    const Vec3 &curr_point = polygon[(i + 1) % polygon.size()];
    const Vec3 &prev_point = polygon[i];

    float curr_distance = distance(curr_point, plane);
    float prev_distance = distance(prev_point, plane);

    if (prev_distance > -0.00001f) {
      result.push_back(prev_point);
    }

    // If there's an intersection between the edge and the plane, add the intersection point
    // to the clipping result.
    // Thanks to https://poe.com/GPT-4 for filling this part
    if ((curr_distance > -0.00001f && prev_distance < -0.00001f) ||
        (curr_distance < -0.00001f && prev_distance > -0.00001f))
    {
      float t = prev_distance / (prev_distance - curr_distance);
      Vec3 intersection_point = prev_point + t * (curr_point - prev_point);
      result.push_back(intersection_point);
    }
  }
  return result;
}

static std::vector<Vec3> clip_polygon(const std::vector<Vec3> &polygon,
                                      const std::vector<Plane> &convex_hull)
{
  // TODO: check that the convex hull is actually convex
  std::vector<Vec3> result = polygon;
  for (const Plane &plane : convex_hull) {
    result = clip_polygon(result, plane);
  }
  return result;
}

static void triangulate_fan(const std::vector<Vec3> &convex_polygon, std::vector<Triangle> &output)
{
  // TODO: assert that the polygon is actually convex
  assert(convex_polygon.size() >= 3);
  for (size_t i = 1; i <= (convex_polygon.size() - 2); i++) {
    output.emplace_back(convex_polygon[0], convex_polygon[i], convex_polygon[i + 1]);
  }
}

static std::vector<Vec3> as_polygon([[maybe_unused]] const Triangle &triangle)
{
  return {triangle.vertices[0], triangle.vertices[1], triangle.vertices[2]};
}

// Inspired by Sutherland-Hodgman algorithm
// https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
// Thanks to Vilem for pointing me to the Sutherland-Hodgman algorithm
// https://www.gamedev.net/forums/topic/698826-whats-the-real-tech-behind-the-gpu-triangles-clipping/5390202/
[[maybe_unused]] static void clip_triangle(const Triangle &triangle,
                                           const Plane &plane,
                                           std::vector<Triangle> &output)
{
  auto clipped_polygon = clip_polygon(as_polygon(triangle), plane);
  if (!clipped_polygon.empty()) {
    triangulate_fan(clipped_polygon, output);
  }
}

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cerr << "Expected arguments: /path/to/mesh.stl" << std::endl;
    return 1;
  }
  std::vector<Triangle> input_triangles = read_binary_stl(argv[1]);
  std::vector<Triangle> output_triangles;
  output_triangles.reserve(input_triangles.size());

  std::vector<Plane> planes = {{.normal = Vec3(0, 0, 1), .point = Vec3(0, 0, 50)},
                               {.normal = Vec3(1, 0, 0), .point = Vec3(0, 0, 0)}};

  for (const Triangle &triangle : input_triangles) {
    auto clipped_polygon = clip_polygon(as_polygon(triangle), planes);
    if (clipped_polygon.empty()) {
      continue;
    }
    triangulate_fan(clipped_polygon, output_triangles);
  }
  std::cout << "Number of triangles after clipping: " << output_triangles.size() << std::endl;
  write_binary_stl("clipped.stl", output_triangles);
  return 0;
}
