#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

#include "stl_io.hpp"
#include "vec3.hpp"

// The polygon clipping algorithm down here is inspired by Sutherland-Hodgman algorithm:
// https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
// Thanks to Vilem for pointing to the Sutherland-Hodgman algorithm in this post:
// https://www.gamedev.net/forums/topic/698826-whats-the-real-tech-behind-the-gpu-triangles-clipping/5390202/

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
    Vec3 curr_point = polygon[(i + 1) % polygon.size()];
    Vec3 prev_point = polygon[i];

    float curr_distance = distance(curr_point, plane);
    float prev_distance = distance(prev_point, plane);

    // If previous point is on or above plane, include it in result
    if (prev_distance >= 0.0f) {
      result.push_back(prev_point);
      // If current point is also on or above plane, skip iteration,
      // since in next iteration it becomes the previous point, which we include if it was on or
      // above plane, so this check could be cached if we want to, it is a simple comparison
      // anyways
      // NOTE: this is a nice check to do to skip intersection code
      if (curr_distance >= 0.0f) {
        continue;
      }
    }

    // Ensure consistent intersection results for same edges that are in different triangles and
    // are traversed in the opposite direction, without this there will be tiny differences between
    // the floating point results.
    if (curr_distance < prev_distance) {
      std::swap(curr_distance, prev_distance);
      std::swap(curr_point, prev_point);
    }

    // Otherwise,
    // if there's an intersection between the edge and the plane, add the intersection point
    // to the clipping result
    // Thanks to https://poe.com/GPT-4 for filling this part
    // TODO: double check this maybe?
    // It seems to be working so far anyways
    if ((curr_distance > 0.0f && prev_distance < 0.0f) ||
        (curr_distance < 0.0f && prev_distance > 0.0f))
    {
      float t = prev_distance / (prev_distance - curr_distance);
      Vec3 intersection_point = prev_point + t * (curr_point - prev_point);
      result.push_back(intersection_point);
    }
  }
  return result;
}

static std::vector<Vec3> clip_polygon(const std::vector<Vec3> &polygon,
                                      const std::vector<Plane> &convex_set)
{
  std::vector<Vec3> result = polygon;
  for (const Plane &plane : convex_set) {
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

static void clip_triangle(const Triangle &triangle,
                          const std::vector<Plane> &planes,
                          std::vector<Triangle> &output)
{
  auto clipped_polygon = clip_polygon(as_polygon(triangle), planes);
  if (clipped_polygon.size() >= 3) {
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
    clip_triangle(triangle, planes, output_triangles);
  }
  std::cout << "Number of triangles after clipping: " << output_triangles.size() << std::endl;
  write_binary_stl("clipped.stl", output_triangles);
  return 0;
}
