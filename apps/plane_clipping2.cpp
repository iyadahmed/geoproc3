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

// Inspired by Sutherland-Hodgman algorithm
// https://en.wikipedia.org/wiki/Sutherland%E2%80%93Hodgman_algorithm
// Thanks to Vilem for pointing me to the Sutherland-Hodgman algorithm
// https://www.gamedev.net/forums/topic/698826-whats-the-real-tech-behind-the-gpu-triangles-clipping/5390202/
static void clip_triangle(const Triangle &triangle,
                          const Plane &plane,
                          std::vector<Triangle> &output)
{
  std::vector<Vec3> clipping_result;
  for (int i = 0; i < 3; i++) {
    const Vec3 &curr_point = triangle.vertices[(i + 1) % 3];
    const Vec3 &prev_point = triangle.vertices[i];

    float curr_distance = distance(curr_point, plane);
    float prev_distance = distance(prev_point, plane);

    if (prev_distance >= 0.0f) {
      clipping_result.push_back(prev_point);
    }

    // If there's an intersection between the edge and the plane, add the intersection point
    // to the clipping result.
    // Thanks to https://poe.com/GPT-4 for filling this part
    if ((curr_distance > 0.0f && prev_distance < 0.0f) ||
        (curr_distance < 0.0f && prev_distance > 0.0f))
    {
      float t = prev_distance / (prev_distance - curr_distance);
      Vec3 intersection_point = prev_point + t * (curr_point - prev_point);
      clipping_result.push_back(intersection_point);
    }
  }

  if (clipping_result.size() == 3) {
    output.emplace_back(clipping_result[0], clipping_result[1], clipping_result[2]);
  }
  else if (clipping_result.size() == 4) {
    output.emplace_back(clipping_result[0], clipping_result[1], clipping_result[2]);
    output.emplace_back(clipping_result[2], clipping_result[3], clipping_result[0]);
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
  Plane plane = {.normal = Vec3(0, 0, 1), .point = Vec3(0, 0, 50)};
  for (const Triangle &triangle : input_triangles) {
    clip_triangle(triangle, plane, output_triangles);
  }
  std::cout << "Number of triangles after clipping: " << output_triangles.size() << std::endl;
  write_binary_stl("clipped.stl", output_triangles);
  return 0;
}
