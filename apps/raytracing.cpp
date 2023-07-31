#include <chrono>
#include <iostream>
#include <vector>

#include "stl_io.hpp"

int main(int argc, char **argv)
{
  if (argc != 2) {
    std::cerr << "Expected arguments: /path/to/mesh.stl" << std::endl;
    return 1;
  }
  std::vector<Triangle> input_triangles = read_binary_stl(argv[1]);
  std::vector<Triangle> output_triangles;
  output_triangles.reserve(input_triangles.size());
  std::vector<AABB> bounding_boxes;
  bounding_boxes.reserve(input_triangles.size());
  for (const Triangle &t : input_triangles) {
    bounding_boxes.push_back(t.calc_aabb());
  }
  auto t1 = std::chrono::high_resolution_clock::now();
  BVH bvh(bounding_boxes);
  auto t2 = std::chrono::high_resolution_clock::now();
  std::cout << "BVH construction took "
            << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count()
            << " milliseconds" << std::endl;
}
