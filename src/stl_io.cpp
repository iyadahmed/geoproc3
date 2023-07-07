#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "stl_io.hpp"

constexpr int STL_BINARY_HEADER_SIZE = 80;

std::vector<Triangle> read_binary_stl(const char *filepath)
{

  std::ifstream file;
  // Raise exceptions on failure to open or read file
  file.exceptions(std::ifstream::badbit | std::ifstream::failbit);
  file.open(filepath, std::ios::binary);

  // Skip header
  file.seekg(STL_BINARY_HEADER_SIZE, std::ios::cur);

  uint32_t num_triangles = 0;
  file.read((char *)(&num_triangles), sizeof(uint32_t));

  std::cout << "Number of triangles reported by file: " << num_triangles << std::endl;
  std::vector<Triangle> triangles;
  triangles.reserve(num_triangles);

  for (uint32_t i = 0; i < num_triangles; i++) {
    // Skip normal vector
    file.seekg(sizeof(Vec3), std::ios::cur);

    Triangle triangle = {Vec3(0.0f), Vec3(0.0f), Vec3(0.0f)};
    file.read((char *)(&triangle), sizeof(Triangle));

    // Skip attribute byte count
    file.seekg(sizeof(uint16_t), std::ios::cur);

    triangles.push_back(triangle);
  }
  return triangles;
}

void write_binary_stl(const char *filepath, const std::vector<Triangle> &triangles)
{
  if (triangles.size() > std::numeric_limits<uint32_t>::max()) {
    throw std::invalid_argument("Number of triangles exceeds maximum allowed by STL format");
  }

  std::ofstream file;
  file.exceptions(std::ofstream::badbit | std::ofstream::failbit);
  file.open(filepath, std::ios::binary);

  // Write header
  std::array<char, STL_BINARY_HEADER_SIZE> STL_BINARY_HEADER = {0};
  file.write(STL_BINARY_HEADER.data(), STL_BINARY_HEADER.size());

  // a final safety check
  assert(triangles.size() <= std::numeric_limits<uint32_t>::max());

  auto num_output_triangles = static_cast<uint32_t>(triangles.size());
  file.write((char *)(&num_output_triangles), sizeof(uint32_t));

  for (const Triangle &triangle : triangles) {
    Vec3 normal(0.0f);
    file.write((char *)(&normal), sizeof(Vec3));

    file.write((const char *)(&triangle), sizeof(Triangle));

    uint16_t attribute_byte_count = 0;
    file.write((char *)(&attribute_byte_count), sizeof(uint16_t));
  }
}
