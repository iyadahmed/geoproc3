#pragma once

#include <array>
#include <fstream>
#include <vector>

#include "bvh.hpp"
#include "triangle.hpp"

std::vector<Triangle> read_binary_stl(const char *filepath);
void write_binary_stl(const char *filepath, const std::vector<Triangle> &triangles);
