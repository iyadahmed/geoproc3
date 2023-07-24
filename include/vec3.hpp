#pragma once

#include <algorithm>  // for std::min and max
#include <cassert>
#include <cmath>

struct Vec3 {
  float x;
  float y;
  float z;

  explicit Vec3(float value) : x(value), y(value), z(value) {}

  Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

  Vec3 operator+(const Vec3 &other) const
  {
    return Vec3(x + other.x, y + other.y, z + other.z);
  }

  void operator+=(const Vec3 &other)
  {
    (*this) = (*this) + other;
  }

  Vec3 operator-(const Vec3 &other) const
  {
    return Vec3(x - other.x, y - other.y, z - other.z);
  }

  Vec3 operator*(const Vec3 &other) const
  {
    return Vec3(x * other.x, y * other.y, z * other.z);
  }

  Vec3 operator/(const Vec3 &other) const
  {
    return Vec3(x / other.x, y / other.y, z / other.z);
  }

  Vec3 operator*(float scalar) const
  {
    return Vec3(x * scalar, y * scalar, z * scalar);
  }

  Vec3 operator/(float scalar) const
  {
    return Vec3(x / scalar, y / scalar, z / scalar);
  }

  Vec3 min(const Vec3 &other) const
  {
    return Vec3(std::min(x, other.x), std::min(y, other.y), std::min(z, other.z));
  }

  Vec3 max(const Vec3 &other) const
  {
    return Vec3(std::max(x, other.x), std::max(y, other.y), std::max(z, other.z));
  }

  float dot(const Vec3 &other) const
  {
    return x * other.x + y * other.y + z * other.z;
  }

  float operator[](int index) const
  {
    assert(index >= 0 && index < 3);
    return (&x)[index];
  }

  bool is_close(const Vec3 &other, float tolerance) const
  {
    return std::abs(x - other.x) < tolerance && std::abs(y - other.y) < tolerance &&
           std::abs(z - other.z) < tolerance;
  }

  bool operator<=(const Vec3 &other) const
  {
    return x <= other.x && y <= other.y && z <= other.z;
  }

  bool operator>=(const Vec3 &other) const
  {
    return x >= other.x && y >= other.y && z >= other.z;
  }

  float length() const
  {
    return std::sqrt(x * x + y * y + z * z);
  }

  Vec3 normalized() const
  {
    return (*this) / length();
  }

  friend Vec3 operator*(float scalar, const Vec3 &vector)
  {
    return vector * scalar;
  }
};
