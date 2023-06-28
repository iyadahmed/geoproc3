#include <algorithm>  // for std::min and max
#include <cassert>
#include <cmath>

#include "vec3.hpp"

Vec3::Vec3(float value) : x(value), y(value), z(value) {}

Vec3::Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

Vec3 Vec3::operator+(const Vec3 &other) const
{
  return Vec3(x + other.x, y + other.y, z + other.z);
}

void Vec3::operator+=(const Vec3 &other)
{
  (*this) = (*this) + other;
}

Vec3 Vec3::operator-(const Vec3 &other) const
{
  return Vec3(x - other.x, y - other.y, z - other.z);
}

Vec3 Vec3::operator*(const Vec3 &other) const
{
  return Vec3(x * other.x, y * other.y, z * other.z);
}

Vec3 Vec3::operator/(const Vec3 &other) const
{
  return Vec3(x / other.x, y / other.y, z / other.z);
}

Vec3 Vec3::operator*(float scalar) const
{
  return Vec3(x * scalar, y * scalar, z * scalar);
}

Vec3 Vec3::operator/(float scalar) const
{
  return Vec3(x / scalar, y / scalar, z / scalar);
}

Vec3 Vec3::min(const Vec3 &other) const
{
  return Vec3(std::min(x, other.x), std::min(y, other.y), std::min(z, other.z));
}

Vec3 Vec3::max(const Vec3 &other) const
{
  return Vec3(std::max(x, other.x), std::max(y, other.y), std::max(z, other.z));
}

float Vec3::dot(const Vec3 &other) const
{
  return x * other.x + y * other.y + z * other.z;
}

float Vec3::operator[](int index) const
{
  assert(index >= 0 && index < 3);
  return (&x)[index];
}

bool Vec3::is_close(const Vec3 &other, float tolerance) const
{
  return std::abs(x - other.x) < tolerance && std::abs(y - other.y) < tolerance &&
         std::abs(z - other.z) < tolerance;
}

bool Vec3::operator<=(const Vec3 &other) const
{
  return x <= other.x && y <= other.y && z <= other.z;
}

bool Vec3::operator>=(const Vec3 &other) const
{
  return x >= other.x && y >= other.y && z >= other.z;
}
