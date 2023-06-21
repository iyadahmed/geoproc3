#include "vec3.hpp"

Vec3::Vec3(float value) : x(value), y(value), z(value) {}

Vec3::Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

Vec3 Vec3::operator+(const Vec3 &other) const
{
  return Vec3(x + other.x, y + other.y, z + other.z);
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
