#pragma once

struct Vec3 {
  float x, y, z;
  explicit Vec3(float value);
  Vec3(float x, float y, float z);
  Vec3 operator+(const Vec3 &other) const;
  Vec3 operator-(const Vec3 &other) const;
  Vec3 operator*(const Vec3 &other) const;
  Vec3 operator/(const Vec3 &other) const;
  Vec3 operator*(float scalar) const;
  Vec3 operator/(float scalar) const;
};
