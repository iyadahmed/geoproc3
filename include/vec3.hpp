#pragma once

struct Vec3 {
  float x, y, z;
  explicit Vec3(float value);
  Vec3(float x, float y, float z);
  Vec3 operator+(const Vec3 &other) const;
  void operator+=(const Vec3 &other);
  Vec3 operator-(const Vec3 &other) const;
  Vec3 operator*(const Vec3 &other) const;
  Vec3 operator/(const Vec3 &other) const;
  Vec3 operator*(float scalar) const;
  Vec3 operator/(float scalar) const;
  Vec3 min(const Vec3 &other) const;
  Vec3 max(const Vec3 &other) const;
  float dot(const Vec3 &other) const;
  float operator[](int index) const;
  bool is_close(const Vec3 &other, float tolerance) const;
  bool operator<=(const Vec3 &other) const;
  bool operator>=(const Vec3 &other) const;
};
