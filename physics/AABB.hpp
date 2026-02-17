#pragma once

#include "Model.hpp"
#include "Ray.hpp"
#include "Transform.hpp"
#include <array>
#include <glm/glm.hpp>

struct AABB {

  glm::vec3 min;
  glm::vec3 max;

  AABB(std::array<float, 6U> extrema) {
    for (int i = 0; i < 6; i++) {
      (i % 2 == 0) ? min[i / 2] = extrema[i] : max[i / 2] = extrema[i];
    }
  }

  AABB(glm::vec3 min, glm::vec3 max) : min(min), max(max) {}

  AABB() = default;

  AABB transform_arvo(AABB local, glm::mat4 transform) const {
    float a, b;
    float a_min[3] = {local.min.x, local.min.y, local.min.z};
    float a_max[3] = {local.max.x, local.max.y, local.max.z};

    // translation matrix component
    glm::vec3 translation(transform[3][0], transform[3][1], transform[3][2]);
    float b_min[3] = {translation.x, translation.y, translation.z};
    float b_max[3] = {translation.x, translation.y, translation.z};

    // rotation/scale matrix components
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {

        // transform min/max corners
        a = transform[j][i] * a_min[j];
        b = transform[j][i] * a_max[j];

        if (a < b) {
          b_min[i] += a; // a is smaller, add to min
          b_max[i] += b; // b is larger, add to max
        } else {
          b_min[i] += b;
          b_max[i] += a;
        }
      }
    }

    return AABB(glm::vec3(b_min[0], b_min[1], b_min[2]), glm::vec3(b_max[0], b_max[1], b_max[2]));
  }

  // simple ray intersection test
  // https://psgraphics.blogspot.com/2016/02/new-simple-ray-box-test-from-andrew.html
  bool ray_intersect(const Ray &r, float tmin = 5.0f, float tmax = 500.0f) {

    for (int axis = 0; axis < 3; ++axis) {

      float inverse_dir = 1.0f / r.direction[axis];
      float t0 = (min[axis] - r.origin[axis]) * inverse_dir;
      float t1 = (max[axis] - r.origin[axis]) * inverse_dir;

      if (inverse_dir < 0.0f) {
        std::swap(t0, t1);
      }

      tmin = (t0 > tmin) ? t0 : tmin;
      tmax = (t1 < tmax) ? t1 : tmax;

      if (tmax <= tmin) {
        return false;
      }
    }

    return true;
  }

  // bool point_intersection(const glm::vec3 &v) {}

  // cost of AABB -- want to minimize BVH node surface area
  float surface_area() const {

    // sa of rectangular prism:
    // 2(lw + lh + wh)
    glm::vec3 d = max - min;
    return 2.0f * (d.x * d.y + d.x * d.z + d.y * d.z);
  }

  float volume() const {

    // vol of rectangular prism:
    // l * w * h
    glm::vec3 d = max - min;
    return d.x * d.y * d.z;
  }

  bool contains(const AABB &other) const {
    return (min.x <= other.min.x && min.y <= other.min.y && min.z <= other.min.z && max.x >= other.max.x &&
            max.y >= other.max.y && max.z >= other.max.z);
  }

  bool contains_point(const glm::vec3 &point) const {
    return (point.x >= min.x && point.y >= min.y && point.z >= min.z && point.x <= max.x && point.y <= max.y &&
            point.z <= max.z);
  }

  glm::vec3 center() const { return (min + max) * 0.5f; }

  glm::vec3 half_extents() const { return (max - min) * 0.5f; }
};

inline bool aabb_intersect(const AABB &a, const AABB &b) {
  return (a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y && a.max.y >= b.min.y && a.min.z <= b.max.z &&
          a.max.z >= b.min.z);
}

inline AABB combine(const AABB &a, const AABB &b) {
  return {glm::min(a.min, b.min), glm::max(a.max, b.max)};
}

inline AABB fatten(const AABB &box, float margin) {

  return {box.min - glm::vec3(margin), box.max + glm::vec3(margin)};
}
