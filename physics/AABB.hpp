#pragma once

#include "../inc/Model.hpp"
#include "../inc/Ray.hpp"
#include "../inc/Transform.hpp"
#include <glm/glm.hpp>

struct AABB {

  glm::vec3 min;
  glm::vec3 max;

  /*
    void init(std::array<float, 6U> extrema) {

      for (int i = 0; i < 6; i++) {

        (i % 2 == 0) ? min[i / 2] = extrema[i] : max[i / 2] = extrema[i];
      }
    }
  */

  /*
  // simple ray intersection test
  //
  https://psgraphics.blogspot.com/2016/02/new-simple-ray-box-test-from-andrew.html
  bool ray_intersect(const Ray &r, float tmin = 5.0f, float tmax = 500.0f) {

    for (int axis = 0; axis < 3; ++axis) {

      float inverse_dir = 1.0f / r.direction[axis];
      float t0 = (a.min[axis] - r.origin[axis]) * inverse_dir;
      float t1 = (a.max[axis] - r.origin[axis]) * inverse_dir;

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
  */

  bool point_intersection(const glm::vec3 &v) {}

  // cost of AABB -- want to minimize BVH node surface area
  float surface_area() {

    // sa of rectangular prism:
    // 2(lw + lh + wh)
    glm::vec3 d = max - min;
    return 2.0f * (d.x * d.y + d.x * d.z + d.y * d.z);
  }

  float volume() {

    // vol of rectangular prism:
    // l * w * h
    glm::vec3 d = max - min;
    return d.x * d.y * d.z;
  }

  bool contains(const AABB &other) {
    return (min.x <= other.max.x && min.y >= other.min.y &&
            min.z <= other.min.z && max.x >= other.max.x &&
            max.y >= other.max.y && max.z >= other.max.z

    );
  }
};

inline bool aabb_intersect(const AABB &a, const AABB &b) {
  return (a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y &&
          a.max.y >= b.min.y && a.min.z <= b.max.z && a.max.z >= b.min.z);
}

inline AABB combine(const AABB &a, const AABB &b) {

  return {glm::min(a.min, b.min), glm::max(a.max, b.max)};
}

/*
inline AABB fatten(const AABB& box, float margin) {

    return { box - glm::vec3(margin), box + glm::vec3(margin) };
}
*/
