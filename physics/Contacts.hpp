#pragma once

#include "RigidBody.hpp"
#include <glm/glm.hpp>

// For Contact Caching & Warm Starting
//   body_id:    upper 32 bits = A id,        lower 32 bits = B id
//   feature_id:
union ContactID {
  struct ContactFeature {
    uint8_t idx_A;
    uint8_t idx_B;
    uint8_t type_A;
    uint8_t type_B;
  } feature_id;
  uint64_t body_id;
};

struct ContactPoint {
  glm::vec3 pos; // world-space contact point
  glm::vec3 norm;
  float pen_depth; // penetration depth

  float norm_impulse = 0.0f;
  float tangent_impulse[2] = {};

  ContactID cid;
};

struct Manifold {
  RigidBody *a = nullptr;
  RigidBody *b = nullptr;

  ContactPoint points[4] = {};
  size_t num_points = 0;
  float max_pen_depth = 0;
  glm::vec3 norm = glm::vec3(0.0f); // contact normal from A -> B
};
