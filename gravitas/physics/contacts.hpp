#pragma once

#include "gravitas/physics/rigid_body.hpp"
#include <cstdint>
#include <glm/glm.hpp>
#include <unordered_map>
#include <vector>

struct Contact {
  glm::vec3 pos_world;
  glm::vec3 norm;
  float pen_depth = 0.0f;

  // contact point in the local space of the pair routine's first argument
  // (body A, or B if the pair was swapped) — warm start proximity key
  glm::vec3 a_pos_local;

  float norm_impulse = 0.0f;
  float tangent_impulse[2] = {};

  uint64_t entity_id = 0; // canonical body pair key: pack_id(A.id, B.id)
  // child pair within body_id: (child(A) << 16) | child(B)
  uint32_t child_id = 0;
  uint32_t vertex_id = 0; // topological ClipVertex ID; 0 for quadrics

  Contact() = default;
};

struct Manifold {
  RigidBody *a = nullptr;
  RigidBody *b = nullptr;

  glm::vec3 norm = glm::vec3(0.0f);
  Contact contacts[4] = {};
  size_t num_points = 0;
};

struct CachedImpulse {
  uint32_t child_id;     // only contacts from the same child pair match
  uint32_t vertex_id;    // topological ID for exact matching
  glm::vec3 a_pos_local; // position fallback for quadrics / feature changes
  float norm_impulse;
  float tangent_impulse[2];
};

// Stores accumulated contact impulses from the previous frame, keyed by
// body_id. Only contacts with the same child_id are candidates. Matching
// priority: (1) exact vertex_id, (2) proximity on a_pos_local.
class ContactCache {
  std::unordered_map<uint64_t, std::vector<CachedImpulse>> cache;

  // Proximity threshold used only when exact vertex_id match fails.
  static constexpr float MATCH_THRESHOLD_SQ = 0.04f; // (0.2 m)^2

public:
  void warm_start(std::vector<Manifold> &manifolds);
  void update(const std::vector<Manifold> &manifolds);
};
