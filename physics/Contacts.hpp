#pragma once

#include "RigidBody.hpp"
#include <cstdint>
#include <glm/glm.hpp>

struct ContactID {
  uint64_t body_id;    // High 32 bits: Body A ID | Low 32 bits: Body B ID
  uint64_t feature_id; // High 32 bits: Feature A ID | Low 32 bits: Feature B ID

  // Required for hash map comparisons
  bool operator==(const ContactID &other) const {
    return body_id == other.body_id && feature_id == other.feature_id;
  }
};

// Use ContactID as a key for cache
template <> struct hash<ContactID> {
  size_t operator()(const ContactID &cid) const {
    // A simple hash combining both 64-bit integers
    size_t h1 = std::hash<uint64_t>{}(cid.body_id);
    size_t h2 = std::hash<uint64_t>{}(cid.feature_id);
    return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
  }
};

struct Contact {
  glm::vec3 pos_world; // world-space contact point
  glm::vec3 pos_local;
  glm::vec3 norm;
  float pen_depth; // penetration depth

  glm::vec3 a_pos_world;
  glm::vec3 b_pos_world;
  glm::vec3 a_pos_local;
  glm::vec3 b_pos_local;

  // Forms orthonormal basis
  glm::vec3 normal;
  glm::vec3 tangent[2];

  float norm_impulse;
  float tangent_impulse[2];

  ContactID cid;

  Contact(void) {
    norm_impulse = 0.0f;
    tangent_impulse[0] = 0.0f;
    tangent_impulse[1] = 0.0f;
  }
};

struct Manifold {
  RigidBody *a = nullptr;
  RigidBody *b = nullptr;

  glm::vec3 norm = glm::vec3(0.0f); // from A -> B
  Contact contacts[4] = {};
  size_t num_points = 0;

  float max_pen_depth = 0;
};

class Contacts {

  // Match Collision body IDs to a some cached Manifold
  std::unordered_map<size_t, Manifold> contact_cache;

  void update(std::vector<std::pair<int, int>> collision_pairs) {
    for (const auto &p : collision_pairs) {

      // 1. check if existing contact exists via contact cache map
      contact_cache_lookup(p.first, p.second);

      // 2. if exists, pass [..] to narrowphase and update cache

      // 3. doesn't exist, create new contact? and build contacts in narrowphase
    }
  }

  //
  void contact_cache_lookup(int a, int b) {
    // create id first (order matters, so check both);
    auto itr = contact_cache.find(a);
  }

  /*
#define THRES_SQ 1

  void contact_valid(const Manifold &m) {

    RigidBody *a = m.a;
    RigidBody *b = m.b;

    for (const Contact &c : m.contacts) {
      glm::vec3 a_local_to_world = a->transform.local_to_world(c.a_pos_local);
      glm::vec3 b_local_to_world = b->transform.local_to_world(c.b_pos_local);

      glm::vec3 r_ab = a_local_to_world - b_local_to_world;
      glm::vec3 r_a = c.a_pos_world - a_local_to_world;
      glm::vec3 r_b = c.b_pos_world - b_local_to_world;

      const bool is_penetrating = glm::dot(c.norm, r_ab) <= 0.0f;

      const bool ra_valid = glm::length2(r_a) < THRES_SQ;
      const bool rb_valid = glm::length2(r_b) < THRES_SQ;
    }
  }
  */
};
