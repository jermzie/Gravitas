#include "Broadphase.hpp"
#include "Contacts.hpp"
#include "Narrowphase.hpp"

struct ContactCache {
  glm::vec3 pos_local;
  float norm_impulse = 0.0f;
  float tangent_impulse[2] = {};

  ContactID cid;
};

struct PersistentManifold {
  RigidBody *a = nullptr;
  RigidBody *b = nullptr;

  ContactCache point_cache[4];
  size_t num_points;
};

class myClass {

  // Match Collision body IDs to a some cached Manifold
  std::unordered_map<uint64_t, PersistentManifold> contact_cache;

  // void update(std::vector<std::pair<int, int>> collision_pairs);
  void update(std::vector<std::pair<int, int>> collision_pairs) {
    for (const auto &p : collision_pairs) {

      // 1. check if existing contact exists via contact cache map
      contact_cache_lookup(p.first, p.second);

      // 2. if exists, pass [..] to narrowphase and update cache

      // 3. doesn't exist, create new contact? and build contacts in narrowphase
    }
  }

  void contact_cache_lookup(int a, int b) {
    // create id first (order matters, so check both);
    auto itr = contact_cache.find(a);
  }
};
