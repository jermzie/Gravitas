#include "Broadphase.hpp"
#include "Narrowphase.hpp"

struct ContactCache {
  ContactID id;
  float normal_impulse_accum;
  float tangent_impulse_accum[2];
};

struct ManifoldCache {
  ContactCache contacts[4];
  int num_points;
};

class myClass {

  // Match Collision body IDs to a some cached Manifold
  std::unordered_map<uint64_t, ManifoldCache> contact_cache;

  // void update(std::vector<std::pair<int, int>> collision_pairs);
  void update(std::vector<std::pair<int, int>> collision_pairs) {
    for (const auto &p : collision_pairs) {

      // 1. check if existing contact exists via contact cache map
      contact_cache_lookup(p.first, p.second);

      // 2. if exists

      // 3. doesn't exist
    }
  }

  void contact_cache_lookup(int a, int b) {
    // create id first (order matters, so check both);
    auto itr = contact_cache.find(a);
  }
};
