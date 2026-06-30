#include "Contacts.hpp"

void ContactCache::warm_start(std::vector<Manifold> &manifolds) {
  for (auto &m : manifolds) {
    for (size_t i = 0; i < m.num_points; i++) {
      Contact &c = m.contacts[i];
      auto it = cache.find(c.body_id);
      if (it == cache.end())
        continue;

      const CachedImpulse *match = nullptr;

      // 1. Exact topological match - velocity-independent, works for any speed.
      for (const auto &ci : it->second) {
        if (ci.vertex_id == c.vertex_id) {
          match = &ci;
          break;
        }
      }

      // 2. Proximity fallback - for quadrics or when features genuinely change.
      if (!match) {
        float best_dist_sq = MATCH_THRESHOLD_SQ;
        for (const auto &ci : it->second) {
          glm::vec3 d = c.a_pos_local - ci.a_pos_local;
          float dist_sq = glm::dot(d, d);
          if (dist_sq < best_dist_sq) {
            best_dist_sq = dist_sq;
            match = &ci;
          }
        }
      }

      if (match) {
        c.norm_impulse = match->norm_impulse;
        c.tangent_impulse[0] = match->tangent_impulse[0];
        c.tangent_impulse[1] = match->tangent_impulse[1];
      }
    }
  }
}

void ContactCache::update(const std::vector<Manifold> &manifolds) {
  cache.clear();
  for (const auto &m : manifolds) {
    for (size_t i = 0; i < m.num_points; i++) {
      const Contact &c = m.contacts[i];
      if (c.norm_impulse == 0.0f)
        continue;
      cache[c.body_id].push_back({c.vertex_id, c.a_pos_local, c.norm_impulse,
                                  {c.tangent_impulse[0], c.tangent_impulse[1]}});
    }
  }
}
