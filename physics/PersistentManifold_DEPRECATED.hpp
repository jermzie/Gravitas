#pragma once
#include "Narrowphase.hpp"
#include "RigidBody.hpp"

struct PersistentManifold {
  RigidBody *a, *b;
  Plane *plane = nullptr;
  static constexpr int MAX_POINTS = 4;
  static constexpr float MATCH_DIST_SQ = 0.04f * 0.04f;

  Narrowphase::ContactPoint points[MAX_POINTS];
  float accumulated_lambda[MAX_POINTS] = {};
  int num_points = 0;

  // Local-space contact positions stored at insertion time.
  // These are INVARIANT to body motion — correct to compare against.
  glm::vec3 local_a[MAX_POINTS]; // contact point in body A's local frame
  glm::vec3 local_b[MAX_POINTS]; // contact point in body B's local frame

  void merge(const Narrowphase::ContactPoint &new_cp) {
    // Transform new contact point into A's local frame once
    glm::vec3 new_local_a = glm::inverse(a->get_physics_matrix()) * glm::vec4(new_cp.point, 1.0f);

    for (int i = 0; i < num_points; i++) {
      // Compare against STORED local coords — not re-derived from world pos
      glm::vec3 diff = new_local_a - local_a[i];
      if (glm::dot(diff, diff) < MATCH_DIST_SQ) {
        // Match — update geometry, keep accumulated_lambda
        points[i] = new_cp;
        local_a[i] = new_local_a;
        local_b[i] =
            b ? glm::vec3(glm::inverse(b->get_physics_matrix()) * glm::vec4(new_cp.point, 1.0f)) : glm::vec3(0.f);
        return;
      }
    }

    // No match — insert new point
    if (num_points < MAX_POINTS) {
      int idx = num_points++;
      points[idx] = new_cp;
      accumulated_lambda[idx] = 0.f;
      local_a[idx] = new_local_a;
      local_b[idx] =
          b ? glm::vec3(glm::inverse(b->get_physics_matrix()) * glm::vec4(new_cp.point, 1.0f)) : glm::vec3(0.f);
    } else {
      // Evict shallowest penetration
      int evict = 0;
      for (int i = 1; i < MAX_POINTS; i++)
        if (points[i].penetration < points[evict].penetration)
          evict = i;
      points[evict] = new_cp;
      accumulated_lambda[evict] = 0.f;
      local_a[evict] = new_local_a;
      local_b[evict] =
          b ? glm::vec3(glm::inverse(b->get_physics_matrix()) * glm::vec4(new_cp.point, 1.0f)) : glm::vec3(0.f);
    }
  }

  void cull_stale_points() {
    for (int i = num_points - 1; i >= 0; i--) {
      glm::vec3 world_a_pos = glm::vec3(a->get_physics_matrix() * glm::vec4(local_a[i], 1.0f));

      float separation;
      if (b) {
        // Body-body: separation is gap between the two contact points
        glm::vec3 world_b_pos = glm::vec3(b->get_physics_matrix() * glm::vec4(local_b[i], 1.0f));
        separation = glm::dot(points[i].normal, world_a_pos - world_b_pos);
      } else {
        // Plane contact: use signed distance from point to plane
        // plane->distance = -dot(normal, plane_point), so:
        // dot(normal, world_a_pos) + plane->distance > 0 means outside/separated
        if (plane) {
          separation = glm::dot(points[i].normal, world_a_pos) + plane->distance;
        } else {
          separation = 0.f; // fallback — don't cull
        }
      }

      if (separation > 0.02f) {
        int last = --num_points;
        points[i] = points[last];
        accumulated_lambda[i] = accumulated_lambda[last];
        local_a[i] = local_a[last];
        local_b[i] = local_b[last];
      }
    }
  }
};
