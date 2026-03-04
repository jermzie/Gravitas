#pragma once

#include <chrono>
#include <glm/glm.hpp>

#include <algorithm>

#include "AABB.hpp"
#include "Broadphase.hpp"
#include "CollisionGeometry.hpp"
#include "Config.hpp"
#include "Debugger.hpp"
#include "Model.hpp"
#include "Plane.hpp"
#include "RigidBody.hpp"
// DOESNT WORK
// #include "BoundingSphere.hpp"
#include "ConstraintSolver.hpp"
#include "Narrowphase.hpp"
#include "PersistentManifold.hpp"

class PhysicsEngine {
private:
  Debugger *debug = nullptr;
  Narrowphase sat;
  Broadphase bvh;
  ConstraintSolver pgs;

  AABB a;
  Plane bounds[8];
  int bounds_size;

  PersistentManifold &get_or_create_manifold(RigidBody *a, RigidBody *b, Plane *plane = nullptr) {
    for (auto &pm : persistent_contacts) {
      if (pm.a == a && pm.b == b && pm.plane == plane)
        return pm;
    }
    persistent_contacts.push_back({a, b, plane});
    return persistent_contacts.back();
  }

public:
  PhysicsEngine() {

    bounds_size = 10;
    glm::vec3 axes[] = {{0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    for (int i = 0; i < 6; i++) {
      glm::vec3 normal = axes[i % 3];
      bounds[i].normal = (i < 3) ? normal : -normal;
      bounds[i].point = -bounds[i].normal * bounds_size;
      bounds[i].distance = -glm::dot(bounds[i].normal, bounds[i].point);
    }
  }
  void set_debugger(Debugger *debug) { this->debug = debug; }

  std::vector<RigidBody> bodies;
  std::vector<std::pair<int, int>> potential_collisions;
  std::vector<Narrowphase::ContactManifold> contacts;
  std::vector<PersistentManifold> persistent_contacts;

  void step(float dt) {
    using clock = std::chrono::high_resolution_clock;

    if (debug) {
      for (int i = 0; i < 6; i++) {
        // debug->draw_plane(bounds[i], bounds_size, glm::vec3(1, 1, 1));
        debug->draw_plane(bounds[i], bounds_size, glm::vec3(1, 0, 0));
      }
    }

    // Physics Loop
    // 1. Integrate forces - apply external forces & update velocities (violates constraints)
    // 2. Broadphase - query potential collisions
    // 3. Narrowphase - build contacts
    // 4. Constraint solver - correct velocities with impulses
    // 5. Integrate positions - apply bias???

    auto t0 = clock::now();

    // 1. Integrate forces
    for (auto &b : bodies) {
      if (!b.is_static) {
        b.integrate_forces(dt);
      }
    }

    for (auto &b : bodies) {
      printf("body[%d] pos=(%.3f, %.3f, %.3f) vel=(%.3f, %.3f, %.3f)\n",
          b.id,
          b.position.x,
          b.position.y,
          b.position.z,
          b.lin_velocity.x,
          b.lin_velocity.y,
          b.lin_velocity.z);
    }

    auto t1 = clock::now();

    // 2. Broadphase
    potential_collisions = bvh.update(bodies, nullptr);

    auto t2 = clock::now();

    // 3a. Plane collisions
    for (auto &pm : persistent_contacts)
      pm.cull_stale_points();

    for (auto &b : bodies) {
      for (int i = 0; i < 6; i++) {
        Narrowphase::ContactManifold manifold;
        if (sat.poly_plane_collision(manifold, b, bounds[i], debug)) {

          manifold.a = &b;
          manifold.b = nullptr;
          if (manifold.num_points > 0) {

            PersistentManifold &pm = get_or_create_manifold(&b, nullptr, &bounds[i]);
            for (int j = 0; j < manifold.num_points; j++)
              pm.merge(manifold.points[j]); // ← merge instead of push_back
          }
        }
      }
    }

    auto t3 = clock::now();

    // 3b. Narrowphase
    for (auto pair : potential_collisions) {

      Narrowphase::ContactManifold manifold;
      if (sat.poly_poly_collision(manifold, bodies[pair.first], bodies[pair.second], debug)) {

        manifold.a = &bodies[pair.first];
        manifold.b = &bodies[pair.second];

        // Fix normal direction (A -> B)
        if (glm::dot(manifold.normal, manifold.b->position - manifold.a->position) < 0.0f) {
          manifold.normal = -manifold.normal;
          for (int j = 0; j < manifold.num_points; j++)
            manifold.points[j].normal = manifold.normal;
        }

        PersistentManifold &pm = get_or_create_manifold(&bodies[pair.first], &bodies[pair.second]);
        for (int j = 0; j < manifold.num_points; j++)
          pm.merge(manifold.points[j]);
      }
    }

    // After contacts are built, before pgs.solve()
    for (auto &m : contacts) {
      for (int i = 0; i < m.num_points; i++) {
        float cached = pgs.find_cached_impulse(m.a, m.b, m.points[i].point);
        printf("cp[%d]: warm=%s  cached_lambda=%.4f  pos=(%.3f,%.3f,%.3f)\n",
            i,
            cached > 0.f ? "HIT " : "MISS",
            cached,
            m.points[i].point.x,
            m.points[i].point.y,
            m.points[i].point.z);
      }
    }

    auto t4 = clock::now();

    // 4. Solver
    pgs.solve(persistent_contacts, dt);

    auto t5 = clock::now();

    // 5. Integrate position
    for (auto &b : bodies) {
      if (!b.is_static) {
        b.update(dt);
      }
    }

    auto t6 = clock::now();

    /*
    CLOGI("potential pairs: %zu\n", potential_collisions.size());
    CLOGI("force integration: %ldus\n", std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count());
    CLOGI("broadphase:%ldus\n", std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count());
    CLOGI("plane cols: %ldus\n", std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count());
    CLOGI("narrowphase: %ldus\n", std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count());
    CLOGI("solver: %ldus\n", std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count());
    CLOGI("position integration: %ldus\n", std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5).count());
    */
  }

  bool raycast(const Ray &ray, int &id, float &t) { return bvh.raycast(bodies, ray, id, t); }

  void add_rigid_body(RigidBody &&body) {
    int id = (int)bodies.size();
    AABB aabb = body.collider.local_aabb;
    body.id = id;
    bodies.push_back(std::move(body));
    bvh.insert(aabb, id);
    CLOGI("ADDED BODY %d", id);
  }

  void remove_rigid_body(RigidBody body) {

    // bodies.erase();s
  }

  void remove_all_bodies() { bodies.clear(); }
};
