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

class PhysicsEngine {
private:
  Debugger *debug = nullptr;
  Narrowphase sat;
  Broadphase bvh;
  ConstraintSolver solver;

  AABB a;
  Plane bounds[8];
  int bounds_size;

public:
  PhysicsEngine() {

    bounds_size = 50;
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

  void step(float dt) {
    using clock = std::chrono::high_resolution_clock;

    if (debug) {
      for (int i = 0; i < 6; i++) {
        // debug->draw_plane(bounds[i], bounds_size, glm::vec3(1, 1, 1));
        debug->draw_plane(bounds[i], bounds_size, glm::vec3(0, 0, 0));
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
      CLOGI("body[%d] pos=(%.3f, %.3f, %.3f) linear=(%.3f, %.3f, %.3f) angular=(%.3f, %.3f, %.3f)\n",
          b.id,
          b.position.x,
          b.position.y,
          b.position.z,
          b.lin_velocity.x,
          b.lin_velocity.y,
          b.lin_velocity.z,
          b.ang_velocity.x,
          b.ang_velocity.y,
          b.ang_velocity.z);
    }

    auto t1 = clock::now();

    // 2. Broadphase
    potential_collisions = bvh.update(bodies, debug);

    auto t2 = clock::now();

    // 3a. Plane collisions
    contacts.clear();
    for (auto &b : bodies) {
      for (int i = 0; i < 6; i++) {
        Narrowphase::ContactManifold manifold;
        if (sat.poly_plane_collision(manifold, b, bounds[i], debug)) {

          manifold.a = &b;
          manifold.b = nullptr;
          if (manifold.num_points > 0) {
            contacts.push_back(manifold);
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

        // WARNING:
        // OLD: if (glm::dot(manifold.normal, manifold.b->position - manifold.a->position) < 0.0f) {
        if (glm::dot(manifold.normal, manifold.b->position - manifold.a->position) > 0.0f) {
          manifold.normal = -manifold.normal;
          for (int i = 0; i < manifold.num_points; i++)
            manifold.points[i].normal = manifold.normal;
        }
        contacts.push_back(manifold);
      }
    }

    auto t4 = clock::now();

    // 4. Solver
    solver.build_constraints(contacts);
    solver.solve_constraints(dt);

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
