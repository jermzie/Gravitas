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

  Plane bounds[8];
  int bounds_size;

  int next_id = 0;
  std::unordered_map<int, int> id_to_index;

public:
  PhysicsEngine() {

    bodies.reserve(256);
    global_beta = 0.1f;
    global_restitution = 0.0f;
    global_friction = 1.0f;
    bounds_size = 50;
    glm::vec3 axes[] = {
        {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    for (int i = 0; i < 6; i++) {
      glm::vec3 normal = axes[i % 3];
      bounds[i].normal = (i < 3) ? normal : -normal;
      bounds[i].point = -bounds[i].normal * bounds_size;
      bounds[i].distance = -glm::dot(bounds[i].normal, bounds[i].point);
    }
  }
  void set_debugger(Debugger *debug) { this->debug = debug; }

  float global_beta;
  float global_restitution;
  float global_friction;
  std::vector<RigidBody> bodies;
  std::vector<std::pair<int, int>> potential_collisions;
  std::vector<Manifold> contacts;

  void step(float dt);
  bool raycast(const Ray &ray, int &id, float &t);

  void add_rigid_body(RigidBody &&body);
  void remove_rigid_body(int id);
  void remove_all_bodies();
};
