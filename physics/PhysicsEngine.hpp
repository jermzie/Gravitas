#pragma once

#include <chrono>
#include <glm/glm.hpp>

#include <algorithm>

#include "AABB.hpp"
#include "Broadphase.hpp"
#include "Config.hpp"
#include "ConstraintSolver.hpp"
#include "Debugger.hpp"
#include "Entity.hpp"
#include "Model.hpp"
// #include "Narrowphase.hpp"
#include "Plane.hpp"
#include "RigidBody.hpp"

class PhysicsEngine {
private:
  Debugger *debug = nullptr;
  Narrowphase sat;
  Broadphase bvh;
  ContactCache cache;
  ConstraintSolver solver;

  // inner faces of the sandbox slabs - render only, collision goes through the
  // slab entities themselves
  std::vector<Plane> bound_faces;
  float bounds_size = 0.0f;

  int next_id = 0;
  std::unordered_map<int, int> id2idx_map;

public:
  PhysicsEngine() {

    entities.reserve(256);
    global_beta = 0.1f;
    global_restitution = 0.0f;
    global_friction = 1.0f;
  }
  void set_debugger(Debugger *debug) { this->debug = debug; }

  // sandbox walls as 6 static, invisible box slabs with inner faces at
  // +-half_extent
  void add_bounds(float half_extent = 50.0f, float thickness = 10.0f);

  float global_beta;
  float global_restitution;
  float global_friction;
  std::vector<Entity> entities;
  std::vector<std::pair<int, int>> potential_collisions;
  std::vector<Manifold> contacts;

  void step(float dt);

  // world-space closest-hit ray query - idx is the hit entity's index into
  // `entities` (not its id)
  bool raycast(const Ray &ray, size_t &idx, RayHit &hit);

  void add_entity(Entity &&entity);
  void remove_entity(int id);
  void remove_all_entities();
};
