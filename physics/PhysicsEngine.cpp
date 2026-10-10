#include "PhysicsEngine.hpp"

void PhysicsEngine::step(float dt) {
  using clock = std::chrono::high_resolution_clock;

  // Physics Loop
  // 1. Integrate forces - apply external forces & update velocities (violates
  // constraints)
  // 2. Broadphase - query potential collisions
  // 3. Narrowphase - build contacts
  // 4. Constraint solver - correct velocities with impulses
  // 5. Integrate positions - apply bias???

  auto t0 = clock::now();

  if (debug) {
    for (const Plane &face : bound_faces) {
      // fill the floor only - the walls are border + grid
      const float fill_alpha = (face.normal.y > 0.5f) ? 0.1f : 0.0f;
      debug->draw_plane(face, bounds_size, glm::vec4(1, 1, 1, 1),
                        glm::vec4(1, 1, 1, fill_alpha));
    }
  }

  /*
  // 1. Integrate forces + reset pseudo velocities for all bodies
  for (auto &entity : entities) {

    // auto body = entity.body;
    auto &body = entity.body;

    if (!entity.is_static) {
      body->integrate_velocities(dt);

      // Reset here so bodies that lose contact this frame don't carry stale
      // pseudo velocity from the previous frame into integrate_positions.
      body->psuedo_lin_velocity = glm::vec3(0.0f);
      body->psuedo_ang_velocity = glm::vec3(0.0f);
    }
  }
  */

  auto t1 = clock::now();

  // 2. Broadphase

  // callback fnc
  auto get_aabb = [&](int id) {
    return entities[id2idx_map.at(id)].get_aabb();
  };
  potential_collisions = bvh.update(get_aabb, debug);

  auto t2 = clock::now();

  /*
  // 3. Narrowphase
  // TODO: once Narrowphase ported to take Entity param - map pair ids through
  id2idx_map and
  // skip static-static pairs (sandbox slabs overlap at the corners)
  contacts.clear();
  for (auto pair : potential_collisions) {

    Manifold manifold;
    if (sat.poly_poly_collision(manifold, entities[pair.first],
                                entities[pair.second], debug)) {

      manifold.a = &entities[pair.first];
      manifold.b = &entities[pair.second];

      // WARNING:
      // TODO: orient the normal (B -> A) inside Narrowphase instead - a slab's
      // COM is far off to the side, so this COM heuristic breaks for edge
      // contacts near the walls
      // OLD: if (glm::dot(manifold.normal, manifold.b->position -
      // manifold.a->position) < 0.0f) {
      if (glm::dot(manifold.norm, manifold.b->position - manifold.a->position) >
          0.0f) {
        manifold.norm = -manifold.norm;
        for (int i = 0; i < manifold.num_points; i++)
          manifold.contacts[i].norm = manifold.norm;
      }
      contacts.push_back(manifold);
    }
  }

  auto t4 = clock::now();

  // 4. Solver
  cache.warm_start(contacts);
  solver.build_constraints(contacts);
  solver.solve_constraints(dt);
  solver.write_back(contacts);
  cache.update(contacts);

  auto t5 = clock::now();

  // 5. Integrate position
  for (auto &entity : entities) {
    auto &body = entity.body;
    if (!entity.is_static) {
      body->integrate_positions(dt);
    }
  }
  */

  auto t6 = clock::now();
}

bool PhysicsEngine::raycast(const Ray &ray, size_t &idx, RayHit &hit) {

  // callback fnc
  auto on_leaf = [&](int id, float t_max) {
    const Entity &entity = entities[id2idx_map.at(id)];
    // static bodies (sandbox slabs) aren't pickable - let rays pass through
    if (entity.is_static) {
      return RayHit{};
    }
    return entity.raycast(ray, t_max);
  };

  int id;
  if (!bvh.raycast(ray, on_leaf, id, hit)) {
    return false;
  }
  idx = id2idx_map.at(id);
  return true;
}

void PhysicsEngine::add_entity(Entity &&entity) {
  int id = next_id++;
  int idx = entities.size();
  id2idx_map[id] = idx;

  entity.id = id;
  entities.push_back(std::move(entity));
  bvh.insert(entities.back().get_aabb(), id);

  CLOGI("ADDED ENTITY - ID: %d", id);
}

// Swap and pop to avoid shifting elements
void PhysicsEngine::remove_entity(int id) {
  int idx = id2idx_map[id];
  int last_idx = entities.size() - 1;

  if (idx != last_idx) {

    std::swap(entities[idx], entities[last_idx]); // Swap with last body
    id2idx_map[entities[idx].id] = idx; // Update swapped body's index in map
  }

  entities.pop_back();
  id2idx_map.erase(id);
  bvh.remove(id);
}

void PhysicsEngine::remove_all_entities() { entities.clear(); }

// Slabs are thick so SAT can't resolve a deep penetration out through the far
// face (thin slabs tunnel once penetration > thickness / 2), but not so thick
// that vertex precision suffers. They sit outside the interior and overlap at
// the edges/corners so there are no seams.
void PhysicsEngine::add_bounds(float half_extent, float thickness) {
  bounds_size = half_extent;
  bound_faces.clear();

  const glm::vec3 axes[] = {
      {0.0f, 1.0f, 0.0f}, {1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
  for (int i = 0; i < 6; i++) {
    // points into the sandbox
    const glm::vec3 normal = (i < 3) ? axes[i % 3] : -axes[i % 3];
    const glm::vec3 along = glm::abs(normal);
    const glm::vec3 half_extents =
        along * (0.5f * thickness) + (1.0f - along) * (half_extent + thickness);

    // unit cube spans [-1, 1]
    auto slab = std::make_shared<Polyhedron>();
    slab->scale(half_extents);
    auto geom = std::make_unique<Geometry>();
    geom->add_child(slab);

    // static - density only feeds mass properties that make_static discards
    glm::vec3 com_offset;
    MassProperties props = geom->finalize(1.0f, com_offset);
    const glm::vec3 centre = -normal * (half_extent + 0.5f * thickness);

    Entity wall(std::make_unique<RigidBody>(props, 1.0f, centre),
                std::move(geom));
    wall.make_static();
    wall.is_visible = false;
    add_entity(std::move(wall));

    bound_faces.emplace_back(normal, -normal * half_extent);
  }
}
