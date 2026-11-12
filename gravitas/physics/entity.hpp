#pragma once

#include "geometry.hpp"
#include "primitive.hpp"
#include "rigid_body.hpp"
#include <memory>

/*
 Atomic unit in physics engine
 */

class Entity {
public:
  std::unique_ptr<RigidBody> body; // physics object
  std::unique_ptr<Geometry> geom;  // topology

  size_t id;
  glm::vec4 color;
  bool is_static = false;
  bool is_visible = true; // hint for the renderer to skip this entity
  bool is_dragging = false;
  bool is_rotating = false;

  Entity() = default;
  Entity(std::unique_ptr<RigidBody> body, std::unique_ptr<Geometry> geom)
      : body(std::move(body)), geom(std::move(geom)) {};

  // world-space -> geometry-space ray query - convert using RigidBody xfrm
  RayHit raycast(const Ray &world, float t_max) const;
  void make_static();
  void drag(glm::vec3 delta);
  void rotate(const glm::vec3 &x_axis, const glm::vec3 &y_axis, float x_offset,
              float y_offset);

  AABB get_aabb() const {
    return transform_arvo(geom->get_aabb(),
                          body->get_dynamic_xfrm().get_matrix());
  }
};

inline Entity make_entity(std::unique_ptr<Geometry> geom, float density,
                          glm::vec3 pos,
                          glm::quat rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
  glm::vec3 com_offset;
  MassProperties props = geom->finalize(density, com_offset);

  return Entity(
      std::make_unique<RigidBody>(props, density, pos + rot * com_offset, rot),
      std::move(geom));
}

inline Entity make_entity(std::unique_ptr<Primitive> prim, float density,
                          glm::vec3 pos,
                          glm::quat rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
  // create parent object
  auto geom = std::make_unique<Geometry>();
  geom->add_child(std::move(prim));

  glm::vec3 com_offset;
  MassProperties props = geom->finalize(density, com_offset);
  return Entity(
      std::make_unique<RigidBody>(props, density, pos + rot * com_offset, rot),
      std::move(geom));
}
