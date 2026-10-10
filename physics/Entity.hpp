#pragma once

#include "Debugger.hpp"
#include "Geometry.hpp"
#include "RigidBody.hpp"
#include <memory>

/*
 Atomic unit in physics engine
 */

class Entity {
public:
  std::unique_ptr<RigidBody> body; // physics object
  std::unique_ptr<Geometry> geom;  // topology

  int id;
  glm::vec3 color; // temp
  bool is_static = false;
  bool is_visible = true; // collides either way - only skips Entity::draw
  bool is_dragging = false;
  bool is_rotating = false;

  Entity() = default;
  Entity(std::unique_ptr<RigidBody> body, std::unique_ptr<Geometry> geom)
      : body(std::move(body)), geom(std::move(geom)) {};

  // draws each child at (parent * child local_xfrm), uploaded to `uniform` -
  // for a gWVP-style shader, pass projection * view * body matrix as parent
  void draw(Shader &shader, const glm::mat4 &parent,
            const std::string &uniform = "model", Debugger *debug = nullptr);

  // world-space -> geometry-space ray query - convert using RigidBody xfrm
  RayHit raycast(const Ray &world, float t_max) const;
  void make_static();
  void drag(glm::vec3 delta);
  void rotate(const glm::vec3 &x_axis, const glm::vec3 &y_axis, float x_offset,
              float y_offset);

  AABB get_aabb() const;
};
