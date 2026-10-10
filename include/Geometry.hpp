#pragma once

#include "AABB.hpp"
#include "Primitive.hpp"
#include <memory>
#include <vector>

/*
Pre-defined Geometry - thin wrapper over Primitive.hpp
*/

class Geometry {
private:
  std::vector<std::shared_ptr<Primitive>> children;
  AABB aabb;

public:
  void add_child(std::shared_ptr<Primitive> shape) {
    // combine child & geometry AABBs in geometry-space
    // *first child replaces default box
    aabb = children.empty() ? shape->geom_aabb()
                            : combine(aabb, shape->geom_aabb());
    children.push_back(std::move(shape));
  }

  AABB get_aabb() const { return aabb; }

  const std::vector<std::shared_ptr<Primitive>> &get_children() const {
    return children;
  }

  // geometry-space ray query against children
  RayHit raycast(const Ray &r, float t_max) const {
    RayHit best{};
    for (auto &c : children) {

      // ray-primitive test (local-space)
      RayHit query = c->geom_raycast(r, t_max);
      if (query.is_hit) {
        best = query;
        t_max = query.hit_dist;
      }
    }
    // return closest child
    return best;
  }

  // each child computes mass properties in geometry-space - COM and inertia
  // relative to placement within parent frame
  //
  // *overlapping children double-count the shared volume
  MassProperties compute_mass_properties(float density) const {
    std::vector<MassProperties> parts;
    parts.reserve(children.size());
    for (const auto &child : children) {

      parts.push_back(child->compute_mass_properties(density));
    }
    return combine_mass_properties(parts);
  }

  /*
  shift children so union COM sits at the geometric origin -
  rigid body's position is its COM.
  */
  MassProperties finalize(float density, glm::vec3 &com_offset) {
    MassProperties props = compute_mass_properties(density);
    com_offset = props.centre_of_mass;

    // uniform shift leaves inertia (already centered about COM) unchanged
    for (auto &child : children) {
      child->translate(-com_offset);
    }
    aabb = AABB(aabb.min - com_offset, aabb.max - com_offset);
    props.centre_of_mass = glm::vec3(0.0f);
    return props;
  }

  // combine parts via the parallel axis theorem:
  //   I = sum(I_i + m_i * ((d.d) * 1 - d d^T)),  d = com_i - com
  static MassProperties
  combine_mass_properties(const std::vector<MassProperties> &parts) {
    float mass = 0.0f;
    glm::vec3 com(0.0f);
    for (const MassProperties &p : parts) {
      mass += p.mass;
      com += p.mass * p.centre_of_mass;
    }

    if (parts.empty() || mass <= 0.0f) {
      CLOGW("Geometry has no mass - returning zeroed mass properties");
      return MassProperties{.mass = 0.0f,
                            .inv_mass = 0.0f,
                            .centre_of_mass = glm::vec3(0.0f),
                            .inertia_tensor = glm::mat3(0.0f),
                            .inv_inertia_tensor = glm::mat3(0.0f)};
    }
    com /= mass;

    glm::mat3 inertia(0.0f);
    for (const MassProperties &p : parts) {
      glm::vec3 d = p.centre_of_mass - com;
      inertia += p.inertia_tensor + p.mass * (glm::dot(d, d) * glm::mat3(1.0f) -
                                              glm::outerProduct(d, d));
    }

    return MassProperties{.mass = mass,
                          .inv_mass = 1.0f / mass,
                          .centre_of_mass = com,
                          .inertia_tensor = inertia,
                          .inv_inertia_tensor = glm::inverse(inertia)};
  }

  // Table standing at y = 0, centered at xz-plane origin
  static std::unique_ptr<Geometry>
  make_table(float width = 2.0f, float depth = 1.2f, float height = 1.0f,
             float top_thickness = 0.1f, float leg_thickness = 0.1f) {
    auto table = std::make_unique<Geometry>();

    auto top = std::make_shared<Polyhedron>();
    top->scale({width / 2, top_thickness / 2, depth / 2})
        .translate({0.0f, height - top_thickness / 2, 0.0f});
    table->add_child(top);

    const float leg_height = height - top_thickness;
    const float leg_x = width / 2 - leg_thickness / 2;
    const float leg_z = depth / 2 - leg_thickness / 2;
    const glm::vec2 corners[4] = {
        {-leg_x, -leg_z}, {leg_x, -leg_z}, {-leg_x, leg_z}, {leg_x, leg_z}};

    for (const glm::vec2 &c : corners) {
      auto leg = std::make_shared<Polyhedron>();
      leg->scale({leg_thickness / 2, leg_height / 2, leg_thickness / 2})
          .translate({c.x, leg_height / 2, c.y});
      table->add_child(leg);
    }

    return table;
  }

  // Dumbbell along the x-axis, centered at origin
  static std::unique_ptr<Geometry> make_dumbbell(float bar_length = 1.0f,
                                                 float bar_radius = 0.05f,
                                                 float weight_radius = 0.25f) {
    auto dumbbell = std::make_unique<Geometry>();

    // TODO: swap for Cylinder once implemented
    auto bar = std::make_shared<Polyhedron>();
    bar->scale({bar_length / 2, bar_radius, bar_radius});
    dumbbell->add_child(bar);

    for (float side : {-1.0f, 1.0f}) {
      auto weight = std::make_shared<Sphere>(weight_radius);
      weight->translate(glm::vec3(side * bar_length / 2, 0.0f, 0.0f));
      dumbbell->add_child(weight);
    }

    return dumbbell;
  }

  // Flat ground slab with its top face at y = 0
  static std::unique_ptr<Geometry>
  make_plane(float width = 20.0f, float depth = 20.0f, float thickness = 0.1f) {
    auto plane = std::make_unique<Geometry>();

    auto slab = std::make_shared<Polyhedron>();
    slab->scale({width / 2, thickness / 2, depth / 2})
        .translate({0.0f, -thickness / 2, 0.0f});
    plane->add_child(slab);

    return plane;
  }
};
