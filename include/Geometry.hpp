#pragma once

#include "Primitive.hpp"
#include <memory>
#include <vector>

/*
Pre-defined Geometry - thin wrapper over Primitive.hpp
*/

class Geometry {
private:
  std::vector<std::shared_ptr<Primitive>> children;

public:
  void add_child(std::shared_ptr<Primitive> shape) {
    children.push_back(shape);
  }

  const std::vector<std::shared_ptr<Primitive>> &get_children() const {
    return children;
  }

  /* Example composites. Each generates render models, so a current OpenGL
   * context must exist before calling them. Base cube spans [-1, 1],
   * so cube scale factors below are half-extents. */

  // Table standing at y = 0, centered at xz-plane origin
  static Geometry make_table(float width = 2.0f, float depth = 1.2f,
                             float height = 1.0f, float top_thickness = 0.1f,
                             float leg_thickness = 0.1f) {
    Geometry table;

    auto top = std::make_shared<Polyhedron>();
    top->scale({width / 2, top_thickness / 2, depth / 2})
        .translate({0.0f, height - top_thickness / 2, 0.0f});
    table.add_child(top);

    const float leg_height = height - top_thickness;
    const float leg_x = width / 2 - leg_thickness / 2;
    const float leg_z = depth / 2 - leg_thickness / 2;
    const glm::vec2 corners[4] = {
        {-leg_x, -leg_z}, {leg_x, -leg_z}, {-leg_x, leg_z}, {leg_x, leg_z}};

    for (const glm::vec2 &c : corners) {
      auto leg = std::make_shared<Polyhedron>();
      leg->scale({leg_thickness / 2, leg_height / 2, leg_thickness / 2})
          .translate({c.x, leg_height / 2, c.y});
      table.add_child(leg);
    }

    return table;
  }

  // Dumbbell along the x-axis, centered at origin
  static Geometry make_dumbbell(float bar_length = 1.0f,
                                float bar_radius = 0.05f,
                                float weight_radius = 0.25f) {
    Geometry dumbbell;

    // TODO: swap for Cylinder once implemented
    auto bar = std::make_shared<Polyhedron>();
    bar->scale({bar_length / 2, bar_radius, bar_radius});
    dumbbell.add_child(bar);

    for (float side : {-1.0f, 1.0f}) {
      auto weight = std::make_shared<Sphere>(weight_radius);
      weight->translate(glm::vec3(side * bar_length / 2, 0.0f, 0.0f));
      dumbbell.add_child(weight);
    }

    return dumbbell;
  }

  // Flat ground slab with its top face at y = 0
  static Geometry make_plane(float width = 20.0f, float depth = 20.0f,
                             float thickness = 0.1f) {
    Geometry plane;

    auto slab = std::make_shared<Polyhedron>();
    slab->scale({width / 2, thickness / 2, depth / 2})
        .translate({0.0f, -thickness / 2, 0.0f});
    plane.add_child(slab);

    return plane;
  }
};
