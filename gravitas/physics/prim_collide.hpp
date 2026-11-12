#pragma once

#include <cstddef>
#include <cstdint>
#include <glm/glm.hpp>

#include "Contacts.hpp"
#include "Primitive.hpp"
#include "Transform.hpp"

/*
  Pair routines between two child primitives. Arguments are ordered
  type(a) <= type(b) - Narrowphase::collide swaps the rest and flips the
  result. Output is world-space with the normal pointing B -> A.
 */

namespace col {

struct ShapeProperties {
  const Primitive *prim; // owned by the entity's Geometry
  // rigid body transform - cached matrix/inverse/normal shared across pairs.
  // child placement (local_xfrm) comes from prim, e.g.
  // Polyhedron::get_geom_mesh()
  const Transform *xfrm;
  size_t entity_id;
  size_t child_idx;
};

using PairFn = bool (*)(Manifold &, const ShapeProperties &,
                        const ShapeProperties &);

// Packs two 32-bit IDs into a single uint64 (upper | lower).
inline uint64_t pack_id(uint32_t upper, uint32_t lower) {
  return (static_cast<uint64_t>(upper) << 32) | static_cast<uint64_t>(lower);
}

// one-point manifold - normal B -> A, quadric contacts carry no topology id
inline void set_single_contact(Manifold &m, const ShapeProperties &a,
                               const ShapeProperties &b, glm::vec3 pos_world,
                               glm::vec3 normal, float depth) {
  Contact &c = m.contacts[0];
  c.pos_world = pos_world;
  c.norm = normal;
  c.pen_depth = depth;
  c.entity_id = pack_id(static_cast<uint32_t>(a.entity_id),
                        static_cast<uint32_t>(b.entity_id));
  c.vertex_id = 0; // no topology - warm start matches on a_pos_local
  c.a_pos_local =
      glm::vec3(a.xfrm->get_inverse_matrix() * glm::vec4(pos_world, 1.0f));

  m.norm = normal;
  m.num_points = 1;
}

// sphere child resolved to world-space
struct WorldSphere {
  glm::vec3 centre;
  float radius;
};

// Quadric.cpp - collision table guarantees prim is a Sphere
WorldSphere sphere_cast(const ShapeProperties &s);

// Poly.cpp
bool poly2(Manifold &m, const ShapeProperties &a, const ShapeProperties &b);
bool poly_sphere(Manifold &m, const ShapeProperties &a,
                 const ShapeProperties &b);
bool poly_frustum(Manifold &m, const ShapeProperties &a,
                  const ShapeProperties &b);

// Quadric.cpp
bool sphere2(Manifold &m, const ShapeProperties &a, const ShapeProperties &b);
bool sphere_frustum(Manifold &m, const ShapeProperties &a,
                    const ShapeProperties &b);
bool frustum2(Manifold &m, const ShapeProperties &a, const ShapeProperties &b);

} // namespace col
