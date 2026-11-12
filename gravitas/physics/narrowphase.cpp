#include "Narrowphase.hpp"
#include "AABB.hpp"
#include "Contacts.hpp"
#include "DebugDraw.hpp"
#include <utility>

using col::PairFn;
using col::ShapeProperties;

const PairFn Narrowphase::collide_matrix[PRIM_COUNT][PRIM_COUNT] = {
    // POLYHEDRON
    {col::poly2, col::poly_sphere, col::poly_frustum, nullptr},
    // SPHERE
    {nullptr, col::sphere2, col::sphere_frustum, nullptr},
    // FRUSTUM
    {nullptr, nullptr, col::frustum2, nullptr},
    // CAPSULE
    {nullptr, nullptr, nullptr, nullptr},
};

namespace {
// routine ran as (b, a) - flip its output back so the normal points B -> A
void flip_manifold(Manifold &m) {
  m.norm = -m.norm;
  for (size_t i = 0; i < m.num_points; i++) {
    Contact &c = m.contacts[i];
    c.norm = -c.norm;
    // a_pos_local stays in the routine's first body - same frame every step
    // for this pair, which is all warm start proximity needs
  }
}
} // namespace

bool Narrowphase::collide(std::vector<Manifold> &out, const Entity &a,
                          const Entity &b, DebugDraw *debug) {

  // neither entity can move - contacts are useless
  if (a.is_static && b.is_static) {
    return false;
  }

  const Transform &a_xfrm = a.body->get_dynamic_xfrm();
  const Transform &b_xfrm = b.body->get_dynamic_xfrm();
  const auto &ca = a.geom->get_children();
  const auto &cb = b.geom->get_children();

  // collision functions only work with primitive shapes
  std::vector<std::pair<ShapeProperties, ShapeProperties>> prim_collisions;
  for (size_t i = 0; i < ca.size(); i++) {
    for (size_t j = 0; j < cb.size(); j++) {
      // query colliding pairs between all child primitives

      AABB a_world = transform_arvo(ca[i]->geom_aabb(), a_xfrm.get_matrix());
      AABB b_world = transform_arvo(cb[j]->geom_aabb(), b_xfrm.get_matrix());
      if (aabb_intersect(a_world, b_world)) {
        // cheap AABB test

        ShapeProperties a_shape = {ca[i].get(), &a_xfrm, a.id, i};
        ShapeProperties b_shape = {cb[j].get(), &b_xfrm, b.id, j};
        prim_collisions.push_back({a_shape, b_shape});
      }
    }
  }

  // out might hold manifolds from earlier pairs - only draw this call's
  const size_t first_new = out.size();

  bool collides = false;
  for (const auto &[pa, pb] : prim_collisions) {
    PrimitiveType a_type = pa.prim->get_type();
    PrimitiveType b_type = pb.prim->get_type();

    // get collision function pointer
    const bool swap = a_type > b_type;
    PairFn collide_fn =
        swap ? collide_matrix[b_type][a_type] : collide_matrix[a_type][b_type];

    // invalid function pointer
    if (!collide_fn) {
      static bool warned = false;
      if (!warned) {
        CLOGW("Narrowphase: unsupported primitive pair (%d, %d) - skipping",
              a_type, b_type);
        warned = true;
      }
      continue;
    }

    Manifold m;
    bool hit = swap ? collide_fn(m, pb, pa) : collide_fn(m, pa, pb);
    if (hit) {
      if (swap) {
        // enforce normal points B -> A
        flip_manifold(m);
      }

      // warm start key - keeps child pairs of the same two entities apart
      const uint32_t child_id =
          (static_cast<uint32_t>(pa.child_idx & 0xFFFF) << 16) |
          static_cast<uint32_t>(pb.child_idx & 0xFFFF);
      for (size_t i = 0; i < m.num_points; i++) {
        m.contacts[i].child_id = child_id;
      }
      m.a = a.body.get();
      m.b = b.body.get();
      out.push_back(m);
      collides = true;
    }
  }

  if (debug) {
    // draw contact points
    for (size_t k = first_new; k < out.size(); k++) {
      for (size_t i = 0; i < out[k].num_points; i++) {
        debug->draw_vertex(out[k].contacts[i].pos_world,
                           glm::vec4(1.0f, 0.9f, 0.0f, 1.0f));
      }
    }
  }

  return collides;
}
