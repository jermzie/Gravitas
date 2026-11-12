#pragma once

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/matrix.hpp>
#include <limits>
#include <memory>
#include <numeric>

#include "gravitas/core/log.hpp"
#include "gravitas/physics/contacts.hpp"
#include "gravitas/geometry/debug_draw.hpp"
#include "gravitas/physics/entity.hpp"
#include "gravitas/geometry/halfedge_mesh.hpp"
#include "gravitas/geometry/plane.hpp"
#include "gravitas/geometry/primitive.hpp"
#include "gravitas/physics/rigid_body.hpp"
#include "gravitas/geometry/transform.hpp"

#include "gravitas/physics/collide.hpp"

/**
 * @brief Collision detection & contact generation between two entities.
 * Tests every overlapping pair of child primitives and dispatches each pair to
 * a routine in col:: (see gravitas/physics/collide.hpp).
 */
class Narrowphase {
public:
  bool collide(std::vector<Manifold> &out, const Entity &a, const Entity &b,
               DebugDraw *debug = nullptr);

private:
  // [type(a)][type(b)] - upper triangle only, collide() swaps the rest.
  // nullptr = unsupported pair (skipped)
  static const col::PairFn collide_matrix[PRIM_COUNT][PRIM_COUNT];
};
