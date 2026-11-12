#pragma once

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/matrix.hpp>
#include <limits>
#include <memory>
#include <numeric>

#include "config.hpp"
#include "contacts.hpp"
#include "debug_draw.hpp"
#include "entity.hpp"
#include "halfedge_mesh.hpp"
#include "plane.hpp"
#include "primitive.hpp"
#include "rigid_body.hpp"
#include "transform.hpp"

#include "collide.hpp"

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
