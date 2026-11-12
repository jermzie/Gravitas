#pragma once

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/matrix.hpp>
#include <limits>
#include <memory>
#include <numeric>

#include "Log.hpp"
#include "Contacts.hpp"
#include "DebugDraw.hpp"
#include "Entity.hpp"
#include "HalfEdgeMesh.hpp"
#include "Plane.hpp"
#include "Primitive.hpp"
#include "RigidBody.hpp"
#include "Transform.hpp"

#include "Shapes.hpp"

/**
 * @brief Collision detection & contact generation between two entities.
 * Tests every overlapping pair of child primitives and dispatches each pair to
 * a routine in narrowphase:: (see narrowphase/Shapes.hpp).
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
