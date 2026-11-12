#include "constraint_solver.hpp"

void ConstraintSolver::build_constraints(
    const std::vector<Manifold> &contacts) {

  size_t total_points = 0;
  for (auto &manifold : contacts) {
    total_points += manifold.num_points;
  }

  // reserve up front - FrictionConstraint::contact_normal points into
  // contact_constraints, so it must never reallocate after this
  contact_constraints.clear();
  friction_constraints.clear();
  contact_constraints.reserve(total_points);
  friction_constraints.reserve(total_points);

  // Build all contact constraints first so initial_closing_velocity
  // is sampled from pre-warmstart velocities for every contact.
  for (auto &m : contacts) {
    for (size_t i = 0; i < m.num_points; i++) {
      contact_constraints.emplace_back(m.a, m.b, m.contacts[i]);
    }
  }

  // Warm start contacts in a separate pass to avoid corrupting
  // initial_closing_velocity for later contacts in the same manifold.
  size_t ci = 0;
  for (auto &m : contacts) {
    for (size_t i = 0; i < m.num_points; i++) {
      auto &cc = contact_constraints[ci++];
      cc.impulse_accum[0] = m.contacts[i].norm_impulse;
      cc.apply_impulse_mag(0, cc.impulse_accum[0]);
    }
  }

  // Build and warm-start friction constraints after contact warmstart so
  // the tangent basis reflects velocities already close to convergence.
  ci = 0;
  for (auto &m : contacts) {
    for (size_t i = 0; i < m.num_points; i++) {
      const auto &cp = m.contacts[i];
      glm::vec3 ra = cp.pos_world - m.a->get_centre_of_mass();
      glm::vec3 rb =
          m.b ? cp.pos_world - m.b->get_centre_of_mass() : glm::vec3(0.f);
      friction_constraints.emplace_back(m.a, m.b, cp.norm, ra, rb,
                                        &contact_constraints[ci++]);
      auto &fc = friction_constraints.back();
      fc.update_friction_clamping();
      float f0 = std::clamp(cp.tangent_impulse[0], fc.impulse_min[0],
                            fc.impulse_max[0]);
      float f1 = std::clamp(cp.tangent_impulse[1], fc.impulse_min[1],
                            fc.impulse_max[1]);
      fc.impulse_accum[0] = f0;
      fc.impulse_accum[1] = f1;
      fc.apply_impulse_mag(0, f0);
      fc.apply_impulse_mag(1, f1);
    }
  }
}

void ConstraintSolver::write_back(std::vector<Manifold> &manifolds) {
  size_t ci = 0, fi = 0;
  for (auto &m : manifolds) {
    for (size_t i = 0; i < m.num_points; i++) {
      m.contacts[i].norm_impulse = contact_constraints[ci].impulse_accum[0];
      m.contacts[i].tangent_impulse[0] =
          friction_constraints[fi].impulse_accum[0];
      m.contacts[i].tangent_impulse[1] =
          friction_constraints[fi].impulse_accum[1];
      ci++;
      fi++;
    }
  }
}

void ConstraintSolver::solve_constraints(float dt) {

  // 1st pass : impulses
  for (int iter = 0; iter < NUM_ITERATIONS; iter++) {

    for (auto &c : contact_constraints) {
      c.resolve_impulse();
    }

    for (auto &c : friction_constraints) {
      // Update clamp values
      c.update_friction_clamping();
      c.resolve_impulse();
    }
  }

  // 2nd pass : psuedo impulses
  for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
    for (auto &c : contact_constraints) {
      c.resolve_psuedo_impulse(dt);
    }
  }
}
