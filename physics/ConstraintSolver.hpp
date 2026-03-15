#pragma once

#include "Narrowphase.hpp"
#include "Plane.hpp"
#include "RigidBody.hpp"

#define GLM_ENABLE_EXPERIMENTAL
#include "glm/gtx/matrix_cross_product.hpp" // glm::matrixCross3()

#define NUM_ITERATIONS 20

typedef enum {
  DISTANCE,
  HINGE,
  SLIDER,
  PLANE,
  CONTACT,
  FRICTION,
  BALL_SOCKET,
  WELD
} ConstraintType;

template <int n> class Constraint {
public:
  struct Jacobian {
    glm::vec3 va = glm::vec3(0.0f);
    glm::vec3 wa = glm::vec3(0.0f);
    glm::vec3 vb = glm::vec3(0.0f);
    glm::vec3 wb = glm::vec3(0.0f);
  };

  RigidBody *a = nullptr;
  RigidBody *b = nullptr; // If b is nullptr; collision with infinite-mass body

  // Oneshot solving n constraints
  Jacobian jacobian[n]; // n x 12 matrix (depends on dof)
  float effective_mass[n];
  float impulse_accum[n] = {};
  float psuedo_impulse_accum[n] = {}; // for positional correction
  float impulse_min[n] = {};          // for clamping impulses
  float impulse_max[n] = {};
  float initial_closing_velocity;

  float slop_penetration = 0.005f; // 5 mm
  float slop_restitution = 0.5f;   // 0.5 m/s

  Constraint() {
    for (int i = 0; i < n; i++) {
      impulse_accum[i] = 0.0f;
      impulse_min[i] = -1e10f;
      impulse_max[i] = 1e10f;
    }
  }

  /**
   * @brief J * V_1
   * @return float
   */
  float compute_inital_velocity_error(int row) {
    const Jacobian &j = jacobian[row];

    float jv =
        glm::dot(j.va, a->lin_velocity) + glm::dot(j.wa, a->ang_velocity);

    if (b != nullptr) {
      jv += glm::dot(j.vb, b->lin_velocity) + glm::dot(j.wb, b->ang_velocity);
    }

    return jv;
  }

  float compute_psuedo_velocity_error(int row) {
    const Jacobian &j = jacobian[row];

    float jv = glm::dot(j.va, a->psuedo_lin_velocity) +
               glm::dot(j.wa, a->psuedo_ang_velocity);

    if (b != nullptr) {
      jv += glm::dot(j.vb, b->psuedo_lin_velocity) +
            glm::dot(j.wb, b->psuedo_ang_velocity);
    }
    return jv;
  }

  float compute_effective_mass(int row) {
    const Jacobian &j = jacobian[row];

    glm::mat3 aii = a->get_world_inverse_inertia();
    float k = a->properties.inv_mass * glm::dot(j.va, j.va) +
              glm::dot(j.wa, aii * j.wa);

    if (b != nullptr) {
      glm::mat3 bii = b->get_world_inverse_inertia();
      k += b->properties.inv_mass * glm::dot(j.vb, j.vb) +
           glm::dot(j.wb, bii * j.wb);
    }
    // return effective_mass;
    return 1.0f / k;
  }

  void apply_impulse_mag(int row, float lambda) {
    const Jacobian &j = jacobian[row];

    a->lin_velocity += a->properties.inv_mass * j.va * lambda;
    a->ang_velocity += a->get_world_inverse_inertia() * j.wa * lambda;

    if (b != nullptr) {
      b->lin_velocity += b->properties.inv_mass * j.vb * lambda;
      b->ang_velocity += b->get_world_inverse_inertia() * j.wb * lambda;
    }
  }

  void apply_psuedo_impulse_mag(int row, float psuedo_lambda) {
    const Jacobian &j = jacobian[row];

    a->psuedo_lin_velocity += a->properties.inv_mass * j.va * psuedo_lambda;
    a->psuedo_ang_velocity +=
        a->get_world_inverse_inertia() * j.wa * psuedo_lambda;

    if (b != nullptr) {
      b->psuedo_lin_velocity += b->properties.inv_mass * j.vb * psuedo_lambda;
      b->psuedo_ang_velocity +=
          b->get_world_inverse_inertia() * j.wb * psuedo_lambda;

      // b->lin_velocity *= damp;
      // b->ang_velocity *= damp;
    }
  }

  virtual void compute_jacobian() = 0;
  virtual void update_constraint(double dt) {};
  virtual bool is_inequality_constraint() const = 0;
  virtual int get_degree() const = 0;
};

class ContactConstraint : public Constraint<1> {
public:
  float penetration = 0.0f;
  float test_lambda = 0.0f;
  float test_restitution = 0.0f;
  float test_baumgarte = 0.0f;

  glm::vec3 normal = glm::vec3(0.0f);
  glm::vec3 ra = glm::vec3(0.0f);
  glm::vec3 rb = glm::vec3(0.0f);

  ContactConstraint(RigidBody *a, RigidBody *b, ContactPoint cp) {
    this->a = a;
    this->b = b;

    normal = cp.norm;
    penetration = cp.pen_depth;
    ra = cp.pos - a->get_centre_of_mass();
    rb = (b != nullptr) ? cp.pos - b->get_centre_of_mass() : glm::vec3(0.0f);

    compute_jacobian();

    impulse_accum[0] = 0.0f;
    impulse_min[0] = 0.0f;
    impulse_max[0] = 1e10f;
    effective_mass[0] = compute_effective_mass(0);

    // Relative closing velicity between bodies
    glm::vec3 rel_velocity = -a->lin_velocity - glm::cross(a->ang_velocity, ra);
    if (b != nullptr) {
      rel_velocity += b->lin_velocity + glm::cross(b->ang_velocity, rb);
    }
    initial_closing_velocity = glm::dot(rel_velocity, normal);
  }

  inline bool is_inequality_constraint() const override { return true; }
  inline int get_degree() const override { return 1; }

  float compute_baumgarte_term(float dt) {
    float beta;
    if (b != nullptr) {
      beta = 0.5 * (a->baumgarte_factor + b->baumgarte_factor);
    } else {
      beta = a->baumgarte_factor;
    }

    // Allow slight overlap to avoid constantly apply impulses at rest
    float slop = std::max(penetration - slop_penetration, 0.0f);
    return -(beta / dt) * slop;
  }

  float compute_restitution_term(void) {
    float restitution;
    if (b != nullptr) {
      restitution = 0.5 * (a->restitution_coeff + b->restitution_coeff);
    } else {
      restitution = a->restitution_coeff;
    }

    float slop = std::min(initial_closing_velocity + slop_restitution, 0.0f);
    return restitution * slop;
  }

  void resolve_impulse() {
    float jv = compute_inital_velocity_error(0);
    float b = compute_restitution_term();
    test_restitution = b;

    float lambda = effective_mass[0] * (-(jv + b));

    // Clamp accumulated impulse
    float old_lambda = impulse_accum[0];
    impulse_accum[0] = std::max(impulse_accum[0] + lambda, 0.0f);

    // Apply accumulated impulse delta
    lambda = impulse_accum[0] - old_lambda;
    test_lambda = lambda;
    apply_impulse_mag(0, lambda);
  }

  void resolve_psuedo_impulse(float dt) {
    float jv = compute_psuedo_velocity_error(0);
    float b = compute_baumgarte_term(dt);
    test_baumgarte = b;

    float lambda = effective_mass[0] * (-(jv + b));

    // Clamp accumulated impulse
    float old_lambda = psuedo_impulse_accum[0];
    psuedo_impulse_accum[0] = std::max(psuedo_impulse_accum[0] + lambda, 0.0f);

    // Apply accumulated impulse delta
    lambda = psuedo_impulse_accum[0] - old_lambda;
    apply_psuedo_impulse_mag(0, lambda);
  }

private:
  void compute_jacobian() override {
    jacobian[0].va = normal;
    jacobian[0].wa = glm::cross(ra, normal);
    jacobian[0].vb = -normal;
    jacobian[0].wb = -glm::cross(rb, normal);
  }
};

class FrictionConstraint : public Constraint<2> {
public:
  ContactConstraint *contact_normal = nullptr;

  // contact tangent vectors? need to compute from contact normal
  glm::vec3 u = glm::vec3(0.0f);
  glm::vec3 w = glm::vec3(0.0f);

  glm::vec3 normal = glm::vec3(0.0f);
  glm::vec3 ra = glm::vec3(0.0f);
  glm::vec3 rb = glm::vec3(0.0f);

  FrictionConstraint(RigidBody *a, RigidBody *b, const glm::vec3 &normal,
                     const glm::vec3 &ra, const glm::vec3 &rb,
                     ContactConstraint *contact) {

    this->a = a;
    this->b = b;
    this->contact_normal = contact;

    this->normal = normal;
    this->ra = ra;
    this->rb = rb;

    glm::vec3 vel_a = a->lin_velocity + glm::cross(a->ang_velocity, ra);
    glm::vec3 vel_b = (b != nullptr)
                          ? b->lin_velocity + glm::cross(b->ang_velocity, rb)
                          : glm::vec3(0.f);
    glm::vec3 rel_velocity = vel_b - vel_a;

    compute_orthonormal_basis(normal, rel_velocity);
    compute_jacobian();

    for (int i = 0; i < 2; i++) {
      impulse_accum[i] = 0.f;
      impulse_min[i] = -1e10f; // updated each iteration
      impulse_max[i] = 1e10f;
      effective_mass[i] = compute_effective_mass(i);
    }
  }

  /**
   * @brief Friction force is dependent on normal force. Use current normal
   * impulse to clmap friction impulses
   */
  void update_friction_clamping() {
    if (contact_normal == nullptr) {
      return;
    }

    float friction;
    if (b != nullptr) {
      friction = 0.5 * (a->friction_coeff + b->friction_coeff);
    } else {
      // FIXME: PLACEHOLDER FRICTION COEFF FOR GROUND PLANE
      friction = a->friction_coeff;
    }
    float clamp_friction = friction * contact_normal->impulse_accum[0];

    // clamp friction impluses
    for (int i = 0; i < 2; i++) {
      impulse_min[i] = -clamp_friction;
      impulse_max[i] = clamp_friction;
    }
  }

  void resolve_impulse() {

    for (int row = 0; row < 2; row++) {
      float jv = compute_inital_velocity_error(row);
      float lambda = effective_mass[row] * -jv;

      // Clamp accumulated impulse
      float old_lambda = impulse_accum[row];
      impulse_accum[row] = std::clamp(impulse_accum[row] + lambda,
                                      impulse_min[row], impulse_max[row]);

      // Apply accumulated impulse delta
      lambda = impulse_accum[row] - old_lambda;
      apply_impulse_mag(row, lambda);
    }
  }

  inline bool is_inequality_constraint() const override { return true; }
  inline int get_degree() const override { return 2; }

private:
  void compute_orthonormal_basis(glm::vec3 normal,
                                 glm::vec3 relative_velocity) {

    glm::vec3 vt =
        relative_velocity - glm::dot(relative_velocity, normal) * normal;

    if (glm::length2(vt) > 1e-6f) {
      // Align u with the sliding direction — most physically meaningful
      u = glm::normalize(vt);
    } else {
      // Body is nearly stationary — fall back to arbitrary basis
      glm::vec3 ref = (std::abs(normal.x) < 0.57f) ? glm::vec3(1, 0, 0)
                                                   : glm::vec3(0, 1, 0);
      u = glm::normalize(glm::cross(normal, ref));
    }
    w = glm::normalize(glm::cross(normal, u));
  }

  void compute_jacobian() override {

    // Row 0 along u
    jacobian[0].va = u;
    jacobian[0].wa = glm::cross(ra, u);
    jacobian[0].vb = -u;
    jacobian[0].wb = -glm::cross(rb, u);

    // Row 1 along w
    jacobian[1].va = w;
    jacobian[1].wa = glm::cross(ra, w);
    jacobian[1].vb = -w;
    jacobian[1].wb = -glm::cross(rb, w);
  }
};

class ConstraintSolver {

private:
  std::vector<ContactConstraint> contact_constraints;
  std::vector<FrictionConstraint> friction_constraints;

public:
  void build_constraints(const std::vector<ContactManifold> &contacts) {

    size_t total_points = 0;
    for (auto &manifold : contacts) {
      total_points += manifold.num_points;
    }

    contact_constraints.clear();
    friction_constraints.clear();
    contact_constraints.reserve(total_points);
    friction_constraints.reserve(total_points);

    for (auto &m : contacts) {
      for (int i = 0; i < m.num_points; i++) {

        contact_constraints.emplace_back(m.a, m.b, m.points[i]);
      }
    }

    size_t index = 0;
    for (auto &m : contacts) {
      for (int i = 0; i < m.num_points; i++) {
        const auto &cp = m.points[i];
        glm::vec3 ra = cp.pos - m.a->get_centre_of_mass();
        glm::vec3 rb =
            m.b ? cp.pos - m.b->get_centre_of_mass() : glm::vec3(0.f);
        friction_constraints.emplace_back(m.a, m.b, cp.norm, ra, rb,
                                          &contact_constraints[index++]);
      }
    }
  }

  void solve_constraints(float dt) {

    // Reset pseudo velocities
    for (auto &c : contact_constraints) {
      c.a->psuedo_lin_velocity = glm::vec3(0.0f);
      c.a->psuedo_ang_velocity = glm::vec3(0.0f);
      if (c.b != nullptr) {
        c.b->psuedo_lin_velocity = glm::vec3(0.0f);
        c.b->psuedo_ang_velocity = glm::vec3(0.0f);
      }
    }

    // 1st pass : impulses
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {

      // CLOGI("ITERATION[%d]\n", iter);
      int i = 0;
      for (auto &c : contact_constraints) {
        c.resolve_impulse();

        /*
        CLOGI("CONTACT[%d] - ID %d & ID %d\n penetration: %.4f\n  "
              "accumulated impulse: %.4f\n "
              "delta impulse: "
              "%.4f\n velocity err: %.6f\n"
              "restitution correction: %.6f\n",
              i, c.a->id, (c.b) ? c.b->id : -1, c.penetration,
              c.impulse_accum[0], c.test_lambda,
              c.compute_inital_velocity_error(0), c.test_restitution);
        */
        i++;
      }

      for (auto &c : friction_constraints) {
        // Update clamp values
        c.update_friction_clamping();
        c.resolve_impulse();
      }
    }

    // 2nd pass : psuedo impulses
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {
      // CLOGI("ITERATION[%d]\n", iter);
      int i = 0;
      for (auto &c : contact_constraints) {
        c.resolve_psuedo_impulse(dt);

        /*
        CLOGI("CONTACT[%d] - ID %d & ID %d\n"
              "accumulated psuedo impulse: %.4f\n "
              "%.4f\n psuedo velocity err: %.6f\n"
              "pos correction: %.6f\n",
              i, c.a->id, (c.b) ? c.b->id : -1, c.psuedo_impulse_accum[0],
              c.compute_psuedo_velocity_error(0), c.test_baumgarte);
      */
        i++;
      }
    }
  }
};

// ====================================================
// JOINTS
// ====================================================

class DistanceConstraint : public Constraint<1> {};
class HingeConstraint : public Constraint<1> {};
class SliderConstraint : public Constraint<1> {};
class BallSocketConstraint : public Constraint<1> {};
class WeldConstraint : public Constraint<1> {};
