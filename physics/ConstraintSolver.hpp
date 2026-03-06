#pragma once

#include "Narrowphase.hpp"
#include "Plane.hpp"
#include "RigidBody.hpp"

#define GLM_ENABLE_EXPERIMENTAL
// TODO: HELLO??? DO WE STILL NEED?
#include "glm/gtx/matrix_cross_product.hpp" // glm::matrixCross3()

#define NUM_ITERATIONS 20

typedef enum { DISTANCE, HINGE, SLIDER, PLANE, CONTACT, FRICTION, BALL_SOCKET, WELD } Type;

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
  Jacobian jacobian[n];        // n x 12 matrix (depends on dof)
  float impulse_accum[n] = {}; // impulse magnitude
  float impulse_min[n] = {};   // for clamping impulses
  float impulse_max[n] = {};
  float beta = 0.0f;

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

    float jv = glm::dot(j.va, a->lin_velocity) + glm::dot(j.wa, a->ang_velocity);

    if (b != nullptr) {
      jv += glm::dot(j.vb, b->lin_velocity) + glm::dot(j.wb, b->ang_velocity);
    }

    return jv;
  }

  float compute_effective_mass(int row) {
    const Jacobian &j = jacobian[row];

    glm::mat3 aii = a->get_world_inverse_inertia();
    // float effective_mass = a->properties.inv_mass * glm::dot(j.va, j.va) + glm::dot(j.wa, aii * j.wa);
    float k = a->properties.inv_mass + glm::dot(j.wa, aii * j.wa);

    if (b != nullptr) {
      glm::mat3 bii = b->get_world_inverse_inertia();
      // effective_mass += b->properties.inv_mass * glm::dot(j.vb, j.vb) + glm::dot(j.wb, bii * j.wb);
      k += b->properties.inv_mass + glm::dot(j.wb, bii * j.wb);
    }
    // return effective_mass;
    return 1.0f / k;
  }

  void apply_impulse(int row, float lambda) {
    const Jacobian &j = jacobian[row];

    a->lin_velocity += a->properties.inv_mass * j.va * lambda;
    a->ang_velocity += a->get_world_inverse_inertia() * j.wa * lambda;

    if (b != nullptr) {
      b->lin_velocity += b->properties.inv_mass * j.vb * lambda;
      b->ang_velocity += b->get_world_inverse_inertia() * j.wb * lambda;
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
  float slop_factor = 0.005f;

  float lambda_test = 0.0f;

  glm::vec3 normal = glm::vec3(0.0f);
  glm::vec3 ra = glm::vec3(0.0f);
  glm::vec3 rb = glm::vec3(0.0f);

  ContactConstraint(RigidBody *a, RigidBody *b, Narrowphase::ContactPoint cp) {
    this->a = a;
    this->b = b;

    normal = cp.normal;
    penetration = cp.penetration;
    ra = cp.point - a->get_centre_of_mass();
    rb = (b != nullptr) ? cp.point - b->get_centre_of_mass() : glm::vec3(0.0f);

    compute_jacobian();

    impulse_accum[0] = 0.0f;
    impulse_min[0] = 0.0f;
    impulse_max[0] = 1e10f;
  }

  inline bool is_inequality_constraint() const override { return true; }
  inline int get_degree() const override { return 1; }

  float compute_bias(float dt) {

    float beta;
    float restitution;
    if (b != nullptr) {
      beta = a->baumgarte_factor * b->baumgarte_factor;
      restitution = a->restitution_coeff * b->restitution_coeff;
    } else {
      beta = a->baumgarte_factor;
      restitution = a->restitution_coeff;
    }

    float slop_penetration = std::max(penetration - slop_factor, 0.0f);

    glm::vec3 rel_velocity = -a->lin_velocity - glm::cross(a->ang_velocity, ra);

    if (b != nullptr) {
      rel_velocity += b->lin_velocity + glm::cross(b->ang_velocity, rb);
    }

    float norm_velocity = glm::dot(rel_velocity, normal);

    return -(beta / dt) * slop_penetration + restitution * norm_velocity;
  }

  void resolve(float dt) {
    float jv = compute_inital_velocity_error(0);
    float m_eff = compute_effective_mass(0);
    float b = compute_bias(dt);

    float lambda = m_eff * (-(jv + b));

    // clamp lambda
    float old_lambda = impulse_accum[0];
    impulse_accum[0] = std::max(impulse_accum[0] + lambda, 0.0f);
    lambda = impulse_accum[0] - old_lambda;

    lambda_test = lambda;

    apply_impulse(0, lambda);
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

  FrictionConstraint(RigidBody *a,
      RigidBody *b,
      const glm::vec3 &normal,
      const glm::vec3 &ra,
      const glm::vec3 &rb,
      ContactConstraint *contact) {

    this->a = a;
    this->b = b;
    this->contact_normal = contact;

    this->normal = normal;
    this->ra = ra;
    this->rb = rb;

    glm::vec3 vel_a = a->lin_velocity + glm::cross(a->ang_velocity, ra);
    glm::vec3 vel_b = (b != nullptr) ? b->lin_velocity + glm::cross(b->ang_velocity, rb) : glm::vec3(0.f);
    glm::vec3 rel_velocity = vel_b - vel_a;

    compute_orthonormal_basis(normal, rel_velocity);
    compute_jacobian();

    for (int i = 0; i < 2; i++) {
      impulse_accum[i] = 0.f;
      impulse_min[i] = -1e10f; // updated each iteration
      impulse_max[i] = 1e10f;
    }
  }

  /**
   * @brief Friction force is dependent on normal force. Use current normal impulse to clmap friction impulses
   */
  void update_friction_clamping() {
    if (contact_normal == nullptr) {
      return;
    }

    float friction;
    if (b != nullptr) {
      friction = std::sqrt(a->friction_coeff * b->friction_coeff);
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

  void resolve() {

    for (int row = 0; row < 2; row++) {
      float jv = compute_inital_velocity_error(row);
      float m_eff = compute_effective_mass(row);
      float lambda = m_eff * -jv;

      // clamp lambda
      float old_lambda = impulse_accum[row];
      impulse_accum[row] = std::clamp(impulse_accum[row] + lambda, impulse_min[row], impulse_max[row]);
      lambda = impulse_accum[row] - old_lambda;

      apply_impulse(row, lambda);
    }
  }

  inline bool is_inequality_constraint() const override { return true; }
  inline int get_degree() const override { return 2; }

private:
  void compute_orthonormal_basis(glm::vec3 normal, glm::vec3 relative_velocity) {

    glm::vec3 vt = relative_velocity - glm::dot(relative_velocity, normal) * normal;

    if (glm::length2(vt) > 1e-6f) {
      // Align u with the sliding direction — most physically meaningful
      u = glm::normalize(vt);
    } else {
      // Body is nearly stationary — fall back to arbitrary basis
      glm::vec3 ref = (std::abs(normal.x) < 0.57f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
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
  void build_constraints(std::vector<Narrowphase::ContactManifold> contacts) {

    size_t total_points = 0;
    for (auto &manifold : contacts) {
      total_points += manifold.num_points;
    }

    contact_constraints.clear();
    friction_constraints.clear();
    contact_constraints.reserve(total_points);
    friction_constraints.reserve(total_points);

    for (auto &manifold : contacts) {
      for (int i = 0; i < manifold.num_points; i++) {
        contact_constraints.emplace_back(manifold.a, manifold.b, manifold.points[i]);
      }
    }

    size_t index = 0;
    for (auto &manifold : contacts) {
      for (int i = 0; i < manifold.num_points; i++) {
        const auto &cp = manifold.points[i];
        glm::vec3 ra = cp.point - manifold.a->get_centre_of_mass();
        glm::vec3 rb = manifold.b ? cp.point - manifold.b->get_centre_of_mass() : glm::vec3(0.f);
        friction_constraints.emplace_back(manifold.a, manifold.b, cp.normal, ra, rb, &contact_constraints[index++]);
      }
    }
  }

  void solve_constraints(float dt) {

    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {

      float total_error = 0.f;

      for (auto &c : contact_constraints) {
        c.resolve(dt);

        total_error += std::abs(c.compute_inital_velocity_error(0));
      }

      for (auto &c : friction_constraints) {
        // Update clamp values
        c.update_friction_clamping();
        c.resolve();
      }

      // CLOGI("iter %d total_err=%.6f\n", iter, total_error);
    }

    /*
    for (int i = 0; i < (int)contact_constraints.size(); ++i) {
      auto &c = contact_constraints[i];
      CLOGI("contact[%d]: pen=%.4f  λ=%.4f  JV=%.6f  body_a=%d  body_b=%s\n",
          i,
          c.penetration,
          c.lambda_test,
          c.compute_inital_velocity_error(0),
          c.a->id,
          c.b ? std::to_string(c.b->id).c_str() : "plane");
    }
    */
  }
};
