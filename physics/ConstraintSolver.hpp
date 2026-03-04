#pragma once

#include "Narrowphase.hpp"
#include "Plane.hpp"
#include "RigidBody.hpp"

#define NUM_ITERATIONS 20

typedef enum { DISTANCE, HINGE, SLIDER, PLANE, CONTACT, FRICTION, BALL_SOCKET, WELD } ConstraintType;

// SOO WE HAVE PERSISTENT CONSTRAINTS (JOINTS) AND TEMP CONSTRAINTS (CONTACT/FRICTION)
// PERSISTENT CONSTRAINTS EXIST ACROSS FRAMES; HENCE `update_constraint()`
// TEMP CONSTRAINTS ONLY EXIST PER-FRAME; COMPUTED/SOLVED WITHIN SAME FRAME
//
struct CachedContact {
  glm::vec3 point;
  float normal_impulse;
  float friction_impulse[2];
};

struct ContactCache {
  RigidBody *a, *b;
  std::vector<CachedContact> points;
};

template <int n> class Constraint {
public:
  struct Jacobian {
    glm::vec3 a_linear = glm::vec3(0.0f);
    glm::vec3 a_angular = glm::vec3(0.0f);
    glm::vec3 b_linear = glm::vec3(0.0f);
    glm::vec3 b_angular = glm::vec3(0.0f);
  };

  RigidBody *a = nullptr;
  RigidBody *b = nullptr; // If b is nullptr; collision with infinite-mass body

  Jacobian jacobian[n];        // jacobian n x 12 matrix (depends on dof)
  float impulse_accum[n] = {}; // impulse magnitude
  float impulse_min[n] = {};   // for clamping impulses
  float impulse_max[n] = {};
  float bias = 0.0f;

  Constraint() {
    for (int i = 0; i < n; i++) {
      impulse_accum[i] = 0.0f;
      impulse_min[i] = -1e10f;
      impulse_max[i] = 1e10f;
    }
  }

  /**
   * @brief JV
   * @return float
   */
  float compute_velocity_error(int row) {
    const Jacobian &j = jacobian[row];

    float jv = glm::dot(j.a_linear, a->lin_velocity) + glm::dot(j.a_angular, a->ang_velocity);

    if (b != nullptr) {
      jv += glm::dot(j.b_linear, b->lin_velocity) + glm::dot(j.b_angular, b->ang_velocity);
    }

    return jv;
  }

  float compute_effective_mass(int row) {
    const Jacobian &j = jacobian[row];

    glm::mat3 a_ii = a->get_world_inverse_inertia();
    float effective_mass =
        a->properties.inv_mass * glm::dot(j.a_linear, j.a_linear) + glm::dot(j.a_angular, a_ii * j.a_angular);

    if (b != nullptr) {
      glm::mat3 b_ii = b->get_world_inverse_inertia();
      effective_mass +=
          b->properties.inv_mass * glm::dot(j.b_linear, j.b_linear) + glm::dot(j.b_angular, b_ii * j.b_angular);
    }
    return effective_mass;
  }

  void apply_impulse(int row, float lambda) {
    const Jacobian &j = jacobian[row];

    // WARNING: QUESTIONABLE MATH
    a->lin_velocity += (a->properties.inv_mass * lambda) * j.a_linear;
    float angular_mag = glm::length(j.a_angular);
    if (angular_mag > 1e-4f)
      a->ang_velocity += a->get_world_inverse_inertia() * (lambda * j.a_angular);

    if (b != nullptr) {
      b->lin_velocity += (b->properties.inv_mass * lambda) * j.b_linear;
      float b_angular_mag = glm::length(j.b_angular);
      if (b_angular_mag > 1e-4f)
        b->ang_velocity += b->get_world_inverse_inertia() * (lambda * j.b_angular);
    }
    /*
    a->ang_velocity += a->get_world_inverse_inertia() * (delta_lambda * jj.a_angular);

    if (b != nullptr) {
      b->lin_velocity += (b->properties.inv_mass * delta_lambda) * jj.b_linear;
      b->ang_velocity += b->get_world_inverse_inertia() * (delta_lambda * jj.b_angular);
    }
    */
  }

  virtual void compute_jacobian() = 0;
  virtual void update_constraint(double dt) {};
  virtual bool is_inequality_constraint() const = 0;
  virtual int get_degree() const = 0;
};

// CONTACT
class ContactConstraint : public Constraint<1> {
public:
  float penetration = 0.0f;
  float baumgarte_factor = 0.2f;
  float slop_factor = 0.005f;
  float restitution_coeff = 0.0f;

  glm::vec3 normal = glm::vec3(0.0f);
  glm::vec3 r_a = glm::vec3(0.0f);
  glm::vec3 r_b = glm::vec3(0.0f);

  // WARNING:
  // v_n_initial: relative normal velocity captured BEFORE the solve loop,
  // needed for correct restitution (Deshpande / Catto formulation).
  float v_normal_initial = 0.0f;

  ContactConstraint(RigidBody *a, RigidBody *b, Narrowphase::ContactPoint cp) {
    this->a = a;
    this->b = b;

    normal = cp.normal;
    penetration = cp.penetration;
    r_a = cp.point - a->get_centre_of_mass();
    r_b = (b != nullptr) ? cp.point - b->get_centre_of_mass() : glm::vec3(0.0f);

    compute_jacobian();
    cache_initial_velocity();

    impulse_accum[0] = 0.0f;
    impulse_min[0] = 0.0f;
    impulse_max[0] = 1e10f;
  }

  ContactConstraint(RigidBody *a, const glm::vec3 &plane_normal, const glm::vec3 &contact_point, float pen) {
    this->a = a;
    this->b = nullptr;

    normal = plane_normal; // pointing away from plane surface toward a
    penetration = pen;
    r_a = contact_point - a->get_centre_of_mass();
    r_b = glm::vec3(0.f); // b is static, irrelevant

    compute_jacobian();
    cache_initial_velocity();

    impulse_accum[0] = 0.f;
    impulse_min[0] = 0.f;
    impulse_max[0] = 1e10f;
  }

  float compute_lambda(float dt) {
    float mass_eff = compute_effective_mass(0);

    // Avoid zero division
    if (mass_eff < 1e-10f) {
      return 0.0f;
    }

    float jv = compute_velocity_error(0);
    float b = compute_bias(dt);

    return -(jv + b) / mass_eff;
  }

  // Split impulse: correct position drift WITHOUT touching true velocities.
  // Call this once per frame AFTER the PGS velocity loop.
  // Uses a pseudo-velocity (δv) that moves bodies apart and is discarded —
  // it never feeds back into the real velocity state.
  void apply_positional_correction() {
    float depth = penetration - slop_factor;
    if (depth <= 0.0f) {
      return;
    }

    float total_inv_mass = a->properties.inv_mass + (b ? b->properties.inv_mass : 0.0f);
    if (total_inv_mass < 1e-10f)
      return;

    // correction magnitude in metres, split by mass ratio
    float correction = baumgarte_factor * depth / total_inv_mass;

    // Separate bodies along the contact normal
    a->position += a->properties.inv_mass * correction * normal; // push A away from B
    if (b != nullptr) {
      b->position -= b->properties.inv_mass * correction * normal; // push B away from A
    }
  }

  inline bool is_inequality_constraint() const override { return true; }
  inline int get_degree() const override { return 1; }

private:
  void compute_jacobian() override {
    jacobian[0].a_linear = normal;
    jacobian[0].a_angular = glm::cross(r_a, normal);
    jacobian[0].b_linear = -normal;
    jacobian[0].b_angular = -glm::cross(r_b, normal);

    /*
    j[0].a_linear = -normal;
    j[0].a_angular = -glm::cross(r_a, normal);
    j[0].b_linear = normal;
    j[0].b_angular = glm::cross(r_b, normal);
    */
  }

  void cache_initial_velocity() {
    glm::vec3 v_a_contact = a->lin_velocity + glm::cross(a->ang_velocity, r_a);
    glm::vec3 v_b_contact = b ? (b->lin_velocity + glm::cross(b->ang_velocity, r_b)) : glm::vec3(0.f);
    v_normal_initial = glm::dot(normal, v_a_contact - v_b_contact);
  }

  float compute_bias(float dt) const {
    const float restitution_thres = 1.0f;
    float e = (std::abs(v_normal_initial) < restitution_thres) ? 0.0f : restitution_coeff;
    return e * v_normal_initial;
  }
};

// FRICTION
class FrictionConstraint : public Constraint<2> {
public:
  float fric_coeff = 0.4f;

  ContactConstraint *contact = nullptr;

  // contact tangent vectors? need to compute from contact normal
  glm::vec3 u = glm::vec3(0.0f);
  glm::vec3 w = glm::vec3(0.0f);

  glm::vec3 normal = glm::vec3(0.0f);
  glm::vec3 r_a = glm::vec3(0.0f);
  glm::vec3 r_b = glm::vec3(0.0f);

  FrictionConstraint(RigidBody *a,
      RigidBody *b,
      const glm::vec3 &normal,
      const glm::vec3 &r_a,
      const glm::vec3 &r_b,
      ContactConstraint *contact) {

    this->a = a;
    this->b = b;
    this->contact = contact;

    this->normal = normal;
    this->r_a = r_a;
    this->r_b = r_b;

    compute_orthonormal_basis(normal);
    compute_jacobian();

    for (int i = 0; i < 2; i++) {
      impulse_accum[i] = 0.f;
      impulse_min[i] = -1e10f; // updated each iteration
      impulse_max[i] = 1e10f;
    }
  }

  float compute_lambda_row(int row, float dt) {
    float mass_eff = compute_effective_mass(row);

    // Avoid zero division
    if (mass_eff < 1e-10f) {
      return 0.0f;
    }

    float jv = compute_velocity_error(row);

    // Friction has no bias
    return -jv / mass_eff;
  }

  /**
   * @brief Friction force is dependent on normal force. Use current normal impulse to clmap friction impulses
   */
  void update_friction_clamping() {
    if (contact == nullptr) {
      return;
    }

    float clamp_friction = fric_coeff * contact->impulse_accum[0];

    // clamp friction impluses
    for (int i = 0; i < 2; i++) {
      impulse_min[i] = -clamp_friction;
      impulse_max[i] = clamp_friction;
    }
  }

  inline bool is_inequality_constraint() const override { return true; }
  inline int get_degree() const override { return 2; }

private:
  void compute_orthonormal_basis(glm::vec3 normal) {
    glm::vec3 ref = (std::abs(normal.x) < 0.57f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
    u = glm::normalize(glm::cross(normal, ref));
    w = glm::cross(normal, u);
  }

  void compute_jacobian() override {

    // Row 0 along u
    jacobian[0].a_linear = u;
    jacobian[0].a_angular = glm::cross(r_a, u);
    jacobian[0].b_linear = -u;
    jacobian[0].b_angular = -glm::cross(r_b, u);

    // Row 1 along w
    jacobian[1].a_linear = w;
    jacobian[1].a_angular = glm::cross(r_a, w);
    jacobian[1].b_linear = -w;
    jacobian[1].b_angular = -glm::cross(r_b, w);
  }
};

class ConstraintSolver {
public:
  std::vector<ContactCache> contact_cache;

  float find_cached_impulse(RigidBody *a, RigidBody *b, const glm::vec3 &point) {
    for (auto &cache : contact_cache) {
      if (cache.a != a || cache.b != b)
        continue;
      for (auto &cp : cache.points) {
        // Match by proximity in world space
        if (glm::length(cp.point - point) < 0.02f)
          return cp.normal_impulse;
      }
    }
    return 0.0f;
  }

  void solve(std::vector<Narrowphase::ContactManifold> contacts, double dt) {

    std::vector<ContactConstraint> contact_constraints;
    std::vector<FrictionConstraint> friction_constraints;

    size_t total_points = 0;
    for (auto &manifold : contacts) {
      total_points += manifold.num_points;
    }

    contact_constraints.reserve(total_points);
    friction_constraints.reserve(total_points);

    // 1. Build Transient Constraints
    for (auto &manifold : contacts) {
      for (int i = 0; i < manifold.num_points; i++) {
        contact_constraints.emplace_back(manifold.a, manifold.b, manifold.points[i]);
        contact_constraints[i].impulse_accum[0] = find_cached_impulse(manifold.a, manifold.b, manifold.points[i].point);
        if (contact_constraints[i].impulse_accum[0] > 0.0f)
          contact_constraints[i].apply_impulse(0, contact_constraints[i].impulse_accum[0]);
      }
    }

    int idx = 0;
    for (auto &manifold : contacts) {
      for (int i = 0; i < manifold.num_points; i++) {
        const auto &cp = manifold.points[i];
        glm::vec3 r_a = cp.point - manifold.a->get_centre_of_mass();
        glm::vec3 r_b = manifold.b ? cp.point - manifold.b->get_centre_of_mass() : glm::vec3(0.f);
        friction_constraints.emplace_back(manifold.a, manifold.b, cp.normal, r_a, r_b, &contact_constraints[idx++]);
      }
    }

    // 2. Iterative solver loop
    for (int iter = 0; iter < NUM_ITERATIONS; iter++) {

      for (auto &c : contact_constraints) {
        float impulse_magnitude = c.compute_lambda(dt);

        // Normal clamping
        float temp = c.impulse_accum[0];
        c.impulse_accum[0] = std::max(c.impulse_accum[0] + impulse_magnitude, 0.0f);
        impulse_magnitude = c.impulse_accum[0] - temp;

        if (std::abs(impulse_magnitude) > 1e-10f) {
          c.apply_impulse(0, impulse_magnitude);
        }
      }

      for (auto &c : friction_constraints) {

        // Update clamp values
        c.update_friction_clamping();
        for (int row = 0; row < 2; row++) {
          float impulse_magnitude = c.compute_lambda_row(row, dt);

          // Friction clamping
          float temp = c.impulse_accum[row];
          c.impulse_accum[row] =
              std::clamp(c.impulse_accum[row] + impulse_magnitude, c.impulse_min[row], c.impulse_max[row]);
          impulse_magnitude = c.impulse_accum[row] - temp;

          if (std::abs(impulse_magnitude) > 1e-10f)
            c.apply_impulse(row, impulse_magnitude);
        }
      }
    }

    // 3. Split impulse: position correction pass (runs once, after velocity solve).
    // Moves bodies out of penetration without injecting velocity.
    // Deduplication ensures bodies with multiple contact points aren't
    // moved once per contact point (would over-correct).
    std::unordered_set<RigidBody *> corrected;
    for (auto &c : contact_constraints) {
      c.apply_positional_correction();
      if (c.a && corrected.find(c.a) == corrected.end()) {
        c.a->update_transform();
        corrected.insert(c.a);
      }
      if (c.b && corrected.find(c.b) == corrected.end()) {
        c.b->update_transform();
        corrected.insert(c.b);
      }
    }
  }
};

/*
// JOINTS

// DISTANCE
class DistanceConstraint : public Constraint<1> {
public:
  static const int DOF = 1;
  DistanceConstraint(RigidBody *a, RigidBody *b, float penetration);

  void update_constraint(double dt) override;

  inline bool is_inequality_constraint() const override { return false; }
  inline int get_degree() const override { return 1; }

private:
  glm::vec3 r_a;
  glm::vec3 r_b;
};

// HINGE
class HingeConstraint : public Constraint<5> {
public:
  static const int DOF = 1;
  HingeConstraint(RigidBody *a, RigidBody *b, float penetration);

  void update_constraint(double dt) override;
};

// BALL & SOCKET
class BallSocketConstraint : public Constraint<3> {
public:
  BallSocketConstraint(RigidBody *a, RigidBody *b, float penetration);

  void update_constraint(double dt) override;
};

// SLIDER
class SliderConstraint : public Constraint<5> {
public:
  SliderConstraint(RigidBody *a, RigidBody *b, float penetration);

  void update_constraint(double dt) override;
};

// WELD
class WeldConstraint : public Constraint<6> {
public:
  static const int DOF = 6;
  WeldConstraint(RigidBody *a, RigidBody *b, float penetration);

  void update_constraint(double dt) override;
};
*/
