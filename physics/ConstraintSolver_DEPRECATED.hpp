#pragma once

// ─────────────────────────────────────────────────────────────────────────────
// ConstraintSolver.hpp  –  Projected Gauss–Seidel (PGS) constraint solver
//
// MATHEMATICAL FOUNDATION
// ───────────────────────
// Each constraint row i enforces:   J_i · V  +  b_i  ≥  0   (inequality)
//                               or  J_i · V  +  b_i  =  0   (equality)
//
// V  = [v_a  ω_a  v_b  ω_b]^T       (12-vector of velocities)
// J  = [J_va  J_ωa  J_vb  J_ωb]     (Jacobian row, 1×12)
// M⁻¹ = block-diag(m_a⁻¹I, I_a⁻¹, m_b⁻¹I, I_b⁻¹)
//
// Per-iteration impulse update (PGS):
//   Δλ_i  =  −(J_i V + b_i) / (J_i M⁻¹ J_i^T)
//   λ_i   =  clamp(λ_i + Δλ_i,  lo_i,  hi_i)
//   ΔV    =  M⁻¹ J_i^T  ·  (clamped Δλ_i)
//
// Contact bias (velocity-level, per Catto / Deshpande):
//   b = (β/dt)·max(d - slop, 0)   ← Baumgarte position correction
//     + e · v_n_initial             ← restitution (only when closing fast)
//
// Where:
//   β     = baumgarte_factor   (typically 0.1–0.3)
//   d     = penetration depth
//   slop  = allowed penetration before correction kicks in (~0.005 m)
//   e     = coefficient of restitution
//   v_n_initial = dot(n, v_a − v_b) captured BEFORE the solve loop
//                 (negative = closing, positive = separating)
//
// Split-impulse positional correction runs ONCE after the velocity loop.
// It moves bodies along the contact normal proportional to their inverse mass
// WITHOUT injecting energy into the velocity state.
// ─────────────────────────────────────────────────────────────────────────────

#include <algorithm>
#include <unordered_set> // FIX 5: was missing
#include <vector>

#include "Narrowphase.hpp"
// #include "PersistentManifold.hpp"
#include "Plane.hpp"
#include "RigidBody.hpp"

static constexpr int NUM_ITERATIONS = 30;
static constexpr float RESTITUTION_THRESHOLD = 1.0f; // m/s – below this, no bounce

// ─────────────────────────────────────────────────────────────────────────────
// Constraint<N>  –  base class templated on number of Jacobian rows
// ─────────────────────────────────────────────────────────────────────────────
template <int N> class Constraint {
public:
  // One Jacobian row: J = [J_va | J_ωa | J_vb | J_ωb]
  struct JRow {
    glm::vec3 va = glm::vec3(0.f); // coefficient of v_a  (linear A)
    glm::vec3 wa = glm::vec3(0.f); // coefficient of ω_a  (angular A)
    glm::vec3 vb = glm::vec3(0.f); // coefficient of v_b  (linear B)
    glm::vec3 wb = glm::vec3(0.f); // coefficient of ω_b  (angular B)
  };

  RigidBody *a = nullptr;
  RigidBody *b = nullptr; // nullptr → b is a static/infinite-mass body

  JRow J[N];
  float lambda[N] = {}; // accumulated impulse magnitude
  float lo[N] = {};     // lower clamp (0 for non-penetration, -∞ for equality)
  float hi[N] = {};     // upper clamp

  Constraint() {
    for (int i = 0; i < N; ++i) {
      lambda[i] = 0.f;
      lo[i] = -1e10f;
      hi[i] = 1e10f;
    }
  }

  virtual ~Constraint() = default;

  // ── JV: scalar velocity error for row r ──────────────────────────────────
  // JV = J_va·v_a + J_ωa·ω_a + J_vb·v_b + J_ωb·ω_b
  float velocity_error(int r) const {
    const JRow &j = J[r];
    float val = glm::dot(j.va, a->lin_velocity) + glm::dot(j.wa, a->ang_velocity);
    if (b != nullptr) {
      val += glm::dot(j.vb, b->lin_velocity) + glm::dot(j.wb, b->ang_velocity);
    }
    return val;
  }

  // ── Effective mass: k = J M⁻¹ J^T ───────────────────────────────────────
  // k = m_a⁻¹|J_va|² + J_ωa·(I_a⁻¹ J_ωa)
  //   + m_b⁻¹|J_vb|² + J_ωb·(I_b⁻¹ J_ωb)   [if b ≠ null]
  float effective_mass(int r) const {
    const JRow &j = J[r];
    glm::mat3 Ia_inv = a->get_world_inverse_inertia();
    float k = a->properties.inv_mass * glm::dot(j.va, j.va) + glm::dot(j.wa, Ia_inv * j.wa);
    if (b) {
      glm::mat3 Ib_inv = b->get_world_inverse_inertia();
      k += b->properties.inv_mass * glm::dot(j.vb, j.vb) + glm::dot(j.wb, Ib_inv * j.wb);
    }
    return k;
  }

  // ── Apply impulse: ΔV = M⁻¹ J^T · δλ ────────────────────────────────────
  // Δv_a = m_a⁻¹ · J_va · δλ
  // Δω_a = I_a⁻¹ · J_ωa · δλ
  // (same for b)
  //
  // FIX 1: The original code had:
  //   if (glm::length(j.a_angular) > 1e-4f)   ← WRONG
  //     ang_velocity += I_inv * (λ · J_ang)
  // This silently skipped the angular impulse for small J_ang, but
  // effective_mass() ALWAYS counted that term → computed λ was wrong.
  // Now we always apply the full impulse unconditionally.
  void apply_impulse(int r, float delta_lambda) {
    const JRow &j = J[r];
    glm::mat3 Ia_inv = a->get_world_inverse_inertia();

    a->lin_velocity += (a->properties.inv_mass * delta_lambda) * j.va;
    a->ang_velocity += Ia_inv * (delta_lambda * j.wa); // always applied

    if (b) {
      glm::mat3 Ib_inv = b->get_world_inverse_inertia();
      b->lin_velocity += (b->properties.inv_mass * delta_lambda) * j.vb;
      b->ang_velocity += Ib_inv * (delta_lambda * j.wb);
    }
  }

  virtual void build_jacobian() = 0;
  virtual bool is_inequality() const = 0;
  virtual int degrees() const = 0;
};

// ─────────────────────────────────────────────────────────────────────────────
// ContactConstraint  –  1-row non-penetration constraint
//
// Constraint:  n · (v_a + ω_a×r_a  −  v_b − ω_b×r_b) + b  ≥  0
//
// Jacobian derivation using scalar triple product identity:
//   n · (ω_a × r_a)  =  ω_a · (r_a × n)
// Therefore:
//   J_va = n,           J_ωa = r_a × n
//   J_vb = −n,          J_ωb = −(r_b × n)
// ─────────────────────────────────────────────────────────────────────────────
class ContactConstraint : public Constraint<1> {
public:
  float penetration = 0.f;
  float baumgarte_factor = 0.05f;
  float slop_factor = 0.005f;
  float restitution_coeff = 0.0f;

  glm::vec3 normal = glm::vec3(0.f);
  glm::vec3 ra = glm::vec3(0.f);
  glm::vec3 rb = glm::vec3(0.f);

  // Pre-solve relative normal velocity (needed for correct restitution).
  // Captured BEFORE the PGS loop starts so the impulse target is stable.
  float v_normal_pre = 0.f;

  // ── Constructor: rigid body B ─────────────────────────────────────────────
  ContactConstraint(RigidBody *a, RigidBody *b, const Narrowphase::ContactPoint &cp) {
    this->a = a;
    this->b = b;
    normal = cp.normal;
    penetration = cp.penetration;
    ra = cp.point - a->get_centre_of_mass();
    rb = b ? (cp.point - b->get_centre_of_mass()) : glm::vec3(0.f);

    build_jacobian();
    cache_pre_velocity();

    lambda[0] = 0.f;
    lo[0] = 0.f; // non-penetration: impulse must push apart
    hi[0] = 1e10f;
  }

  // ── Constructor: infinite-mass plane (b == nullptr) ───────────────────────
  ContactConstraint(RigidBody *a, const glm::vec3 &plane_normal, const glm::vec3 &contact_point, float pen) {
    this->a = a;
    this->b = nullptr;
    normal = plane_normal;
    penetration = pen;
    ra = contact_point - a->get_centre_of_mass();
    rb = glm::vec3(0.f);

    build_jacobian();
    cache_pre_velocity();

    lambda[0] = 0.f;
    lo[0] = 0.f;
    hi[0] = 1e10f;
  }

  // ── Single-iteration PGS step ─────────────────────────────────────────────
  // Δλ = −(JV + b) / k
  float compute_delta_lambda(float dt) const {
    float k = effective_mass(0);
    if (k < 1e-10f)
      return 0.f;
    return -(velocity_error(0) + compute_bias(dt)) / k;
  }

  // ── Split-impulse positional correction ───────────────────────────────────
  // Moves bodies out of penetration by directly adjusting positions.
  // Proportional to each body's share of the total inverse mass.
  // Runs ONCE after the full PGS velocity loop – does NOT modify velocities.
  void apply_positional_correction() const {
    float depth = penetration - slop_factor;
    if (depth <= 0.f)
      return;

    float total_inv_mass = a->properties.inv_mass + (b ? b->properties.inv_mass : 0.f);
    if (total_inv_mass < 1e-10f)
      return;

    // Cap correction to 2cm per frame max — prevents large initial penetrations
    // from launching bodies
    float max_correction = 0.02f;
    float correction = std::min(baumgarte_factor * depth / total_inv_mass, max_correction);

    a->position += a->properties.inv_mass * correction * normal;
    if (b)
      b->position -= b->properties.inv_mass * correction * normal;
  }

  bool is_inequality() const override { return true; }
  int degrees() const override { return 1; }

private:
  void build_jacobian() override {
    // J_va = n,   J_ωa = r_a × n
    // J_vb = −n,  J_ωb = −(r_b × n)
    J[0].va = normal;
    J[0].wa = glm::cross(ra, normal);
    J[0].vb = -normal;
    J[0].wb = -glm::cross(rb, normal);
  }

  void cache_pre_velocity() {
    glm::vec3 va_contact = a->lin_velocity + glm::cross(a->ang_velocity, ra);
    glm::vec3 vb_contact = b ? (b->lin_velocity + glm::cross(b->ang_velocity, rb)) : glm::vec3(0.f);
    v_normal_pre = glm::dot(normal, va_contact - vb_contact);
  }

  // ── Bias term ─────────────────────────────────────────────────────────────
  // b = (β/dt) · max(d − slop, 0)   ← Baumgarte (drives position error to 0)
  //   + e · v_n_pre                  ← restitution  (only when closing fast)
  //
  // FIX 2: Original code only returned  e * v_normal_initial  (no Baumgarte).
  //         Without the Baumgarte term the velocity solve never corrects
  //         position drift, so bodies sink into each other over time.
  //
  // FIX 4: Original used  std::abs(v_normal_initial) < threshold  which fired
  //         restitution even when bodies were SEPARATING (v_n > 0).
  //         Correct check: only bounce when v_n_pre < −threshold (closing).
  float compute_bias(float dt) const {
    // Baumgarte stabilization
    // float baumgarte = -(baumgarte_factor / dt) * std::max(penetration - slop_factor, 0.f);

    // Restitution: only when bodies are closing fast enough
    float e = (v_normal_pre < -RESTITUTION_THRESHOLD) ? restitution_coeff : 0.f;
    float restitution = e * v_normal_pre; // v_n_pre < 0 → restitution < 0 → adds impulse

    // return baumgarte + restitution;
    return restitution;
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// FrictionConstraint  –  2-row tangential constraint
//
// Two constraint rows, one per tangent direction {u, w} ⊥ n:
//   u · (v_a + ω_a×r_a  −  v_b − ω_b×r_b)  =  0
//   w · (v_a + ω_a×r_a  −  v_b − ω_b×r_b)  =  0
//
// Impulse is clamped by Coulomb cone:   |λ_fric| ≤ μ · λ_contact
// ─────────────────────────────────────────────────────────────────────────────
class FrictionConstraint : public Constraint<2> {
public:
  float fric_coeff = 0.4f;
  ContactConstraint *contact_ref = nullptr; // sibling contact (for Coulomb clamp)

  glm::vec3 normal = glm::vec3(0.f);
  glm::vec3 r_a = glm::vec3(0.f);
  glm::vec3 r_b = glm::vec3(0.f);

  FrictionConstraint(RigidBody *a,
      RigidBody *b,
      const glm::vec3 &n,
      const glm::vec3 &ra,
      const glm::vec3 &rb,
      ContactConstraint *contact) {
    this->a = a;
    this->b = b;
    contact_ref = contact;
    normal = n;
    r_a = ra;
    r_b = rb;

    compute_tangent_basis(n);
    build_jacobian();

    for (int i = 0; i < 2; ++i) {
      lambda[i] = 0.f;
      lo[i] = -1e10f; // updated each iteration via update_clamp()
      hi[i] = 1e10f;
    }
  }

  // Recompute Coulomb clamp from current accumulated normal impulse.
  // Called at the start of each friction iteration so the clamp tracks
  // the normal impulse as it evolves through the PGS loop.
  void update_coulomb_clamp() {
    if (!contact_ref)
      return;
    float limit = fric_coeff * contact_ref->lambda[0];
    for (int i = 0; i < 2; ++i) {
      lo[i] = -limit;
      hi[i] = limit;
    }
  }

  float compute_delta_lambda_row(int r, float /*dt*/) const {
    float k = effective_mass(r);
    if (k < 1e-10f)
      return 0.f;
    return -velocity_error(r) / k; // no bias for friction
  }

  bool is_inequality() const override { return true; }
  int degrees() const override { return 2; }

private:
  glm::vec3 u, w; // contact tangent vectors

  // Build orthonormal {u, w} perpendicular to n.
  // Choose reference axis away from n to avoid near-parallel cross products.
  void compute_tangent_basis(const glm::vec3 &n) {
    glm::vec3 ref = (std::abs(n.x) < 0.57f) ? glm::vec3(1, 0, 0) : glm::vec3(0, 1, 0);
    u = glm::normalize(glm::cross(n, ref));
    w = glm::cross(n, u); // already unit length (n⊥u, both unit)
  }

  void build_jacobian() override {
    // Row 0 – tangent u
    J[0].va = u;
    J[0].wa = glm::cross(r_a, u);
    J[0].vb = -u;
    J[0].wb = -glm::cross(r_b, u);

    // Row 1 – tangent w
    J[1].va = w;
    J[1].wa = glm::cross(r_a, w);
    J[1].vb = -w;
    J[1].wb = -glm::cross(r_b, w);
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Warm-start cache  –  persist normal impulses across frames
// ─────────────────────────────────────────────────────────────────────────────
struct CachedContact {
  glm::vec3 point;
  float normal_impulse;
  float friction_impulse[2];
};

struct ContactCache {
  RigidBody *a, *b;
  std::vector<CachedContact> points;
};

// ─────────────────────────────────────────────────────────────────────────────
// ConstraintSolver  –  PGS driver
// ─────────────────────────────────────────────────────────────────────────────
class ConstraintSolver {
public:
  std::vector<ContactCache> contact_cache;

  // ── Look up a cached normal impulse for warm-starting ────────────────────
  float find_cached_impulse(RigidBody *a, RigidBody *b, const glm::vec3 &pt) const {
    for (const auto &cache : contact_cache) {
      if (cache.a != a || cache.b != b)
        continue;
      for (const auto &cp : cache.points) {
        if (glm::length(cp.point - pt) < 0.02f)
          return cp.normal_impulse;
      }
    }
    return 0.f;
  }

  // ── Main solve ────────────────────────────────────────────────────────────
  void solve(std::vector<PersistentManifold> &contacts, float dt) {

    // Pre-count contact points so we can reserve without reallocating
    // (reallocating would invalidate pointers inside FrictionConstraint).
    size_t total_points = 0;
    for (const auto &m : contacts)
      total_points += m.num_points;

    std::vector<ContactConstraint> cc;
    std::vector<FrictionConstraint> fc;
    cc.reserve(total_points);
    fc.reserve(total_points);

    // ── 1. Build constraints + warm-start ─────────────────────────────────
    // FIX 3: Original code used per-manifold loop variable `i` as the index
    //         into cc[], so for the second manifold i reset to 0 and warm-start
    //         wrote into cc[0] and cc[1] again instead of cc[2] and cc[3].
    //         We use a single running index `ci` across all manifolds.
    {
      int ci = 0;
      for (auto &m : contacts) {
        for (int i = 0; i < m.num_points; ++i, ++ci) {
          cc.emplace_back(m.a, m.b, m.points[i]);

          // Warm-start: apply cached impulse immediately so the solver
          // starts from a physically plausible velocity state.
          // float cached = find_cached_impulse(m.a, m.b, m.points[i].point);
          float cached = m.accumulated_lambda[i] * 0.85f;
          m.accumulated_lambda[i] = cached;
          if (cached > 0.f) {
            cc[ci].lambda[0] = cached;
            cc[ci].apply_impulse(0, cached);
          }
        }
      }
    }

    // Build friction constraints AFTER all contact constraints are in place
    // (FrictionConstraint holds a raw pointer into cc[]).
    {
      int ci = 0;
      for (auto &m : contacts) {
        for (int i = 0; i < m.num_points; ++i, ++ci) {
          const auto &cp = m.points[i];
          glm::vec3 ra = cp.point - m.a->get_centre_of_mass();
          glm::vec3 rb = m.b ? (cp.point - m.b->get_centre_of_mass()) : glm::vec3(0.f);
          fc.emplace_back(m.a, m.b, cp.normal, ra, rb, &cc[ci]);
        }
      }
    }

    // ── 2. PGS velocity loop ──────────────────────────────────────────────
    for (int iter = 0; iter < NUM_ITERATIONS; ++iter) {
      float total_error = 0.f;

      // All contact normals first
      for (int i = 0; i < (int)cc.size(); ++i) {
        float delta = cc[i].compute_delta_lambda(dt);
        float old = cc[i].lambda[0];
        cc[i].lambda[0] = std::max(old + delta, 0.f);
        delta = cc[i].lambda[0] - old;
        if (std::abs(delta) > 1e-10f)
          cc[i].apply_impulse(0, delta);
        total_error += std::abs(cc[i].velocity_error(0));
      }

      // All friction after — Coulomb clamp uses fully-updated normal lambdas
      for (int i = 0; i < (int)fc.size(); ++i) {
        fc[i].update_coulomb_clamp();
        for (int r = 0; r < 2; ++r) {
          float delta = fc[i].compute_delta_lambda_row(r, dt);
          float old = fc[i].lambda[r];
          fc[i].lambda[r] = std::clamp(old + delta, fc[i].lo[r], fc[i].hi[r]);
          delta = fc[i].lambda[r] - old;
          if (std::abs(delta) > 1e-10f)
            fc[i].apply_impulse(r, delta);
        }
      }

      printf("iter %d total_err=%.6f\n", iter, total_error);
    }

    // ── 3. Split-impulse positional correction ────────────────────────────
    // Runs once, after the velocity loop.  Directly moves positions to
    // remove penetration WITHOUT injecting velocity (energy-safe).
    // De-duplicate so a body shared by multiple contacts isn't moved N times.
    std::unordered_set<RigidBody *> corrected;
    for (auto &c : cc) {
      c.apply_positional_correction();
      if (c.a && corrected.insert(c.a).second)
        c.a->update_transform();
      if (c.b && corrected.insert(c.b).second)
        c.b->update_transform();
    }

    for (int i = 0; i < (int)cc.size(); ++i) {
      auto &c = cc[i];
      printf("contact[%d]: pen=%.4f  λ=%.4f  JV=%.6f  body_a=%d  body_b=%s\n",
          i,
          c.penetration,
          c.lambda[0],
          c.velocity_error(0),
          c.a->id,
          c.b ? std::to_string(c.b->id).c_str() : "plane");
    }

    // ── 4. Update warm-start cache ────────────────────────────────────────
    contact_cache.clear();
    {
      int ci = 0;
      for (auto &pm : contacts) {
        for (int i = 0; i < pm.num_points; ++i, ++ci) {
          pm.accumulated_lambda[i] = cc[ci].lambda[0];
        }
      }
    }
  }
};

// ─────────────────────────────────────────────────────────────────────────────
// Joint stubs (not yet implemented)
// ─────────────────────────────────────────────────────────────────────────────
/*
class DistanceConstraint  : public Constraint<1> { ... };  // DOF 5, 1 row
class HingeConstraint     : public Constraint<5> { ... };  // DOF 1, 5 rows
class BallSocketConstraint: public Constraint<3> { ... };  // DOF 3, 3 rows
class SliderConstraint    : public Constraint<5> { ... };  // DOF 1, 5 rows
class WeldConstraint      : public Constraint<6> { ... };  // DOF 0, 6 rows
*/
