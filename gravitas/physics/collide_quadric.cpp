#include "collide.hpp"

#include <cmath>

namespace {

using col::CollideProperties;

constexpr float EPSILON = 1e-6f;

} // namespace

namespace col {

CollideSphere sphere_cast(const CollideProperties &s) {
  assert(s.prim->get_type() == SPHERE);
  const auto *sphere = static_cast<const Sphere *>(s.prim);
  const Transform &local = sphere->get_local_xfrm();

  // transform to world-space (dynamic_xfrm + local_xfrm)
  const glm::vec3 centre =
      glm::vec3(s.xfrm->get_matrix() * glm::vec4(local.get_position(), 1.0f));

  // local_xfrm defines scale (rigid body cannot change scale)
  const float radius = sphere->get_radius() * std::abs(local.get_scale().x);

  return {centre, radius};
}

// centre distance vs sum of radii - single contact point
bool sphere2(Manifold &m, const CollideProperties &a,
             const CollideProperties &b) {

  const CollideSphere sa = sphere_cast(a);
  const CollideSphere sb = sphere_cast(b);

  const glm::vec3 d = sa.centre - sb.centre; // B -> A
  const float dist2 = glm::dot(d, d);
  const float rsum = sa.radius + sb.radius;

  // separated or just touching - no penetration to resolve
  if (dist2 >= rsum * rsum) {
    return false;
  }

  // coincident centres - any axis separates, pick +Y
  const float dist = std::sqrt(dist2);
  const glm::vec3 normal =
      dist > EPSILON ? d / dist : glm::vec3(0.0f, 1.0f, 0.0f);
  const float depth = rsum - dist;

  // midway between the deepest points on each surface
  const glm::vec3 a_surface = sa.centre - normal * sa.radius;
  const glm::vec3 b_surface = sb.centre + normal * sb.radius;

  set_single_contact(m, a, b, 0.5f * (a_surface + b_surface), normal, depth);
  return true;
}

// TODO
bool sphere_capsule(Manifold &m, const CollideProperties &a,
                    const CollideProperties &b) {
  return false;
}

// TODO
bool sphere_frustum(Manifold &m, const CollideProperties &a,
                    const CollideProperties &b) {
  return false;
}

bool capsule2(Manifold &m, const CollideProperties &a,
              const CollideProperties &b) {
  return false;
}

bool capsule_frustum(Manifold &m, const CollideProperties &a,
                     const CollideProperties &b) {
  return false;
}

// TODO
bool frustum2(Manifold &m, const CollideProperties &a,
              const CollideProperties &b) {
  return false;
}

} // namespace col
