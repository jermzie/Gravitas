#pragma once

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/matrix.hpp>
#include <limits>
#include <numeric>

#include "CollisionGeometry.hpp"
#include "Config.hpp"
#include "Contacts.hpp"
#include "ConvexMesh.hpp"
#include "ConvexMeshBuilder.hpp"
#include "Debugger.hpp"
#include "Plane.hpp"
#include "RigidBody.hpp"

/**
 * @brief Collision detection & contact generation between two convex rigid
 * bodies. Implements Gauss map optimized Separating Axis Theorem.
 */

struct FaceColInfo {
  float separation = 0.0f;
  size_t face_idx;

  FaceColInfo() = default;
  FaceColInfo(float separation_dist, size_t face_index)
      : separation(separation_dist), face_idx(face_index) {}
};

struct EdgeColInfo {
  float separation = 0.0f;
  std::pair<size_t, size_t> edge_idx;

  EdgeColInfo() = default;
  EdgeColInfo(float separation_dist, std::pair<size_t, size_t> indices)
      : separation(separation_dist), edge_idx(indices) {}
};

struct FaceContactInfo {
  const RigidBody *ref_body, *inc_body;
  const size_t ref_face_idx;
};

class Narrowphase {
public:
  bool poly_sphere_collision(Manifold &out, const RigidBody &poly);
  bool poly_plane_collision(Manifold &out, const RigidBody &poly,
                            const Plane &plane, Debugger *debug = nullptr);
  bool poly_poly_collision(Manifold &out, const RigidBody &poly_A,
                           const RigidBody &poly_B, Debugger *debug = nullptr);

private:
  FaceColInfo test_face_normals(const RigidBody &poly_A,
                                const RigidBody &poly_B, glm::mat4 transform);
  EdgeColInfo test_edge_combos(const RigidBody &poly_A, const RigidBody &poly_B,
                               glm::mat4 A_to_B);

  Manifold create_face_contact(const RigidBody &poly_A, const RigidBody &poly_B,
                               const FaceColInfo &fa, const FaceColInfo &fb);
  Manifold create_edge_contact(const RigidBody &poly_A, const RigidBody &poly_B,
                               const EdgeColInfo &eab);
  Manifold create_plane_contact(const RigidBody &poly, const Plane &plane,
                                Debugger *debug = nullptr);

  bool is_minkowski_face(const glm::vec3 &a, const glm::vec3 &b,
                         const glm::vec3 &c, const glm::vec3 &d);
  glm::vec3 find_support_point(glm::vec3 axis, const RigidBody &poly);

  size_t find_incident_face(const glm::vec3 &ref_norm_world,
                            const RigidBody &poly);
  FaceContactInfo bias_reference_face(const RigidBody &a, const RigidBody &b,
                                      const FaceColInfo &fa,
                                      const FaceColInfo &fb);

  void clip_against_reference_face(std::vector<glm::vec3> &polygon,
                                   const ConvexMesh &ref_mesh,
                                   const glm::mat4 &ref_transform,
                                   const ConvexMesh::Face &ref_face,
                                   const Plane &ref_plane);

  std::vector<glm::vec3>
  clip_polygon_against_plane(const std::vector<glm::vec3> &polygon,
                             const Plane &plane);
  std::vector<ContactPoint> reduce_manifold(std::vector<ContactPoint> contacts,
                                            glm::vec3 ref_norm);

  inline float get_signed_triangle_area(const glm::vec3 &a, const glm::vec3 &b,
                                        const glm::vec3 &c,
                                        const glm::vec3 &normal);

  static inline glm::mat3 get_normal_matrix(glm::mat4 model) {
    return glm::transpose(glm::inverse(glm::mat3(model)));
  }

  // Packs two 32-bit IDs into a single uint64 (upper | lower).
  static inline uint64_t pack_id(uint32_t upper, uint32_t lower) {
    return (static_cast<uint64_t>(upper) << 32) | static_cast<uint64_t>(lower);
  }
  static constexpr float EPSILON = 1e-6f;
  static constexpr float REFERENCE_BIAS = 0.01f;
};
