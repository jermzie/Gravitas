#pragma once

#include <glm/common.hpp>
#include <glm/geometric.hpp>
#include <glm/matrix.hpp>
#include <limits>
#include <numeric>

#include "CollisionGeometry.hpp"
#include "Config.hpp"
#include "ConvexMesh.hpp"
#include "ConvexMeshBuilder.hpp"
#include "Debugger.hpp"
#include "Plane.hpp"
#include "RigidBody.hpp"

/**
 * @brief Collision detection & contact generation between two convex rigid
 * bodies. Implements Gauss map optimized Separating Axis Theorem.
 */

// SAT is purely a test to check if two polygons/hedra are intersecting
// possible separation axes include:
// All face normals of poly A
// All face normals of poly B
// Cross products of all edge combos between A and B (3D only)

struct FaceColInfo {
  float separation = 0.0f;
  size_t face_index;

  FaceColInfo() = default;
  FaceColInfo(float separation, size_t face_index)
      : separation(separation), face_index(face_index) {}
};

struct FaceContact {
  const RigidBody *ref_body, *inc_body;
  const FaceColInfo *ref_face, *inc_face;
};

struct EdgeColInfo {
  float separation = 0.0f;
  std::pair<size_t, size_t> edge_indices;

  EdgeColInfo() = default;
  EdgeColInfo(float separation, std::pair<size_t, size_t> edge_indices)
      : separation(separation), edge_indices(edge_indices) {}
};

struct ContactID {
  uint64_t body_id;    // upper 32 bits: c2 body id, lower 32 bits: c1 body id
  uint64_t feature_id; // upper 32 bits: c2 feature, lower 32 bits: c1 feature
};

struct ContactPoint {

  glm::vec3 pos; // world-space contact point
  glm::vec3 norm;
  float pen_depth; // penetration depth

  float restitution;
  float kinetic_fric_coeff;
  float static_fric_coeff;

  ContactID id;
};

struct ContactManifold {
  ContactPoint points[4];
  size_t num_points;

  glm::vec3 norm; // contact normal from A -> B
  float max_pen_depth;

  RigidBody *a;
  RigidBody *b;
};

class Narrowphase {
public:
  bool poly_sphere_collision(ContactManifold &out, const RigidBody &poly);
  bool poly_plane_collision(ContactManifold &out, const RigidBody &poly,
                            const Plane &plane, Debugger *debug = nullptr);
  bool poly_poly_collision(ContactManifold &out, const RigidBody &poly_A,
                           const RigidBody &poly_B, Debugger *debug = nullptr);

private:
  FaceColInfo query_face_normals(const RigidBody &poly_A,
                                 const RigidBody &poly_B, glm::mat4 transform);
  EdgeColInfo query_edge_combos(const RigidBody &poly_A,
                                const RigidBody &poly_B, glm::mat4 matrix_A,
                                glm::mat4 matrix_B, glm::mat4 A_to_B,
                                glm::mat3 normal_A);

  ContactManifold create_face_contact(const FaceColInfo &fa,
                                      const FaceColInfo &fb,
                                      const RigidBody &poly_A,
                                      const RigidBody &poly_B);
  ContactManifold create_edge_contact(EdgeColInfo eab, const RigidBody &poly_A,
                                      const RigidBody &poly_B);
  ContactManifold create_plane_contact(const RigidBody &poly,
                                       const Plane &plane,
                                       Debugger *debug = nullptr);

  // FIXME: TOO COMPUTATIONAL EXPENSIVE
  // Normal transformations use different matrix from model matrix;
  static inline glm::mat3 get_normal_matrix(glm::mat4 model) {
    return glm::transpose(glm::inverse(glm::mat3(model)));
  }

  bool is_minkowski_face(const glm::vec3 &a, const glm::vec3 &b,
                         const glm::vec3 &c, const glm::vec3 &d);
  glm::vec3 find_support_point(glm::vec3 axis, const RigidBody &poly);

  std::vector<glm::vec3>
  clip_polygon_against_plane(const std::vector<glm::vec3> &polygon,
                             const Plane &plane);

  ConvexMesh::Face find_incident_face(const glm::vec3 &ref_norm_world,
                                      const RigidBody &poly);

  inline float get_signed_triangle_area(const glm::vec3 &a, const glm::vec3 &b,
                                        const glm::vec3 &c,
                                        const glm::vec3 &normal);

  std::vector<ContactPoint>
  reduce_contact_manifold(std::vector<ContactPoint> contacts, glm::vec3 normal);
};
