#pragma once

// NEEDS TESTING
//
// #include "../inc/Debugger.hpp"
#include "CollisionGeometry.hpp"
#include "ConvexMesh.hpp"
#include "ConvexMeshBuilder.hpp"
#include "RigidBody.hpp"

// SAT is purely a test to check if two polygons/hedra are intersecting
//

// possible separation axes include:
// All face normals of poly A
// All face normals of poly B
// Cross products of all edge combos between A and B (3D only)

class Narrowphase {
public:
  struct CollisionInfo {

    int apid;
    int bpid;
    glm::vec3 axis;
    float penetration;
  };

  struct FaceColInfo {

    float separation = 0.0f;
    size_t face_index;

    FaceColInfo() = default;
    FaceColInfo(float distance, size_t face)
        : separation(distance), face_index(face) {}
  };

  struct EdgeColInfo {

    float separation = 0.0f;
    std::pair<size_t, size_t> edge_indices;

    EdgeColInfo() = default;
    EdgeColInfo(float distance, std::pair<size_t, size_t> edges)
        : separation(distance), edge_indices(edges) {}
  };

  struct ContactPoint {

    glm::vec3 point; // world-space contact point
    glm::vec3 normal;
    float penetration; // penetration depth
    int contact_id;

    /*
    float restitution;			// elasticity coefficient?
    float kinFricCoeff;
    float statFricCoeff;
    */
  };

  struct ContactManifold {

    int num_points;
    ContactPoint point[4];
    glm::vec3 normal; // contact normal from A -> B
    float max_penetration;
  };

  // Contact Creation
  // Need to identify axis of minimum penetration (ie. smallest penetration?)
  // choose between face A/B normals or edge-edge cross product

  // O(n^2)
  // Gauss Map Optimization
  bool poly_poly_collision(const RigidBody &poly_A, const RigidBody &poly_B) {

    CollisionInfo out{-1, -1, glm::vec3(0.0f),
                      std::numeric_limits<float>::max()};

    // All computations in local space of hull B -- multiply A by inverse
    // transformations of B
    glm::mat4 model_B = poly_B.get_matrix();
    glm::mat4 model_A = glm::inverse(model_B) * poly_A.get_matrix();

    // Check all face normals of A -- O(n^2)
    FaceColInfo fa = query_face_normals(poly_A, poly_B, model_A);
    // std::cout << "polyA normals: " << fa.separation << std::endl;
    if (fa.separation > 0.0f) {

      return false;
    }

    // Check all face normals of B -- O(n^2)
    FaceColInfo fb = query_face_normals(poly_B, poly_A, glm::inverse(model_A));
    // std::cout << "polyB normals: " << fb.separation << std::endl;
    if (fb.separation > 0.0f) {

      return false;
    }

    // Check all edge combos between -- O(n^2)
    EdgeColInfo eab = query_edge_combos(poly_A, poly_B);
    // std::cout << "edge combos: " << eab.separation << std::endl;

    if (eab.separation > 0.0f) {

      return false;
    }

    // No separating axis found -- collision detected
    // assume all separation results are negative
    // axis of separation is axis w/ smallest separation distance (i.e. closest
    // to zero)
    bool isFaceContactA = fa.separation > eab.separation;
    bool isFaceContactB = fb.separation > eab.separation;
    if (isFaceContactA && isFaceContactB) {
      // createFaceContact();
    } else {
      create_edge_contact();
    }

    return true;
  }

  // ??? NOT DONE
  ConvexMeshBuilder::Face find_incident_face(const glm::vec3 normal,
                                             const RigidBody &poly) {

    ConvexMeshBuilder::Face incident;
    int max = std::numeric_limits<int>::max();
    for (auto face : poly.get_mesh().faces) {
      int angle = glm::dot(normal, face.plane.normal);
      if (angle < max) {
        incident.plane = face.plane;
      }
    }

    return incident;
  }

  /*
  // use SutherlandHodgeman clipping to fin contact area????
  // we alr have face idx of colliding face
  ContactManifold createFaceContact(const FaceColInfo &collision_info, const
  ConvexHull &polyA, const ConvexHull &polyB) { MeshBuilder::Face
  reference_face; MeshBuilder::Face = find_incident_face();

    // how to find face side planes?

  }
  */

  // find closest points on two colliding edges (midpoint????)
  void create_edge_contact() {}

  FaceColInfo query_face_normals(const RigidBody &poly_A,
                                 const RigidBody &poly_B, glm::mat4 transform) {

    float max_distance = -std::numeric_limits<float>::max();
    size_t max_index;

    const size_t face_count = poly_A.get_mesh().faces.size();
    for (size_t i = 0; i < face_count; i++) {

      const ConvexMesh::Face face = poly_A.get_mesh().faces[i];
      Plane plane = face.plane;

      // Transform normal into hull B's local space
      glm::mat3 normal_matrix = get_normal_matrix(transform);
      glm::vec3 axis = normal_matrix * -plane.normal;

      // Find furthest vertex in direction opposite to normal
      glm::vec3 vertex = find_support_point(axis, poly_B);

      // Compute separation distance between vertex and face plane
      plane.normal = normal_matrix * plane.normal;
      plane.point = glm::vec3(transform * glm::vec4(plane.point, 1.0f));
      plane.distance = -glm::dot(plane.normal, plane.point);

      float dist = get_signed_distance_to_plane(vertex, plane);

      if (dist > max_distance) {
        max_distance = dist;
        max_index = i;
      }
    }

    return FaceColInfo(max_distance, max_index);
  }

  EdgeColInfo query_edge_combos(const RigidBody &poly_A,
                                const RigidBody &poly_B) {

    glm::mat4 modelB = poly_B.get_matrix();
    glm::mat4 modelA = poly_A.get_matrix();
    glm::mat4 transform = glm::inverse(modelB) * modelA;

    // transform world centroid pos. (getCentroid()) into B's local space
    //
    // MAY FAIL
    // .POSTION MAY BE GEOMETRIC CENTER OR COM
    glm::vec3 centroid_A =
        glm::vec3(transform * glm::vec4(poly_A.get_geometric_centriod(), 1.0f));
    glm::vec3 centroid_B = poly_B.get_centre_of_mass();

    ConvexMesh mesh_A = poly_A.get_mesh();
    ConvexMesh mesh_B = poly_B.get_mesh();

    size_t edge_count_A = mesh_A.half_edges.size();
    size_t edge_count_B = mesh_B.half_edges.size();

    float max_distance = -std::numeric_limits<float>::max();
    std::pair<size_t, size_t> max_index;

    glm::mat3 normal_matrix_A = get_normal_matrix(transform);

    std::vector<bool> visited_A(edge_count_A, false);
    for (size_t i = 0; i < edge_count_A; i++) {

      // skip twin half-edge
      if (visited_A[i]) {
        continue;
      }

      const auto half_edge_A = mesh_A.half_edges[i];
      visited_A[i] = true;
      visited_A[half_edge_A.twin] = true;

      // get indices to halfedge vertices
      const auto he_indices_A = mesh_A.get_half_edge_vertices(half_edge_A);
      const auto vertices_A = mesh_A.vertices;

      // transform A's vertices into B's local space
      glm::vec3 p1 =
          glm::vec3(transform * glm::vec4(vertices_A[he_indices_A[0]], 1.0f));
      glm::vec3 q1 =
          glm::vec3(transform * glm::vec4(vertices_A[he_indices_A[1]], 1.0f));
      glm::vec3 edgeA = q1 - p1;

      // get face normals for edge A in B's local space
      glm::vec3 local_norm_A = mesh_A.faces[half_edge_A.face].plane.normal;
      glm::vec3 local_norm_B =
          mesh_A.faces[mesh_A.half_edges[half_edge_A.twin].face].plane.normal;
      glm::vec3 normal_A = glm::normalize(normal_matrix_A * local_norm_A);
      glm::vec3 normal_B = glm::normalize(normal_matrix_A * local_norm_B);

      std::vector<bool> visited_B(edge_count_B, false);
      for (size_t j = 0; j < edge_count_B; j++) {

        // skip twin half-edge
        if (visited_B[j]) {
          continue;
        }

        const auto half_edge_B = mesh_B.half_edges[j];
        visited_B[j] = true;
        visited_B[half_edge_B.twin] = true;

        // get indices to edge vertices
        const auto he_indices_B = mesh_B.get_half_edge_vertices(half_edge_B);
        const auto vertices_B = mesh_B.vertices;

        glm::vec3 p2 = vertices_B[he_indices_B[0]];
        glm::vec3 q2 = vertices_B[he_indices_B[1]];
        glm::vec3 edge_B = q2 - p2;

        // get face normals for edge B in B's local space
        glm::vec3 normal_C =
            glm::normalize(mesh_B.faces[half_edge_B.face].plane.normal);
        glm::vec3 normal_D = glm::normalize(
            mesh_B.faces[mesh_B.half_edges[half_edge_B.twin].face]
                .plane.normal);

        // check if edges form Minkowski face
        // negate hull B's normals to account for Minkowski difference
        if (is_minkowski_face(normal_A, normal_B, -normal_C, -normal_D)) {

          // skip near-parallel edges
          glm::vec3 cross = glm::cross(edgeA, edge_B);
          float cross_len = glm::length(cross);

          if (cross_len >
              0.005f * glm::sqrt(glm::length2(edgeA) * glm::length2(edge_B))) {

            // compute separating axis
            glm::vec3 axis = cross / cross_len;

            // ensure axis points from A to B
            if (glm::dot(axis, centroid_B - centroid_A) < 0.0f) {
              axis = -axis;
            }

            /*
            // Compute distance between edges along the axis
            float separation = glm::dot(axis, p2 - p1);

            if (separation > maxDist) {
                    maxDist = separation;
                    maxIdx = { i, j };
            }
            */

            auto proj = [](const glm::vec3 &v, const glm::vec3 &axis) {
              return glm::dot(axis, v);
            };

            // projections of edge A in B-local space (we already had p1 and q1)
            float a1 = proj(p1, axis);
            float a2 = proj(q1, axis);
            float min_A = std::min(a1, a2);
            float max_A = std::max(a1, a2);

            // projections of edge B (p2, q2 are in B-local space)
            float b1 = proj(p2, axis);
            float b2 = proj(q2, axis);
            float min_B = std::min(b1, b2);
            float max_B = std::max(b1, b2);

            // separation = distance between intervals (positive = separated)
            // float separation = std::max(minB - maxA, minA - maxB);
            float forward_pen = max_A - min_B;
            float reverse_pen = max_B - min_A;

            float separation = std::min(forward_pen, reverse_pen);

            // tiny epsilon to avoid numeric noise
            if (separation > max_distance + 1e-6f) {
              max_distance = separation;
              max_index = {i, j};
            }
          }
        }
      }
    }

    return EdgeColInfo(max_distance, max_index);
  }

  // Normal transformations use different matrix from model matrix;
  static inline glm::mat3 get_normal_matrix(glm::mat4 model) {
    return glm::transpose(glm::inverse(glm::mat3(model)));
  }

  bool is_minkowski_face(const glm::vec3 &a, const glm::vec3 &b,
                         const glm::vec3 &c, const glm::vec3 &d) {

    // recall vertices are simply normals of adjacent faces for both edges
    // test if arcs AB and CD intersect on unit sphere

    // planes through arcs AB and CD respectively
    glm::vec3 BxA = glm::cross(b, a);
    glm::vec3 DxC = glm::cross(d, c);

    // find signed distance to plane
    float CBA = glm::dot(c, BxA);
    float DBA = glm::dot(d, BxA);
    float ADC = glm::dot(a, DxC);
    float BDC = glm::dot(b, DxC);

    // overlap test -- if vertices separated by plane, sign is negative
    // hemisphere test (edge case) -- overlap test fails if arcs on different
    // hemispheres
    return CBA * DBA < 0.0f && ADC * BDC < 0.0f && CBA * BDC > 0.0f;
  }

  glm::vec3 find_support_point(glm::vec3 axis, const RigidBody &poly) {

    auto local_vertices = poly.get_mesh().vertices;

    float max_projection = -std::numeric_limits<float>::max();
    glm::vec3 max_vertex;
    for (size_t i = 0; i < local_vertices.size(); i++) {

      // glm::vec3 worldVert = glm::vec3(model *
      // glm::vec4(localVerts[i], 1.0f));
      float projection = glm::dot(local_vertices[i], axis);

      if (projection > max_projection) {

        max_projection = projection;
        max_vertex = local_vertices[i];
      }
    }

    return max_vertex;
  }
};
