#pragma once

#include "../inc/Debugger.hpp"
#include <glm/common.hpp>
#include <limits>
#include <numeric>

#include "../inc/Debugger.hpp"
#include "CollisionGeometry.hpp"
#include "ConvexMesh.hpp"
#include "ConvexMeshBuilder.hpp"
#include "RigidBody.hpp"

/**
 * @brief Collision detection & contact generation between two convex rigid bodies. Implements Gauss map optimized
 * Separating Axis Theorem.
 */

// SAT is purely a test to check if two polygons/hedra are intersecting
// possible separation axes include:
// All face normals of poly A
// All face normals of poly B
// Cross products of all edge combos between A and B (3D only)

// TODO: Finish contact generation
// TODO: Add collision cache
// TODO: Add transform cache (too many matrix computations every frame)

class Narrowphase {
public:
  struct CollisionInfo {
    int apid;
    int bpid;
    glm::vec3 axis;
    float penetration;
  };

  struct FaceCollision {
    float separation = 0.0f;
    size_t reference_face_index;

    FaceCollision() = default;
    FaceCollision(float distance, size_t face) : separation(distance), reference_face_index(face) {}
  };

  struct EdgeCollision {
    float separation = 0.0f;
    std::pair<size_t, size_t> edge_indices;

    EdgeCollision() = default;
    EdgeCollision(float distance, std::pair<size_t, size_t> edges) : separation(distance), edge_indices(edges) {}
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
  bool poly_poly_collision(const RigidBody &poly_A, const RigidBody &poly_B, Debugger *debug = nullptr) {
    CollisionInfo out{-1, -1, glm::vec3(0.0f), std::numeric_limits<float>::max()};

    // FIXME: Computing inverse matrix every frame
    // All computations in local space of hull B -- multiply A by inverse
    // transformations of B
    glm::mat4 model_B = poly_B.get_physics_matrix();
    glm::mat4 model_A = glm::inverse(model_B) * poly_A.get_physics_matrix();

    // Check all face normals of A -- O(n^2)
    FaceCollision fa = query_face_normals(poly_A, poly_B, model_A);
    std::cout << "polyA normals: " << fa.separation << std::endl;
    if (fa.separation > 0.0f) {
      return false;
    }

    // FIXME: Computing inverse matrix every frame
    // Check all face normals of B -- O(n^2)
    FaceCollision fb = query_face_normals(poly_B, poly_A, glm::inverse(model_A));
    std::cout << "polyB normals: " << fb.separation << std::endl;
    if (fb.separation > 0.0f) {
      return false;
    }

    // Check all edge combos between -- O(n^2)
    EdgeCollision eab = query_edge_combos(poly_A, poly_B);
    std::cout << "edge combos: " << eab.separation << std::endl;
    if (eab.separation > 0.0f) {
      return false;
    }

    // No separating axis found -- collision detected
    // assume all separation results are negative
    // axis of separation is axis w/ smallest separation distance (i.e. closest
    // to zero)
    /*
    bool contact_face_A = fa.separation > eab.separation;
    bool contact_face_B = fb.separation > eab.separation;
    if (contact_face_A && contact_face_B) {
      // std::cerr << "FACE COLLISION" << std::endl;
      ContactManifold out = create_face_contact(fa, fb, poly_A, poly_B);
      // std::cerr << "NUM CONTACTS: " << out.num_points << std::endl;

      if (debug) {
        auto transform_A = poly_A.get_physics_matrix();
        std::vector<glm::vec3> points;
        for (int i = 0; i < 4; i++) {

          points.push_back(glm::vec3(transform_A * glm::vec4(out.point[i].point, 1.0f)));
        }
        debug->draw_unordered_polygon(points, glm::vec3(1.0, 0.0, 0.0f));
      }

    } else {
      // std::cerr << "EDGE COLLISION" << std::endl;
      // ContactManifold out = create_edge_contact(eab, poly_A, poly_B);
      // std::cerr << "NUM CONTACTS: " << out.num_points << std::endl;

      if (debug) {
        auto mesh_A = poly_A.get_mesh();
        auto mesh_B = poly_B.get_mesh();
        auto transform_A = poly_A.get_physics_matrix();
        auto transform_B = poly_B.get_physics_matrix();
        auto e1 = mesh_A.get_half_edge_vertices(mesh_A.half_edges[eab.edge_indices.first]);
        auto e2 = mesh_B.get_half_edge_vertices(mesh_B.half_edges[eab.edge_indices.second]);
        glm::vec3 v11 = glm::vec3(transform_A * glm::vec4(mesh_A.vertices[e1[0]], 1.0f));
        glm::vec3 v12 = glm::vec3(transform_A * glm::vec4(mesh_A.vertices[e1[1]], 1.0f));
        glm::vec3 v21 = glm::vec3(transform_B * glm::vec4(mesh_B.vertices[e2[0]], 1.0f));
        glm::vec3 v22 = glm::vec3(transform_B * glm::vec4(mesh_B.vertices[e2[1]], 1.0f));

        debug->draw_line(v11, v12, glm::vec3(1.0, 0.0, 0.0));
        debug->draw_line(v21, v22, glm::vec3(1.0, 0.0, 0.0));
        debug->draw_vertex(out.point[0].point, glm::vec3(1.0, 0.0, 0.0));
      }
    }
    */

    return true;
  }

private:
  FaceCollision query_face_normals(const RigidBody &poly_A, const RigidBody &poly_B, glm::mat4 transform) {

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

    return FaceCollision(max_distance, max_index);
  }

  EdgeCollision query_edge_combos(const RigidBody &poly_A, const RigidBody &poly_B) {

    glm::mat4 model_B = poly_B.get_physics_matrix();
    glm::mat4 model_A = poly_A.get_physics_matrix();
    glm::mat4 transform = glm::inverse(model_B) * model_A;

    // BUG:
    // transform world centroid pos. (getCentroid()) into B's local space
    glm::vec3 centroid_A = glm::vec3(transform * glm::vec4(poly_A.get_local_geometric_centroid(), 1.0f));
    glm::vec3 centroid_B = poly_B.get_local_geometric_centroid();

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

      const auto he_A = mesh_A.half_edges[i];
      visited_A[i] = true;
      visited_A[he_A.twin] = true;

      // get indices to halfedge vertices
      const auto he_vertices_A = mesh_A.get_half_edge_vertices(he_A);
      const auto vertices_A = mesh_A.vertices;

      // transform A's vertices into B's local space
      glm::vec3 p1 = glm::vec3(transform * glm::vec4(vertices_A[he_vertices_A[0]], 1.0f));
      glm::vec3 q1 = glm::vec3(transform * glm::vec4(vertices_A[he_vertices_A[1]], 1.0f));
      glm::vec3 edge_A = q1 - p1;

      // get face normals for edge A in B's local space
      glm::vec3 local_norm_A = mesh_A.faces[he_A.face].plane.normal;
      glm::vec3 local_norm_B = mesh_A.faces[mesh_A.half_edges[he_A.twin].face].plane.normal;

      glm::vec3 normal_A = glm::normalize(normal_matrix_A * local_norm_A);
      glm::vec3 normal_B = glm::normalize(normal_matrix_A * local_norm_B);

      std::vector<bool> visited_B(edge_count_B, false);
      for (size_t j = 0; j < edge_count_B; j++) {
        // skip twin half-edge
        if (visited_B[j]) {
          continue;
        }

        const auto he_B = mesh_B.half_edges[j];
        visited_B[j] = true;
        visited_B[he_B.twin] = true;

        // get indices to edge vertices
        const auto he_vertices_B = mesh_B.get_half_edge_vertices(he_B);
        const auto vertices_B = mesh_B.vertices;

        glm::vec3 p2 = vertices_B[he_vertices_B[0]];
        glm::vec3 q2 = vertices_B[he_vertices_B[1]];
        glm::vec3 edge_B = q2 - p2;

        // get face normals for edge B in B's local space
        glm::vec3 normal_C = glm::normalize(mesh_B.faces[he_B.face].plane.normal);
        glm::vec3 normal_D = glm::normalize(mesh_B.faces[mesh_B.half_edges[he_B.twin].face].plane.normal);

        // check if edges form Minkowski face
        // negate hull B's normals to account for Minkowski difference
        if (is_minkowski_face(normal_A, normal_B, -normal_C, -normal_D)) {

          // compute edge separation

          // skip near-parallel edges
          glm::vec3 cross = glm::cross(edge_A, edge_B);
          float cross_len = glm::length(cross);

          if (cross_len > 0.005f * glm::sqrt(glm::length2(edge_A) * glm::length2(edge_B))) {

            // compute separating axis
            glm::vec3 axis = cross / cross_len;

            // ensure axis points from A to B
            if (glm::dot(axis, centroid_B - centroid_A) < 0.0f) {
              axis = -axis;
            }

            auto proj = [](const glm::vec3 &v, const glm::vec3 &axis) { return glm::dot(axis, v); };

            // projections of edge A in B-local space (we already had p1 and
            // q1)
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
            // float separation = std::max(min_B - max_A, min_A - max_B);
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

    return EdgeCollision(max_distance, max_index);
  }

  // use SutherlandHodgeman clipping to fin contact area????
  // we alr have face idx of colliding face
  ContactManifold create_face_contact(
      const FaceCollision &fab, const FaceCollision &fb, const RigidBody &poly_A, const RigidBody &poly_B) {

    ContactManifold out;
    out.num_points = 0;
    out.max_penetration = -std::numeric_limits<float>::max();

    ConvexMesh mesh_A = poly_A.get_mesh();
    ConvexMesh mesh_B = poly_B.get_mesh();

    // FIXME:
    // NEED TO BIAS ONE POLY OVER THE OTHER (CONSISTENT CONTACTS FOR COHERENCE)
    // BIAS ONE POLY AS REFERENCE
    ConvexMesh::Face reference_face = mesh_A.faces[fab.reference_face_index];
    ConvexMesh::Face incident_face =
        find_incident_face(reference_face.plane.normal, poly_B, poly_B.get_physics_matrix());

    // Get world-space incident face vertices
    glm::mat4 transform_B = poly_B.get_physics_matrix();
    std::vector<glm::vec3> incident_polygon;
    auto incident_vertices = mesh_B.get_face_vertices(incident_face);
    for (size_t index : incident_vertices) {
      incident_polygon.push_back(glm::vec3(transform_B * glm::vec4(mesh_B.vertices[index], 1.0f)));
    }

    // Get world-space reference face side planes
    glm::mat4 transform_A = poly_A.get_physics_matrix();
    glm::mat3 normal_matrix_A = get_normal_matrix(transform_A);
    Plane reference_plane = reference_face.plane;
    reference_plane.normal = glm::normalize(normal_matrix_A * reference_plane.normal);
    reference_plane.point = glm::vec3(transform_A * glm::vec4(reference_plane.point, 1.0f));
    reference_plane.distance = -glm::dot(reference_plane.normal, reference_plane.point);

    auto reference_half_edges = mesh_A.get_face_half_edges(reference_face);
    for (size_t index : reference_half_edges) {
      auto &he = mesh_A.half_edges[index];
      auto he_vertices = mesh_A.get_half_edge_vertices(he);

      glm::vec3 v0 = glm::vec3(transform_A * glm::vec4(mesh_A.vertices[he_vertices[0]], 1.0f));
      glm::vec3 v1 = glm::vec3(transform_A * glm::vec4(mesh_A.vertices[he_vertices[1]], 1.0f));

      // Perhaps flip if side normal is pointing inwards?
      glm::vec3 edge_direction = glm::normalize(v1 - v0);
      glm::vec3 side_normal = glm::cross(edge_direction, reference_face.plane.normal);

      // Side planes must be perpendicular to refernce face?
      Plane side_plane(side_normal, v0);

      incident_polygon = clip_polygon_against_plane(incident_polygon, side_plane);
    }

    std::vector<ContactPoint> all_contacts;

    for (const auto &point : incident_polygon) {
      float penetration = -get_signed_distance_to_plane(point, reference_plane);

      // Keep points that are penetrating (below the reference plane)
      if (penetration >= -1e-6f) { // Small epsilon for numerical tolerance
        ContactPoint contact;
        contact.point = point;
        contact.normal = reference_plane.normal;
        contact.penetration = penetration;
        contact.contact_id = all_contacts.size();

        all_contacts.push_back(contact);

        // Track maximum penetration
        if (penetration > out.max_penetration) {
          out.max_penetration = penetration;
        }
      }
    }

    std::vector<ContactPoint> reduced_contacts = reduce_contact_manifold(all_contacts, reference_plane.normal);

    out.normal = reference_plane.normal;
    out.num_points = std::min(4, (int)reduced_contacts.size());

    for (int i = 0; i < out.num_points; ++i) {
      out.point[i] = reduced_contacts[i];
    }

    return out;
  }

  // find closest points on two colliding edges (midpoint????)
  ContactManifold create_edge_contact(EdgeCollision eab, const RigidBody &poly_A, const RigidBody &poly_B) {

    ContactManifold out;
    out.num_points = 1;
    out.normal = glm::vec3(0.0f);

    ConvexMesh mesh_A = poly_A.get_mesh();
    ConvexMesh mesh_B = poly_B.get_mesh();

    auto &half_edge_A = mesh_A.half_edges[eab.edge_indices.first];
    auto &half_edge_B = mesh_B.half_edges[eab.edge_indices.second];

    glm::mat4 transform_A = poly_A.get_physics_matrix();
    glm::mat4 transform_B = poly_B.get_physics_matrix();

    auto he_indices_A = mesh_A.get_half_edge_vertices(half_edge_A);
    auto he_indices_B = mesh_B.get_half_edge_vertices(half_edge_B);

    // parametric equation for edges
    // p1 + (p2 - p1) * s
    // q1 + (q2 - q1) * t

    // transform into world space
    glm::vec3 p1 = glm::vec3(transform_A * glm::vec4(mesh_A.vertices[he_indices_A[0]], 1.0f));
    glm::vec3 q1 = glm::vec3(transform_A * glm::vec4(mesh_A.vertices[he_indices_A[1]], 1.0f));
    glm::vec3 p2 = glm::vec3(transform_B * glm::vec4(mesh_B.vertices[he_indices_B[0]], 1.0f));
    glm::vec3 q2 = glm::vec3(transform_B * glm::vec4(mesh_B.vertices[he_indices_B[1]], 1.0f));

    // glm::vec3 edge_A = p2 - p1;
    // glm::vec3 edge_B = q2 - q1;

    glm::vec3 edge_A = q1 - p1;
    glm::vec3 edge_B = q2 - p2;

    // closest points between edges occur
    // when line segment is perpendicular to both edges
    glm::vec3 w0 = p1 - p2;
    float a = glm::dot(edge_A, edge_A);
    float b = glm::dot(edge_B, edge_B);
    float a_b = glm::dot(edge_A, edge_B);
    float a_w0 = glm::dot(edge_A, w0);
    float b_w0 = glm::dot(edge_B, w0);
    float denom = a * b - a_b * a_b;
    /*
    glm::vec3 w0 = q1 - p1;
    float a = glm::dot(edge_A, edge_A);
    float b = glm::dot(edge_B, edge_B);
    float a_b = glm::dot(edge_A, edge_B);
    float a_w0 = glm::dot(edge_A, w0);
    float b_w0 = glm::dot(edge_B, w0);
    float denom = a_b * a_b - a * b;
    */

    float s, t;
    const float epsilon = 1e-6f;
    if (std::abs(denom) < epsilon) {
      // Use midpoint of one edge
      out.point[0].point = 0.5f * (p1 + q1);
      // s = 0.0f;
      //  t = (b > epsilon) ? glm::clamp(b_w0 / b, 0.0f, 1.0f) : 0.0f;
    } else {

      s = (a_b * b_w0 - b * a_w0) / denom;
      t = (a * b_w0 - a_b * a_w0) / denom;

      // Clamp to edge segments
      s = glm::clamp(s, 0.0f, 1.0f);
      t = glm::clamp(t, 0.0f, 1.0f);

      glm::vec3 closest_A = p1 + edge_A * s;
      glm::vec3 closest_B = p2 + edge_B * t;

      /*
      // Closest points on the infinite lines
      float s = (a_w0 * a_b - a * b_w0) / denom;
      float t = (-b_w0 * a_b + b * a_w0) / denom;

      glm::vec3 closest_p = p1 + edge_A * s;
      glm::vec3 closest_q = q1 + edge_B * t;

      // Edge contact is midpoint between two closet points
      glm::vec3 edge_contact = 0.5 * (closest_q - closest_p);

      */

      // Midpoint between closest points
      out.point[0].point = 0.5f * (closest_A + closest_B);

      // Contact normal points from A to B
      glm::vec3 contact_vector = closest_B - closest_A;
      float distance = glm::length(contact_vector);

      glm::vec3 axis;
      if (distance > epsilon) {
        axis = contact_vector / distance;
      } else {
        // Edges are essentially touching - use cross product
        axis = glm::normalize(glm::cross(edge_A, edge_B));
      }

      // WARNING: POSSIBLE ISSUE WITH WORLD SPACE
      glm::vec3 centroid_A = poly_A.get_centre_of_mass();
      glm::vec3 centroid_B = poly_B.get_centre_of_mass();

      // Normal should point from A to B
      if (glm::dot(axis, centroid_B - centroid_A) < 0.0f) {
        axis = -axis;
      }

      out.normal = axis;

      // Penetration is negative of distance (since we're interpenetrating)
      out.point[0].penetration = -distance;
      out.max_penetration = -distance;
    }

    out.point[0].normal = out.normal;
    out.point[0].contact_id = 0;

    return out;
  }

  // FIXME: TOO COMPUTATIONAL EXPENSIVE
  // Normal transformations use different matrix from model matrix;
  static inline glm::mat3 get_normal_matrix(glm::mat4 model) { return glm::transpose(glm::inverse(glm::mat3(model))); }

  bool is_minkowski_face(const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c, const glm::vec3 &d) {
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

  std::vector<glm::vec3> clip_polygon_against_plane(const std::vector<glm::vec3> &polygon, const Plane &plane) {
    std::vector<glm::vec3> out;

    if (polygon.empty())
      return out;

    size_t vertex_count = polygon.size();
    for (size_t i = 0; i < vertex_count; i++) {
      const glm::vec3 &first = polygon[i];
      const glm::vec3 &second = polygon[(i + 1) % vertex_count];

      float first_distance = get_signed_distance_to_plane(first, plane);
      float second_distance = get_signed_distance_to_plane(second, plane);

      // Both inside
      if (first_distance <= 0 && second_distance <= 0) {
        out.push_back(second);
      }

      // First outside, second inside
      else if (first_distance > 0 && second_distance <= 0) {
        // Plane-line intersection
        float t = first_distance / (first_distance - second_distance);
        glm::vec3 intersection = first + (second - first) * t;

        out.push_back(intersection);
      }

      // First inside, second outside
      else if (first_distance <= 0 && second_distance > 0) {
        // Plane-line intersection
        float t = first_distance / (first_distance - second_distance);
        glm::vec3 intersection = first + (second - first) * t;

        out.push_back(intersection);
        out.push_back(second);
      }

      // Both outside -- don't add anything
    }

    return out;
  }

  /**
   * @brief Finds face most anti-parallel to reference face
   * @return incident face
   */
  ConvexMesh::Face find_incident_face(const glm::vec3 normal, const RigidBody &poly) {
    ConvexMesh::Face incident_face;
    int max = std::numeric_limits<int>::max();
    for (auto face : poly.get_mesh().faces) {
      int angle = glm::dot(normal, face.plane.normal);
      if (angle < max) {
        incident_face = face;
      }
    }
    return incident_face;
  }

  ConvexMesh::Face find_incident_face(
      const glm::vec3 &reference_normal_world, const RigidBody &poly, const glm::mat4 &poly_transform) {

    // Transform reference normal into poly's local space
    glm::mat3 inv_normal_matrix = glm::mat3(poly_transform) * glm::transpose(glm::inverse(glm::mat3(poly_transform)));
    glm::vec3 reference_normal_local = glm::normalize(inv_normal_matrix * reference_normal_world);

    ConvexMesh::Face incident_face;
    float min_dot = std::numeric_limits<float>::max();

    for (const auto &face : poly.get_mesh().faces) {
      float dot = glm::dot(reference_normal_local, face.plane.normal);
      if (dot < min_dot) {
        min_dot = dot;
        incident_face = face;
      }
    }

    return incident_face;
  }

  inline float get_signed_triangle_area(
      const glm::vec3 &a, const glm::vec3 &b, const glm::vec3 &c, const glm::vec3 &normal) {
    glm::vec3 ab = b - a;
    glm::vec3 ac = c - a;
    glm::vec3 cross = glm::cross(ab, ac);
    return glm::dot(cross, normal);
  }

  std::vector<ContactPoint> reduce_contact_manifold(std::vector<ContactPoint> contacts, glm::vec3 normal) {

    // No reduction required
    if (contacts.size() <= 4) {
      return contacts;
    }

    std::vector<ContactPoint> out;

    // 1. find deepest support point
    int deepest_index = 0;
    float max_penetration = contacts[0].penetration;

    for (int i = 1; i < contacts.size(); i++) {
      if (contacts[i].penetration > max_penetration) {
        max_penetration = contacts[i].penetration;
        deepest_index = i;
      }
    }

    out.push_back(contacts[deepest_index]);

    // 2. furthest point from deepest
    int max_index = -1;
    float max_distance = -std::numeric_limits<float>::max();

    for (int i = 0; i < contacts.size(); ++i) {
      if (i == deepest_index) {
        continue;
      }

      float distance = glm::distance2(contacts[i].point, contacts[deepest_index].point);
      if (distance > max_distance) {
        max_distance = distance;
        max_index = i;
      }
    }

    // edge case -- all points are same
    if (max_index == -1 || max_distance < 1e-6f) {
      return out;
    }

    out.push_back(contacts[max_index]);

    // 3. point that maximizes triangle area
    int third_index = -1;
    float max_area = -std::numeric_limits<float>::max();
    for (int i = 0; i < contacts.size(); ++i) {
      if (i == deepest_index || i == max_index) {
        continue;
      }

      float area =
          get_signed_triangle_area(contacts[deepest_index].point, contacts[max_index].point, contacts[i].point, normal);

      if (area > max_area) {
        max_area = area;
        third_index = i;
      }
    }

    // edge case -- all points are collinear
    if (third_index == -1 || std::abs(max_area) < 1e-6f) {
      return out;
    }

    out.push_back((contacts[third_index]));

    // 4. point that maximizes total area
    // test
    int fourth_index = -1;
    float max_negative_area = 0.0f;

    for (int i = 0; i < contacts.size(); ++i) {

      // Edge 0: deepest -> farthest
      float area0 =
          get_signed_triangle_area(contacts[deepest_index].point, contacts[max_index].point, contacts[i].point, normal);

      // Edge 1: farthest -> third
      float area1 =
          get_signed_triangle_area(contacts[max_index].point, contacts[third_index].point, contacts[i].point, normal);

      // Edge 2: third -> deepest
      float area2 = get_signed_triangle_area(
          contacts[third_index].point, contacts[deepest_index].point, contacts[i].point, normal);

      // Keep the most negative area across all three edges
      float min_area = std::min({area0, area1, area2});

      if (min_area < max_negative_area) {
        max_negative_area = min_area;
        fourth_index = i;
      }
    }

    if (fourth_index != -1) {
      out.push_back(contacts[fourth_index]);
    }

    return out;
  }
};
