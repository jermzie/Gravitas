#include "Narrowphase.hpp"
#include "ConvexMesh.hpp"
#include "RigidBody.hpp"
#include <algorithm>
#include <cstdint>
#include <glm/ext/quaternion_geometric.hpp>
#include <glm/geometric.hpp>
#include <limits>

bool Narrowphase::poly_plane_collision(Manifold &out, const RigidBody &poly,
                                       const Plane &plane, Debugger *debug) {

  glm::vec3 axis_world = -plane.normal;
  glm::vec3 axis_local = poly.transform.get_normal_matrix() * axis_world;
  glm::vec3 vertex_local = find_support_point(axis_local, poly);
  glm::vec3 vertex_world =
      glm::vec3(poly.get_physics_matrix() * glm::vec4(vertex_local, 1.0f));

  float separation = get_signed_distance_to_plane(vertex_world, plane);
  if (separation > 0.0f) {
    return false;
  }

  out = create_plane_contact(poly, plane);

  if (debug) {
    for (int i = 0; i < out.num_points; ++i) {
      debug->draw_vertex(out.contacts[i].pos_world, glm::vec4(0, 1, 0, 1));
    }
  }

  return true;
}

// O(n^2)
// Gauss Map Optimization
bool Narrowphase::poly_poly_collision(Manifold &out, const RigidBody &poly_A,
                                      const RigidBody &poly_B,
                                      Debugger *debug) {

  const glm::mat4 matrix_A = poly_A.get_physics_matrix();
  const glm::mat4 matrix_B = poly_B.get_physics_matrix();
  const glm::mat4 inverse_A = poly_A.transform.get_inverse_matrix();
  const glm::mat4 inverse_B = poly_B.transform.get_inverse_matrix();

  // perform tests in other model's local space
  const glm::mat4 A_to_B = inverse_B * matrix_A;
  const glm::mat4 B_to_A = inverse_A * matrix_B;

  FaceColInfo fa = test_face_normals(poly_A, poly_B, A_to_B);
  // CLOGI("Poly A Normals: %.5f", fa.separation);
  if (fa.separation > 0.0f) {
    return false;
  }

  FaceColInfo fb = test_face_normals(poly_B, poly_A, B_to_A);
  // CLOGI("Poly B Normals: %.5f", fb.separation);
  if (fb.separation > 0.0f) {
    return false;
  }

  EdgeColInfo eab = test_edge_combos(poly_A, poly_B, A_to_B);
  // CLOGI("Edge Cross Product: %.5f", eab.separation);
  if (eab.separation > 0.0f) {
    return false;
  }

  const bool is_face_contact =
      (fa.separation > eab.separation) || (fb.separation > eab.separation);
  if (is_face_contact) {
    out = create_face_contact(poly_A, poly_B, fa, fb, inverse_A);
    // CLOGI("FACE CONTACT -- %lu CONTACTS", out.num_points);

    if (debug) {
      for (int i = 0; i < out.num_points; i++) {
        debug->draw_vertex(out.contacts[i].pos_world, glm::vec4(1, 0, 0, 1));
        // CLOGI("CONTACT[%d] -- PENETRATION: %.5f", (int)i,
        // out.points[i].pen_depth);
      }
    }

  } else {
    out = create_edge_contact(poly_A, poly_B, eab, inverse_A);

    // CLOGI("EDGE CONTACT -- PENETRATION: %.5f", out.points[0].pen_depth);

    if (debug) {
      const ConvexMesh &mesh_A = poly_A.get_mesh();
      const ConvexMesh &mesh_B = poly_B.get_mesh();
      const glm::mat4 xfrm_A = poly_A.get_physics_matrix();
      const glm::mat4 xfrm_B = poly_B.get_physics_matrix();
      const auto verts_A =
          mesh_A.get_half_edge_vertices(mesh_A.half_edges[eab.edge_idx.first]);
      const auto verts_B =
          mesh_B.get_half_edge_vertices(mesh_B.half_edges[eab.edge_idx.second]);

      debug->draw_line(
          glm::vec3(xfrm_A * glm::vec4(mesh_A.vertices[verts_A[0]], 1.0f)),
          glm::vec3(xfrm_A * glm::vec4(mesh_A.vertices[verts_A[1]], 1.0f)),
          glm::vec4(1, 0, 0, 1));
      debug->draw_line(
          glm::vec3(xfrm_B * glm::vec4(mesh_B.vertices[verts_B[0]], 1.0f)),
          glm::vec3(xfrm_B * glm::vec4(mesh_B.vertices[verts_B[1]], 1.0f)),
          glm::vec4(1, 0, 0, 1));
      debug->draw_vertex(out.contacts[0].pos_world, glm::vec4(1, 0, 0, 1));
    }
  }

  return true;
}

FaceColInfo Narrowphase::test_face_normals(const RigidBody &poly_A,
                                           const RigidBody &poly_B,
                                           glm::mat4 A_to_B) {

  glm::mat3 norm_matrix = get_normal_matrix(A_to_B);

  float max_separation = -std::numeric_limits<float>::max();
  size_t max_idx;

  const auto &faces = poly_A.get_mesh().faces;
  for (size_t i = 0; i < faces.size(); i++) {
    Plane plane = faces[i].plane;

    // Transform face to B's local space
    glm::vec3 axis = norm_matrix * plane.normal;
    plane.normal = axis;
    plane.point = glm::vec3(A_to_B * glm::vec4(plane.point, 1.0f));
    plane.distance = -glm::dot(plane.normal, plane.point);

    glm::vec3 vertex = find_support_point(-axis, poly_B);
    float separation = get_signed_distance_to_plane(vertex, plane);

    if (separation > max_separation + EPSILON) {
      max_separation = separation;
      max_idx = i;
    }
  }

  return FaceColInfo(max_separation, max_idx);
}

EdgeColInfo Narrowphase::test_edge_combos(const RigidBody &poly_A,
                                          const RigidBody &poly_B,
                                          glm::mat4 A_to_B) {

  const glm::mat3 norm_matrix_A = get_normal_matrix(A_to_B);
  glm::vec3 centroid_A = glm::vec3(
      A_to_B * glm::vec4(poly_A.get_local_geometric_centroid(), 1.0f));

  const ConvexMesh &mesh_A = poly_A.get_mesh();
  const ConvexMesh &mesh_B = poly_B.get_mesh();

  size_t edge_count_A = mesh_A.half_edges.size();
  size_t edge_count_B = mesh_B.half_edges.size();

  float max_separation = -std::numeric_limits<float>::max();
  std::pair<size_t, size_t> max_idx;

  std::vector<bool> visited_A(edge_count_A, false);
  std::vector<bool> visited_B(edge_count_B, false);

  for (size_t i = 0; i < edge_count_A; i++) {
    if (visited_A[i]) {
      continue;
    }

    const auto he_A = mesh_A.half_edges[i];
    visited_A[i] = true;
    visited_A[he_A.twin] = true;

    const auto he_verts_A = mesh_A.get_half_edge_vertices(he_A);
    glm::vec3 p1 =
        glm::vec3(A_to_B * glm::vec4(mesh_A.vertices[he_verts_A[0]], 1.0f));
    glm::vec3 q1 =
        glm::vec3(A_to_B * glm::vec4(mesh_A.vertices[he_verts_A[1]], 1.0f));
    glm::vec3 edge_A = q1 - p1;

    // Transform A's face normals into B's local space
    const glm::vec3 norm_A =
        glm::normalize(norm_matrix_A * mesh_A.faces[he_A.face].plane.normal);
    const glm::vec3 norm_B = glm::normalize(
        norm_matrix_A *
        mesh_A.faces[mesh_A.half_edges[he_A.twin].face].plane.normal);

    std::fill(visited_B.begin(), visited_B.end(), false);
    for (size_t j = 0; j < edge_count_B; j++) {
      if (visited_B[j]) {
        continue;
      }

      const auto he_B = mesh_B.half_edges[j];
      visited_B[j] = true;
      visited_B[he_B.twin] = true;

      const auto he_verts_B = mesh_B.get_half_edge_vertices(he_B);
      glm::vec3 p2 = mesh_B.vertices[he_verts_B[0]];
      glm::vec3 q2 = mesh_B.vertices[he_verts_B[1]];
      glm::vec3 edge_B = q2 - p2;

      // Face normals in B's local space
      glm::vec3 norm_C = glm::normalize(mesh_B.faces[he_B.face].plane.normal);
      glm::vec3 norm_D = glm::normalize(
          mesh_B.faces[mesh_B.half_edges[he_B.twin].face].plane.normal);

      // Gauss-map pruning: only test edges that form Minkowski face
      // Negate B's normals to account for Minkowski difference
      if (!is_minkowski_face(norm_A, norm_B, -norm_C, -norm_D)) {
        continue;
      }
      glm::vec3 cross = glm::cross(edge_A, edge_B);
      // WARNING: uses sqrt() to compute len
      float cross_len = glm::length(cross);
      if (cross_len < EPSILON) {
        continue;
      }

      // Separation axis points from A towards B
      glm::vec3 axis = cross / cross_len;
      if (glm::dot(axis, p1 - centroid_A) < 0.0f) {
        axis = -axis;
      }

      float separation = glm::dot(axis, p2 - p1);
      if (separation > max_separation + EPSILON) {
        max_separation = separation;
        max_idx = {i, j};
      }
    }
  }

  return EdgeColInfo(max_separation, max_idx);
}

Manifold Narrowphase::create_face_contact(const RigidBody &poly_A,
                                          const RigidBody &poly_B,
                                          const FaceColInfo &fa,
                                          const FaceColInfo &fb,
                                          const glm::mat4 &inv_A) {

  // Bias reference/incident bodies
  auto [ref_body, inc_body, ref_face_idx] =
      bias_reference_face(poly_A, poly_B, fa, fb);

  const ConvexMesh &ref_mesh = ref_body->get_mesh();
  const ConvexMesh &inc_mesh = inc_body->get_mesh();

  const glm::mat4 ref_xfrm = ref_body->get_physics_matrix();
  const glm::mat4 inc_xfrm = inc_body->get_physics_matrix();
  const glm::mat3 ref_norm_mat = ref_body->transform.get_normal_matrix();

  const ConvexMesh::Face &ref_face = ref_mesh.faces[ref_face_idx];
  glm::vec3 ref_norm_world =
      glm::normalize(ref_norm_mat * ref_face.plane.normal);

  // Find incident face (most anti-parallel face)
  size_t inc_face_idx = find_incident_face(ref_norm_world, *inc_body);
  const ConvexMesh::Face &inc_face = inc_mesh.faces[inc_face_idx];

  // Build world-space incident polygon; each vertex carries its mesh index as ID.
  std::vector<ClipVertex> inc_polygon;
  for (size_t idx : inc_mesh.get_face_vertices(inc_face)) {
    inc_polygon.push_back({
        glm::vec3(inc_xfrm * glm::vec4(inc_mesh.vertices[idx], 1.0f)),
        static_cast<uint32_t>(idx)
    });
  }

  // Build world-space reference plane
  Plane ref_plane;
  ref_plane.normal = ref_norm_world;
  ref_plane.point = glm::vec3(ref_xfrm * glm::vec4(ref_face.plane.point, 1.0f));
  ref_plane.distance = -glm::dot(ref_plane.normal, ref_plane.point);

  // Clip incident polygon; intersection vertices get IDs from the clip plane.
  clip_against_reference_face(inc_polygon, ref_mesh, ref_xfrm, ref_face,
                              ref_plane);

  const uint64_t body_id = pack_id(static_cast<uint32_t>(poly_A.id),
                                   static_cast<uint32_t>(poly_B.id));

  std::vector<Contact> candidates;
  float max_pen = -std::numeric_limits<float>::max();

  for (const auto &cv : inc_polygon) {
    float separation = get_signed_distance_to_plane(cv.pos_world, ref_plane);

    // above reference plane - discard candidate
    if (separation >= 0.0f) {
      continue;
    }

    Contact c;
    c.pos_world = cv.pos_world - separation * ref_plane.normal;
    c.norm = ref_plane.normal;
    c.pen_depth = -separation; // store penetration as positive value
    c.body_id = body_id;
    c.vertex_id = cv.id;
    c.a_pos_local = glm::vec3(inv_A * glm::vec4(c.pos_world, 1.0f));

    candidates.push_back(c);
    if (c.pen_depth > max_pen) {
      max_pen = c.pen_depth;
    }
  }

  // Reduce manifold to at most 4 points
  const std::vector<Contact> reduced_contacts =
      reduce_manifold(candidates, ref_plane.normal);

  Manifold out;
  out.norm = ref_plane.normal;
  out.max_pen_depth = max_pen;
  out.num_points = std::min<size_t>(4, reduced_contacts.size());
  for (int i = 0; i < out.num_points; ++i) {
    out.contacts[i] = reduced_contacts[i];
  }

  return out;
}

// find closest points on two colliding edges (midpoint????)
Manifold Narrowphase::create_edge_contact(const RigidBody &poly_A,
                                          const RigidBody &poly_B,
                                          const EdgeColInfo &eab,
                                          const glm::mat4 &inv_A) {

  const ConvexMesh &mesh_A = poly_A.get_mesh();
  const ConvexMesh &mesh_B = poly_B.get_mesh();
  const glm::mat4 xfrm_A = poly_A.get_physics_matrix();
  const glm::mat4 xfrm_B = poly_B.get_physics_matrix();

  const auto &he_A = mesh_A.half_edges[eab.edge_idx.first];
  const auto &he_B = mesh_B.half_edges[eab.edge_idx.second];
  const auto he_verts_A = mesh_A.get_half_edge_vertices(he_A);
  const auto he_verts_B = mesh_B.get_half_edge_vertices(he_B);

  // parametric equation for edges
  // p1 + (p2 - p1) * s
  // q1 + (q2 - q1) * t

  // transform into world space
  const glm::vec3 p1 =
      glm::vec3(xfrm_A * glm::vec4(mesh_A.vertices[he_verts_A[0]], 1.0f));
  const glm::vec3 q1 =
      glm::vec3(xfrm_A * glm::vec4(mesh_A.vertices[he_verts_A[1]], 1.0f));
  const glm::vec3 p2 =
      glm::vec3(xfrm_B * glm::vec4(mesh_B.vertices[he_verts_B[0]], 1.0f));
  const glm::vec3 q2 =
      glm::vec3(xfrm_B * glm::vec4(mesh_B.vertices[he_verts_B[1]], 1.0f));
  const glm::vec3 edge_A = q1 - p1;
  const glm::vec3 edge_B = q2 - p2;

  // closest points between edges occur
  // when line segment is perpendicular to both edges
  const glm::vec3 w0 = p1 - p2;
  const float a = glm::dot(edge_A, edge_A);
  const float b = glm::dot(edge_B, edge_B);
  const float a_b = glm::dot(edge_A, edge_B);
  const float a_w0 = glm::dot(edge_A, w0);
  const float b_w0 = glm::dot(edge_B, w0);
  const float det = a * b - a_b * a_b;

  glm::vec3 closest_A, closest_B;
  if (std::abs(det) < EPSILON) {
    // Parallel edges - closest point is midpoint of one edge
    closest_A = 0.5f * (p1 + q1);
    closest_B = closest_A;
  } else {
    const float s = glm::clamp((a_b * b_w0 - b * a_w0) / det, 0.0f, 1.0f);
    const float t = glm::clamp((a * b_w0 - a_b * a_w0) / det, 0.0f, 1.0f);
    closest_A = p1 + edge_A * s;
    closest_B = p2 + edge_B * t;
  }

  // Contact normal points from A to B
  glm::vec3 contact = closest_B - closest_A;
  float dist = glm::length(contact);

  glm::vec3 axis;
  if (dist > EPSILON) {
    axis = contact / dist;
  } else {
    axis = glm::normalize(glm::cross(edge_A, edge_B));
  }

  // Ensure axis points from A toward B
  if (glm::dot(axis, poly_B.get_centre_of_mass() -
                         poly_A.get_centre_of_mass()) < 0.0f) {
    axis = -axis;
  }

  Manifold out;
  out.num_points = 1;
  out.norm = axis;
  out.max_pen_depth = dist;

  uint64_t body_id = pack_id(static_cast<uint32_t>(poly_A.id),
                             static_cast<uint32_t>(poly_B.id));
  // Pack the two half-edge indices into 32 bits (each clamped to 16 bits).
  uint32_t vertex_id = (static_cast<uint32_t>(eab.edge_idx.first  & 0xFFFF) << 16) |
                        static_cast<uint32_t>(eab.edge_idx.second & 0xFFFF);
  out.contacts[0].pos_world = 0.5f * (closest_A + closest_B);
  out.contacts[0].norm = axis;
  out.contacts[0].pen_depth = dist;
  out.contacts[0].body_id = body_id;
  out.contacts[0].vertex_id = vertex_id;
  out.contacts[0].a_pos_local =
      glm::vec3(inv_A * glm::vec4(out.contacts[0].pos_world, 1.0f));

  return out;
}

Manifold Narrowphase::create_plane_contact(const RigidBody &poly,
                                           const Plane &plane,
                                           Debugger *debug) {

  glm::mat4 xfrm = poly.get_physics_matrix();
  const auto &local_verts = poly.get_mesh().vertices;

  std::vector<Contact> candidates;
  float max_pen = -std::numeric_limits<float>::max();

  for (size_t i = 0; i < local_verts.size(); i++) {
    const glm::vec3 world = glm::vec3(xfrm * glm::vec4(local_verts[i], 1.0f));

    const float separation = get_signed_distance_to_plane(world, plane);

    // Above plane - discard point
    if (separation > 0.0f) {
      continue;
    }
    Contact c;
    c.pos_world = world;
    c.norm = plane.normal;
    c.pen_depth = -separation; // store penetration as positive
    c.body_id = static_cast<uint64_t>(poly.id);
    c.vertex_id = static_cast<uint32_t>(i);
    c.a_pos_local = local_verts[i]; // already in poly's local space
    candidates.push_back(c);

    if (c.pen_depth > max_pen) {
      max_pen = c.pen_depth;
    }
  }

  if (candidates.empty()) {
    return Manifold{};
  }

  const auto reduced = reduce_manifold(candidates, plane.normal);

  Manifold out;
  out.norm = plane.normal;
  out.max_pen_depth = max_pen;
  out.num_points = std::min<size_t>(4, reduced.size());
  for (size_t i = 0; i < out.num_points; i++) {
    out.contacts[i] = reduced[i];
  }

  return out;
}

/**
 * @brief Gauss-map arc intersection test (Minkowski face check)
 *
 * Tests whether arcs AB and CD intersect on unit sphere.
 * Reference:
 * https://media.gdcvault.com/gdc2013/slides/822403Gregorius_Dirk_TheSeparatingAxisTest.pdf
 */
bool Narrowphase::is_minkowski_face(const glm::vec3 &a, const glm::vec3 &b,
                                    const glm::vec3 &c, const glm::vec3 &d) {

  glm::vec3 BxA = glm::cross(b, a);
  glm::vec3 DxC = glm::cross(d, c);

  float CBA = glm::dot(c, BxA);
  float DBA = glm::dot(d, BxA);
  float ADC = glm::dot(a, DxC);
  float BDC = glm::dot(b, DxC);

  return CBA * DBA < 0.0f && ADC * BDC < 0.0f && CBA * BDC > 0.0f;
}

/**
 * @brief: Find furthest vertex in direction opposite to normal
 */
glm::vec3 Narrowphase::find_support_point(glm::vec3 axis,
                                          const RigidBody &poly) {
  const auto &local_verts = poly.get_mesh().vertices;
  float max_proj = -std::numeric_limits<float>::max();
  glm::vec3 result;

  for (size_t i = 0; i < local_verts.size(); i++) {
    float proj = glm::dot(local_verts[i], axis);
    if (proj > max_proj) {
      max_proj = proj;
      result = local_verts[i];
    }
  }

  return result;
}

void Narrowphase::clip_against_reference_face(std::vector<ClipVertex> &polygon,
                                              const ConvexMesh &ref_mesh,
                                              const glm::mat4 &ref_transform,
                                              const ConvexMesh::Face &ref_face,
                                              const Plane &ref_plane) {

  // Get reference face center for side plane orientation
  glm::vec3 ref_center(0.0f);
  auto ref_verts = ref_mesh.get_face_vertices(ref_face);
  for (size_t idx : ref_verts) {
    ref_center +=
        glm::vec3(ref_transform * glm::vec4(ref_mesh.vertices[idx], 1.0f));
  }
  ref_center /= ref_verts.size();

  // Clip polygon against each edge of reference face
  for (size_t idx : ref_mesh.get_face_half_edges(ref_face)) {
    auto &he = ref_mesh.half_edges[idx];
    auto he_verts = ref_mesh.get_half_edge_vertices(he);

    glm::vec3 e0 = glm::vec3(ref_transform *
                             glm::vec4(ref_mesh.vertices[he_verts[0]], 1.0f));
    glm::vec3 e1 = glm::vec3(ref_transform *
                             glm::vec4(ref_mesh.vertices[he_verts[1]], 1.0f));

    glm::vec3 side_normal =
        glm::cross(glm::normalize(e1 - e0), ref_plane.normal);

    if (glm::dot(side_normal, ref_center - (e0 + e1) * 0.5f) > 0.0f) {
      side_normal = -side_normal;
    }

    polygon = clip_polygon_against_plane(polygon, Plane(side_normal, e0),
                                         static_cast<uint32_t>(idx));
  }
}

std::vector<ClipVertex>
Narrowphase::clip_polygon_against_plane(const std::vector<ClipVertex> &polygon,
                                        const Plane &plane, uint32_t clip_id) {
  std::vector<ClipVertex> out;

  if (polygon.empty())
    return out;

  size_t vertex_count = polygon.size();
  for (size_t i = 0; i < vertex_count; i++) {
    const ClipVertex &first  = polygon[i];
    const ClipVertex &second = polygon[(i + 1) % vertex_count];

    float first_distance  = get_signed_distance_to_plane(first.pos_world,  plane);
    float second_distance = get_signed_distance_to_plane(second.pos_world, plane);

    // Current vertex is inside (or on) the plane
    if (first_distance <= 0) {
      out.push_back(first);

      // Edge crosses from inside to outside — emit intersection vertex.
      if (second_distance > 0) {
        float t = first_distance / (first_distance - second_distance);
        ClipVertex intersection;
        intersection.pos_world = first.pos_world + (second.pos_world - first.pos_world) * t;
        intersection.id = CLIP_INTERSECTION_FLAG | clip_id;
        out.push_back(intersection);
      }
    }
    // Current vertex is outside, next is inside — emit intersection only.
    else if (second_distance <= 0) {
      float t = first_distance / (first_distance - second_distance);
      ClipVertex intersection;
      intersection.pos_world = first.pos_world + (second.pos_world - first.pos_world) * t;
      intersection.id = CLIP_INTERSECTION_FLAG | clip_id;
      out.push_back(intersection);
    }

    // Both outside -- don't add anything
  }

  return out;
}

/*
std::vector<ClipVertex>
Narrowphase::clip_polygon_against_plane(const std::vector<ClipVertex> &polygon,
                                        const Plane &plane, uint32_t clip_id) {

  std::vector<ClipVertex> out;
  if (polygon.empty()) {
    return out;
  }

  ClipVertex v1 = polygon.back();
  float d1 = get_signed_distance_to_plane(v1.pos_world, plane);

  for (const auto &v2 : polygon) {
    float d2 = get_signed_distance_to_plane(v2.pos_world, plane);

    // Case: Edge crosses the plane
    if ((d1 > 0 && d2 <= 0) || (d1 <= 0 && d2 > 0)) {
      float t = d1 / (d1 - d2);
      ClipVertex intersection;
      intersection.pos_world = v1.pos_world + t * (v2.pos_world - v1.pos_world);

      // The intersection point is created by the edge (v1, v2)
      // being cut by the clipping_side_id.
      // We use a simplified hash or your pack_id logic here.
      intersection.id = clip_id;
      out.push_back(intersection);
    }

    // Case: Current vertex is inside
    if (d2 <= 0) {
      out.push_back(v2);
    }

    v1 = v2;
    d1 = d2;
  }

  return out;
}
*/

size_t Narrowphase::find_incident_face(const glm::vec3 &ref_norm_world,
                                       const RigidBody &poly) {

  // Transform reference normal from world space to poly's local space
  // For normals: use transpose of rotation matrix (inverse for orthogonal
  // transforms)
  glm::mat3 inv_rot_matrix = glm::transpose(glm::mat3_cast(poly.orientation));
  glm::vec3 ref_norm_local = glm::normalize(inv_rot_matrix * ref_norm_world);

  const auto &faces = poly.get_mesh().faces;
  float min_dot = std::numeric_limits<float>::max();
  size_t min_idx = 0;

  for (size_t i = 0; i < faces.size(); i++) {
    float dot = glm::dot(ref_norm_local, faces[i].plane.normal);
    if (dot < min_dot) {
      min_dot = dot;
      min_idx = i;
    }
  }

  return min_idx;
}

/**
 * Bias poly A as reference face for better frame coherence
 */
FaceContactInfo Narrowphase::bias_reference_face(const RigidBody &a,
                                                 const RigidBody &b,
                                                 const FaceColInfo &fa,
                                                 const FaceColInfo &fb) {

  bool a_bias = (std::abs(fa.separation - fb.separation) < REFERENCE_BIAS)
                    ? a.id < b.id // near-equal: use ID as tiebreak
                    : fa.separation > fb.separation + REFERENCE_BIAS;

  if (a_bias)
    return {&a, &b, fa.face_idx};
  else
    return {&b, &a, fb.face_idx};
}

inline float Narrowphase::get_signed_triangle_area(const glm::vec3 &a,
                                                   const glm::vec3 &b,
                                                   const glm::vec3 &c,
                                                   const glm::vec3 &normal) {
  glm::vec3 ab = b - a;
  glm::vec3 ac = c - a;
  glm::vec3 cross = glm::cross(ab, ac);
  return glm::dot(cross, normal);
}

std::vector<Contact> Narrowphase::reduce_manifold(std::vector<Contact> contacts,
                                                  glm::vec3 ref_norm) {
  if (contacts.size() <= 4) {
    return contacts;
  }

  std::vector<Contact> out;
  out.reserve(4);

  // 1. find deepest support point
  int deepest_idx = 0;
  float max_pen = contacts[0].pen_depth;

  for (int i = 1; i < contacts.size(); i++) {
    if (contacts[i].pen_depth > max_pen) {
      max_pen = contacts[i].pen_depth;
      deepest_idx = i;
    }
  }
  out.push_back(contacts[deepest_idx]);

  // 2. farthest point from deepest
  int farthest_idx = -1;
  float max_dist2 = -std::numeric_limits<float>::max();

  for (int i = 0; i < contacts.size(); ++i) {
    if (i == deepest_idx) {
      continue;
    }

    float dist2 =
        glm::distance2(contacts[i].pos_world, contacts[deepest_idx].pos_world);
    if (dist2 > max_dist2) {
      max_dist2 = dist2;
      farthest_idx = i;
    }
  }

  // edge case - all points are same
  if (farthest_idx == -1 || max_dist2 < 1e-6f) {
    return out;
  }
  out.push_back(contacts[farthest_idx]);

  // 3. point that maximizes triangle area
  int third_idx = -1;
  float max_area = -std::numeric_limits<float>::max();
  for (int i = 0; i < contacts.size(); ++i) {
    if (i == deepest_idx || i == farthest_idx) {
      continue;
    }

    float area = get_signed_triangle_area(contacts[deepest_idx].pos_world,
                                          contacts[farthest_idx].pos_world,
                                          contacts[i].pos_world, ref_norm);

    if (area > max_area) {
      max_area = area;
      third_idx = i;
    }
  }

  // edge case - all points are collinear
  if (third_idx == -1 || std::abs(max_area) < 1e-6f) {
    return out;
  }
  out.push_back((contacts[third_idx]));

  // 4. point that maximizes total area
  int fourth_idx = -1;
  float max_negative_area = 0.0f;
  for (int i = 0; i < contacts.size(); ++i) {
    if (i == deepest_idx || i == farthest_idx || i == third_idx) {
      continue;
    }

    // Edge 0: deepest -> farthest
    float area0 = get_signed_triangle_area(contacts[deepest_idx].pos_world,
                                           contacts[farthest_idx].pos_world,
                                           contacts[i].pos_world, ref_norm);

    // Edge 1: farthest -> third
    float area1 = get_signed_triangle_area(contacts[farthest_idx].pos_world,
                                           contacts[third_idx].pos_world,
                                           contacts[i].pos_world, ref_norm);

    // Edge 2: third -> deepest
    float area2 = get_signed_triangle_area(contacts[third_idx].pos_world,
                                           contacts[deepest_idx].pos_world,
                                           contacts[i].pos_world, ref_norm);

    // Keep the most negative area across all three edges
    float min_area = std::min({area0, area1, area2});

    if (min_area < max_negative_area) {
      max_negative_area = min_area;
      fourth_idx = i;
    }
  }

  if (fourth_idx != -1) {
    out.push_back(contacts[fourth_idx]);
  }

  return out;
}
