#include "Shapes.hpp"

#include <algorithm>
#include <cstdint>
#include <glm/ext/quaternion_geometric.hpp>
#include <glm/geometric.hpp>
#include <limits>
#include <vector>

#include "HalfEdgeMesh.hpp"
#include "Plane.hpp"
#include "Primitive.hpp"

/**
 * @brief Polyhedron pair routines. Implements Gauss map optimized Separating
 * Axis Theorem.
 *
 * Hulls are tested in geometry-space (Polyhedron::get_geom_mesh - local_xfrm
 * baked in), so body transforms stay rigid: rotation alone maps normals and the
 * cached inverse maps between bodies.
 */

namespace {

using col::pack_id;
using col::ShapeProperties;

struct ClipVertex {
  glm::vec3 pos_world;
  uint32_t id; // mesh vertex index, or CLIP_INTERSECTION_FLAG | side half-edge
};

struct FaceCollision {
  float separation = 0.0f;
  size_t face = 0;

  FaceCollision() = default;
  FaceCollision(float separation, size_t index)
      : separation(separation), face(index) {}
};

struct EdgeCollision {
  float separation = 0.0f;
  std::pair<size_t, size_t> edges = {0, 0};
  glm::vec3 axis = glm::vec3(0.0f); // B's geometry-space, pointing A -> B

  EdgeCollision() = default;
  EdgeCollision(float separation, std::pair<size_t, size_t> indices,
                glm::vec3 axis)
      : separation(separation), edges(indices), axis(axis) {}
};

struct FaceContact {
  const ShapeProperties *ref, *inc;
  size_t ref_face;
};

// High bit set on vertex_id indicates a clipping-plane intersection;
// lower 31 bits store the reference side-plane half-edge index.
constexpr uint32_t CLIP_INTERSECTION_FLAG = 0x80000000u;
constexpr float EPSILON = 1e-6f;
constexpr float REFERENCE_BIAS = 0.01f;

// dispatch table guarantees prim is a Polyhedron
const HalfEdgeMesh &body_mesh(const ShapeProperties &s) {
  return static_cast<const Polyhedron *>(s.prim)->get_geom_mesh();
}

FaceCollision poly2_face_normals(const ShapeProperties &a,
                                 const ShapeProperties &b,
                                 const glm::mat4 &a2b_xfrm);
EdgeCollision poly2_edge_cross(const ShapeProperties &a,
                               const ShapeProperties &b,
                               const glm::mat4 &a2b_xfrm);

Manifold create_face_contact(const ShapeProperties &a, const ShapeProperties &b,
                             const FaceCollision &fa, const FaceCollision &fb);
Manifold create_edge_contact(const ShapeProperties &a, const ShapeProperties &b,
                             const EdgeCollision &eab);

bool is_minkowski_face(const glm::vec3 &a, const glm::vec3 &b,
                       const glm::vec3 &c, const glm::vec3 &d);
glm::vec3 find_support_point(glm::vec3 axis, const HalfEdgeMesh &mesh);

glm::vec3 closest_point_on_segment(const glm::vec3 &p, const glm::vec3 &q,
                                   const glm::vec3 &x);
glm::vec3 closest_point_on_face(const HalfEdgeMesh &mesh,
                                const HalfEdgeMesh::Face &face,
                                const glm::vec3 &x);

size_t find_incident_face(const glm::vec3 &ref_norm_world,
                          const ShapeProperties &inc);
FaceContact bias_reference_face(const ShapeProperties &a,
                                const ShapeProperties &b,
                                const FaceCollision &fa,
                                const FaceCollision &fb);

void clip_against_reference_face(std::vector<ClipVertex> &polygon,
                                 const HalfEdgeMesh &ref_mesh,
                                 const glm::mat4 &ref_transform,
                                 const HalfEdgeMesh::Face &ref_face,
                                 const Plane &ref_plane);
std::vector<ClipVertex>
clip_polygon_against_plane(const std::vector<ClipVertex> &polygon,
                           const Plane &plane, uint32_t clip_id);

std::vector<Contact> reduce_manifold(std::vector<Contact> contacts,
                                     glm::vec3 ref_norm);
float get_signed_triangle_area(const glm::vec3 &a, const glm::vec3 &b,
                               const glm::vec3 &c, const glm::vec3 &normal);

} // namespace

namespace col {

// O(n^2)
// Gauss Map Optimization
bool poly2(Manifold &out, const ShapeProperties &a, const ShapeProperties &b) {

  // perform tests in other model's local space
  const glm::mat4 a2b_xfrm =
      b.xfrm->get_inverse_matrix() * a.xfrm->get_matrix();
  const glm::mat4 b2a_xfrm =
      a.xfrm->get_inverse_matrix() * b.xfrm->get_matrix();

  FaceCollision a_test = poly2_face_normals(a, b, a2b_xfrm);
  // CLOGI("Poly A Normals: %.5f", fa.separation);
  if (a_test.separation > 0.0f) {
    return false;
  }

  FaceCollision b_test = poly2_face_normals(b, a, b2a_xfrm);
  // CLOGI("Poly B Normals: %.5f", fb.separation);
  if (b_test.separation > 0.0f) {
    return false;
  }

  EdgeCollision edge_test = poly2_edge_cross(a, b, a2b_xfrm);
  // CLOGI("Edge Cross Product: %.5f", eab.separation);
  if (edge_test.separation > 0.0f) {
    return false;
  }

  // a & b collide
  // collision type depends on separation axis ()

  const bool is_face_contact = (a_test.separation > edge_test.separation) ||
                               (b_test.separation > edge_test.separation);
  if (is_face_contact) {
    out = create_face_contact(a, b, a_test, b_test);

    // CLOGI("FACE CONTACT -- %lu CONTACTS", out.num_points);
    // for (size_t i = 0; i < out.num_points; i++) {
    //  CLOGI("CONTACT%lu -- PENETRATION: %.5f", i, out.contacts[0].pen_depth);
    // }

  } else {
    out = create_edge_contact(a, b, edge_test);

    // CLOGI("EDGE CONTACT -- PENETRATION: %.5f", out.contacts[0].pen_depth);
  }

  // clipping can discard every incident point
  return out.num_points > 0;
}

// sphere centre vs hull in A's body space - single contact point
bool poly_sphere(Manifold &out, const ShapeProperties &a,
                 const ShapeProperties &b) {
  const HalfEdgeMesh &mesh = body_mesh(a);
  const WorldSphere sphere = col::sphere_cast(b);

  const glm::vec3 centre =
      glm::vec3(a.xfrm->get_inverse_matrix() * glm::vec4(sphere.centre, 1.0f));
  const float radius = sphere.radius;

  // face planes are unit & outward - max signed distance is a lower bound on
  // the true distance to the hull
  float max_separation = -std::numeric_limits<float>::max();
  size_t max_face = 0;
  for (size_t i = 0; i < mesh.faces.size(); i++) {
    float separation =
        get_signed_distance_to_plane(centre, mesh.faces[i].plane);
    if (separation > max_separation) {
      max_separation = separation;
      max_face = i;
    }
  }

  if (max_separation > radius) {
    return false;
  }

  glm::vec3 surface;    // closest hull point, body space
  glm::vec3 normal_out; // out of the hull, body space
  float depth;

  if (max_separation <= 0.0f) {
    // centre inside hull - push out through the shallowest face
    normal_out = mesh.faces[max_face].plane.normal;
    surface = centre - max_separation * normal_out;
    depth = radius - max_separation;
  } else {
    // centre outside - closest point lies on a face the centre is in front of
    float min_dist2 = std::numeric_limits<float>::max();
    for (const auto &face : mesh.faces) {
      if (get_signed_distance_to_plane(centre, face.plane) <= 0.0f) {
        continue;
      }
      const glm::vec3 q = closest_point_on_face(mesh, face, centre);
      const glm::vec3 d = centre - q;
      const float dist2 = glm::dot(d, d);
      if (dist2 < min_dist2) {
        min_dist2 = dist2;
        surface = q;
      }
    }

    // within radius of a face plane but beyond an edge/vertex region
    const float dist = std::sqrt(min_dist2);
    if (dist >= radius) {
      return false;
    }

    normal_out = dist > EPSILON ? (centre - surface) / dist
                                : mesh.faces[max_face].plane.normal;
    depth = radius - dist;
  }

  // midway between the hull surface and the sphere's deepest point
  const glm::vec3 sphere_deep = centre - radius * normal_out;
  const glm::vec3 contact = 0.5f * (surface + sphere_deep);

  // body matrix is rigid - rotation alone maps normals; flip to B -> A
  const glm::mat4 a_xfrm = a.xfrm->get_matrix();
  const glm::vec3 pos_world = glm::vec3(a_xfrm * glm::vec4(contact, 1.0f));
  const glm::vec3 normal = -(glm::mat3(a_xfrm) * normal_out);

  set_single_contact(out, a, b, pos_world, normal, depth);
  return true;
}

// TODO
bool poly_frustum(Manifold &out, const ShapeProperties &a,
                  const ShapeProperties &b) {
  return false;
}

} // namespace col

namespace {

FaceCollision poly2_face_normals(const ShapeProperties &a,
                                 const ShapeProperties &b,
                                 const glm::mat4 &a2b_xfrm) {

  // a2b is rigid - rotation matrix alone maps normals
  const glm::mat3 a2b_rot = glm::mat3(a2b_xfrm);
  const HalfEdgeMesh &b_mesh = body_mesh(b);

  float max_separation = -std::numeric_limits<float>::max();
  size_t max_index = 0;

  const auto &faces = body_mesh(a).faces;
  for (size_t i = 0; i < faces.size(); i++) {
    // transform face normal to b's local space
    const glm::vec3 axis = a2b_rot * faces[i].plane.normal;
    const Plane plane(
        axis, glm::vec3(a2b_xfrm * glm::vec4(faces[i].plane.point, 1.0f)));

    glm::vec3 vertex = find_support_point(-axis, b_mesh);
    float separation = get_signed_distance_to_plane(vertex, plane);

    if (separation > max_separation + EPSILON) {
      max_separation = separation;
      max_index = i;
    }
  }

  return FaceCollision(max_separation, max_index);
}

EdgeCollision poly2_edge_cross(const ShapeProperties &a,
                               const ShapeProperties &b,
                               const glm::mat4 &a2b_xfrm) {

  // a2b is rigid - rotation alone maps normals
  const glm::mat3 a2b_rot = glm::mat3(a2b_xfrm);

  const HalfEdgeMesh &a_mesh = body_mesh(a);
  const HalfEdgeMesh &b_mesh = body_mesh(b);

  const glm::vec3 centroid_A = glm::vec3(
      a2b_xfrm * glm::vec4(a_mesh.compute_geometric_centroid(), 1.0f));

  size_t a_nedges = a_mesh.half_edges.size();
  size_t b_nedges = b_mesh.half_edges.size();

  float max_separation = -std::numeric_limits<float>::max();
  std::pair<size_t, size_t> max_index = {0, 0};
  glm::vec3 max_axis(0.0f);

  std::vector<bool> visited_A(a_nedges, false);
  std::vector<bool> visited_B(b_nedges, false);

  for (size_t i = 0; i < a_nedges; i++) {
    if (visited_A[i]) {
      continue;
    }

    const auto he_A = a_mesh.half_edges[i];
    visited_A[i] = true;
    visited_A[he_A.twin] = true;

    const auto he_verts_A = a_mesh.get_half_edge_vertices(he_A);
    glm::vec3 p1 =
        glm::vec3(a2b_xfrm * glm::vec4(a_mesh.vertices[he_verts_A[0]], 1.0f));
    glm::vec3 q1 =
        glm::vec3(a2b_xfrm * glm::vec4(a_mesh.vertices[he_verts_A[1]], 1.0f));
    glm::vec3 edge_A = q1 - p1;

    // Transform A's face normals into B's local space
    const glm::vec3 norm_A =
        glm::normalize(a2b_rot * a_mesh.faces[he_A.face].plane.normal);
    const glm::vec3 norm_B = glm::normalize(
        a2b_rot * a_mesh.faces[a_mesh.half_edges[he_A.twin].face].plane.normal);

    std::fill(visited_B.begin(), visited_B.end(), false);
    for (size_t j = 0; j < b_nedges; j++) {
      if (visited_B[j]) {
        continue;
      }

      const auto he_B = b_mesh.half_edges[j];
      visited_B[j] = true;
      visited_B[he_B.twin] = true;

      const auto he_verts_B = b_mesh.get_half_edge_vertices(he_B);
      glm::vec3 p2 = b_mesh.vertices[he_verts_B[0]];
      glm::vec3 q2 = b_mesh.vertices[he_verts_B[1]];
      glm::vec3 edge_B = q2 - p2;

      // Face normals in B's local space
      glm::vec3 norm_C = glm::normalize(b_mesh.faces[he_B.face].plane.normal);
      glm::vec3 norm_D = glm::normalize(
          b_mesh.faces[b_mesh.half_edges[he_B.twin].face].plane.normal);

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
        max_index = {i, j};
        max_axis = axis;
      }
    }
  }

  return EdgeCollision(max_separation, max_index, max_axis);
}

Manifold create_face_contact(const ShapeProperties &a, const ShapeProperties &b,
                             const FaceCollision &fa, const FaceCollision &fb) {

  // Bias reference/incident bodies
  auto [ref, inc, ref_face_idx] = bias_reference_face(a, b, fa, fb);

  const HalfEdgeMesh &ref_mesh = body_mesh(*ref);
  const HalfEdgeMesh &inc_mesh = body_mesh(*inc);

  const glm::mat4 ref_xfrm = ref->xfrm->get_matrix();
  const glm::mat4 inc_xfrm = inc->xfrm->get_matrix();

  const HalfEdgeMesh::Face &ref_face = ref_mesh.faces[ref_face_idx];
  glm::vec3 ref_norm_world =
      glm::normalize(glm::mat3(ref_xfrm) * ref_face.plane.normal);

  // Find incident face (most anti-parallel face)
  size_t inc_face_idx = find_incident_face(ref_norm_world, *inc);
  const HalfEdgeMesh::Face &inc_face = inc_mesh.faces[inc_face_idx];

  // Build world-space incident polygon; each vertex carries its mesh index as
  // ID.
  std::vector<ClipVertex> inc_polygon;
  for (size_t idx : inc_mesh.get_face_vertices(inc_face)) {
    inc_polygon.push_back(
        {glm::vec3(inc_xfrm * glm::vec4(inc_mesh.vertices[idx], 1.0f)),
         static_cast<uint32_t>(idx)});
  }

  // Build world-space reference plane
  const Plane ref_plane(
      ref_norm_world,
      glm::vec3(ref_xfrm * glm::vec4(ref_face.plane.point, 1.0f)));

  // Clip incident polygon; intersection vertices get IDs from the clip plane.
  clip_against_reference_face(inc_polygon, ref_mesh, ref_xfrm, ref_face,
                              ref_plane);

  // reference normal points out of the reference hull - negate when A holds
  // the reference face so the manifold normal points B -> A
  const glm::vec3 normal = (ref == &a) ? -ref_plane.normal : ref_plane.normal;

  const uint64_t body_id = pack_id(static_cast<uint32_t>(a.entity_id),
                                   static_cast<uint32_t>(b.entity_id));
  const glm::mat4 a_inv_xfrm = a.xfrm->get_inverse_matrix();

  std::vector<Contact> candidates;

  for (const auto &cv : inc_polygon) {
    float separation = get_signed_distance_to_plane(cv.pos_world, ref_plane);

    // above reference plane - discard candidate
    if (separation >= 0.0f) {
      continue;
    }

    Contact c;
    c.pos_world = cv.pos_world - separation * ref_plane.normal;
    c.norm = normal;
    c.pen_depth = -separation; // store penetration as positive value
    c.entity_id = body_id;
    c.vertex_id = cv.id;
    c.a_pos_local = glm::vec3(a_inv_xfrm * glm::vec4(c.pos_world, 1.0f));

    candidates.push_back(c);
  }

  // Reduce manifold to at most 4 points
  const std::vector<Contact> reduced_contacts =
      reduce_manifold(candidates, ref_plane.normal);

  Manifold out;
  out.norm = normal;
  out.num_points = std::min<size_t>(4, reduced_contacts.size());
  for (size_t i = 0; i < out.num_points; ++i) {
    out.contacts[i] = reduced_contacts[i];
  }

  return out;
}

// find closest points on two colliding edges (midpoint????)
Manifold create_edge_contact(const ShapeProperties &a, const ShapeProperties &b,
                             const EdgeCollision &eab) {

  const HalfEdgeMesh &mesh_A = body_mesh(a);
  const HalfEdgeMesh &mesh_B = body_mesh(b);
  const glm::mat4 xfrm_A = a.xfrm->get_matrix();
  const glm::mat4 xfrm_B = b.xfrm->get_matrix();

  const auto &he_A = mesh_A.half_edges[eab.edges.first];
  const auto &he_B = mesh_B.half_edges[eab.edges.second];
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
  const float a_a = glm::dot(edge_A, edge_A);
  const float b_b = glm::dot(edge_B, edge_B);
  const float a_b = glm::dot(edge_A, edge_B);
  const float a_w0 = glm::dot(edge_A, w0);
  const float b_w0 = glm::dot(edge_B, w0);
  const float det = a_a * b_b - a_b * a_b;

  glm::vec3 closest_A, closest_B;
  if (std::abs(det) < EPSILON) {
    // Parallel edges - closest point is midpoint of one edge
    closest_A = 0.5f * (p1 + q1);
    closest_B = closest_A;
  } else {
    const float s = glm::clamp((a_b * b_w0 - b_b * a_w0) / det, 0.0f, 1.0f);
    const float t = glm::clamp((a_a * b_w0 - a_b * a_w0) / det, 0.0f, 1.0f);
    closest_A = p1 + edge_A * s;
    closest_B = p2 + edge_B * t;
  }

  // SAT axis (A -> B, B's body space) to world, negated so the manifold normal
  // points B -> A
  const glm::vec3 normal = -glm::normalize(glm::mat3(xfrm_B) * eab.axis);

  Manifold out;
  out.num_points = 1;
  out.norm = normal;

  uint64_t body_id = pack_id(static_cast<uint32_t>(a.entity_id),
                             static_cast<uint32_t>(b.entity_id));
  // Pack the two half-edge indices into 32 bits (each clamped to 16 bits).
  uint32_t vertex_id = (static_cast<uint32_t>(eab.edges.first & 0xFFFF) << 16) |
                       static_cast<uint32_t>(eab.edges.second & 0xFFFF);
  out.contacts[0].pos_world = 0.5f * (closest_A + closest_B);
  out.contacts[0].norm = normal;
  // eab.separation is the signed overlap on this axis (negative = penetrating);
  // negate it to store as a positive penetration depth for Baumgarte.
  out.contacts[0].pen_depth = -eab.separation;
  out.contacts[0].entity_id = body_id;
  out.contacts[0].vertex_id = vertex_id;
  out.contacts[0].a_pos_local =
      glm::vec3(a.xfrm->get_inverse_matrix() *
                glm::vec4(out.contacts[0].pos_world, 1.0f));

  return out;
}

/**
 * @brief Gauss-map arc intersection test (Minkowski face check)
 *
 * Tests whether arcs AB and CD intersect on unit sphere.
 * Reference:
 * https://media.gdcvault.com/gdc2013/slides/822403Gregorius_Dirk_TheSeparatingAxisTest.pdf
 */
bool is_minkowski_face(const glm::vec3 &a, const glm::vec3 &b,
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
glm::vec3 find_support_point(glm::vec3 axis, const HalfEdgeMesh &mesh) {
  const auto &local_verts = mesh.vertices;
  float max_proj = -std::numeric_limits<float>::max();
  glm::vec3 result(0.0f);

  for (size_t i = 0; i < local_verts.size(); i++) {
    float proj = glm::dot(local_verts[i], axis);
    if (proj > max_proj) {
      max_proj = proj;
      result = local_verts[i];
    }
  }

  return result;
}

void clip_against_reference_face(std::vector<ClipVertex> &polygon,
                                 const HalfEdgeMesh &ref_mesh,
                                 const glm::mat4 &ref_transform,
                                 const HalfEdgeMesh::Face &ref_face,
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
clip_polygon_against_plane(const std::vector<ClipVertex> &polygon,
                           const Plane &plane, uint32_t clip_id) {
  std::vector<ClipVertex> out;

  if (polygon.empty())
    return out;

  size_t vertex_count = polygon.size();
  for (size_t i = 0; i < vertex_count; i++) {
    const ClipVertex &first = polygon[i];
    const ClipVertex &second = polygon[(i + 1) % vertex_count];

    float first_distance = get_signed_distance_to_plane(first.pos_world, plane);
    float second_distance =
        get_signed_distance_to_plane(second.pos_world, plane);

    // Current vertex is inside (or on) the plane
    if (first_distance <= 0) {
      out.push_back(first);

      // Edge crosses from inside to outside — emit intersection vertex.
      if (second_distance > 0) {
        float t = first_distance / (first_distance - second_distance);
        ClipVertex intersection;
        intersection.pos_world =
            first.pos_world + (second.pos_world - first.pos_world) * t;
        intersection.id = CLIP_INTERSECTION_FLAG | clip_id;
        out.push_back(intersection);
      }
    }
    // Current vertex is outside, next is inside — emit intersection only.
    else if (second_distance <= 0) {
      float t = first_distance / (first_distance - second_distance);
      ClipVertex intersection;
      intersection.pos_world =
          first.pos_world + (second.pos_world - first.pos_world) * t;
      intersection.id = CLIP_INTERSECTION_FLAG | clip_id;
      out.push_back(intersection);
    }

    // Both outside -- don't add anything
  }

  return out;
}

/*
std::vector<ClipVertex>
clip_polygon_against_plane(const std::vector<ClipVertex> &polygon,
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

size_t find_incident_face(const glm::vec3 &ref_norm_world,
                          const ShapeProperties &inc) {

  // Transform reference normal from world space to inc's body space - rigid
  // inverse, so its rotation alone maps normals
  glm::vec3 ref_norm_local = glm::normalize(
      glm::mat3(inc.xfrm->get_inverse_matrix()) * ref_norm_world);

  const auto &faces = body_mesh(inc).faces;
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
FaceContact bias_reference_face(const ShapeProperties &a,
                                const ShapeProperties &b,
                                const FaceCollision &fa,
                                const FaceCollision &fb) {

  bool a_bias =
      (std::abs(fa.separation - fb.separation) < REFERENCE_BIAS)
          ? a.entity_id < b.entity_id // near-equal: use ID as tiebreak
          : fa.separation > fb.separation + REFERENCE_BIAS;

  if (a_bias)
    return {&a, &b, fa.face};
  else
    return {&b, &a, fb.face};
}

float get_signed_triangle_area(const glm::vec3 &a, const glm::vec3 &b,
                               const glm::vec3 &c, const glm::vec3 &normal) {
  glm::vec3 ab = b - a;
  glm::vec3 ac = c - a;
  glm::vec3 cross = glm::cross(ab, ac);
  return glm::dot(cross, normal);
}

std::vector<Contact> reduce_manifold(std::vector<Contact> contacts,
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

} // namespace

namespace {

glm::vec3 closest_point_on_segment(const glm::vec3 &p, const glm::vec3 &q,
                                   const glm::vec3 &x) {
  const glm::vec3 pq = q - p;
  const float len2 = glm::dot(pq, pq);
  if (len2 < EPSILON) {
    return p;
  }
  const float t = std::clamp(glm::dot(x - p, pq) / len2, 0.0f, 1.0f);
  return p + t * pq;
}

// convex face polygon - projection if inside, else nearest boundary point
glm::vec3 closest_point_on_face(const HalfEdgeMesh &mesh,
                                const HalfEdgeMesh::Face &face,
                                const glm::vec3 &x) {
  const glm::vec3 &n = face.plane.normal;
  const glm::vec3 proj = x - get_signed_distance_to_plane(x, face.plane) * n;

  const std::vector<size_t> verts = mesh.get_face_vertices(face);
  const size_t count = verts.size();

  // inside iff proj is on the same side of every edge - winding agnostic
  bool has_pos = false, has_neg = false;
  for (size_t i = 0; i < count; i++) {
    const glm::vec3 &v0 = mesh.vertices[verts[i]];
    const glm::vec3 &v1 = mesh.vertices[verts[(i + 1) % count]];
    const float side = glm::dot(glm::cross(v1 - v0, proj - v0), n);
    has_pos |= side > 0.0f;
    has_neg |= side < 0.0f;
  }
  if (!(has_pos && has_neg)) {
    return proj;
  }

  glm::vec3 best = proj;
  float best_dist2 = std::numeric_limits<float>::max();
  for (size_t i = 0; i < count; i++) {
    const glm::vec3 q = closest_point_on_segment(
        mesh.vertices[verts[i]], mesh.vertices[verts[(i + 1) % count]], x);
    const glm::vec3 d = x - q;
    const float dist2 = glm::dot(d, d);
    if (dist2 < best_dist2) {
      best_dist2 = dist2;
      best = q;
    }
  }
  return best;
}

} // namespace
