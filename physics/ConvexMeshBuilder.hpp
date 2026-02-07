#pragma once

#include <glm/glm.hpp>

#include <array>
#include <cassert>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include "../inc/Plane.hpp"

// Container for half-edge representation of polygon mesh
// Makes iterating over edges & faces easier
// FIXED: Now supports both triangular AND polygonal faces
class ConvexMeshBuilder {
public:
  struct HalfEdge {

    size_t vert; // end vertex the half-edge points to
    size_t twin; // oppositely oriented half-edge on adjacent face
    size_t face; // index of incident face
    size_t next; // points to next edge on incident face
    size_t prev;

    void disable() { vert = std::numeric_limits<size_t>::max(); }
    bool is_disabled() const { return vert == std::numeric_limits<size_t>::max(); }
  };

  struct Face {

    size_t he; // index of any one of its half-edges
    Plane plane;

    size_t farthest_point;
    float farthest_distance;
    size_t visibility_checked_on_iteration;

    std::uint8_t is_visible_on_current_iteration : 1;
    std::uint8_t in_face_stack : 1;
    std::uint8_t horizon_edges_on_current_iteration : 3; // Bit for each half edge assigned to this face, each being 0
                                                         // or 1 depending on whether the edge belongs to horizon edge
    std::unique_ptr<std::vector<size_t>> points_above_plane;

    Face()
        : he(std::numeric_limits<size_t>::max()), farthest_point(0), farthest_distance(0),
          visibility_checked_on_iteration(0), is_visible_on_current_iteration(0), in_face_stack(0),
          horizon_edges_on_current_iteration(0) {}

    void disable() { he = std::numeric_limits<size_t>::max(); }
    bool is_disabled() const { return he == std::numeric_limits<size_t>::max(); }
  };

  // Mesh data
  std::vector<Face> faces;
  std::vector<HalfEdge> half_edges;

  // Mark deleted faces & half-edges as disabled (don't actually delete)
  // For future reusage (no vector reallocations)
  std::vector<size_t> disabled_faces, disabled_half_edges;

  ConvexMeshBuilder() = default;

  /**
   * @brief Build initial tetrahedron from vertex indices
   */
  void setup(size_t a, size_t b, size_t c, size_t d) {

    faces.clear();
    half_edges.clear();
    disabled_faces.clear();
    disabled_half_edges.clear();

    // tetrahedron has 4 faces & 6 edges
    faces.reserve(4);
    half_edges.reserve(12);

    // create initial halfedges
    // ------------------------
    // face ABC
    HalfEdge AB;
    AB.vert = b;
    AB.twin = 6;
    AB.face = 0;
    AB.next = 1;
    AB.prev = 2;
    half_edges.push_back(AB); // halfEdges[0]

    HalfEdge BC;
    BC.vert = c;
    BC.twin = 9;
    BC.face = 0;
    BC.next = 2;
    BC.prev = 0;
    half_edges.push_back(BC); // halfEdges[1]

    HalfEdge CA;
    CA.vert = a;
    CA.twin = 3;
    CA.face = 0;
    CA.next = 0;
    CA.prev = 1;
    half_edges.push_back(CA); // halfEdges[2]

    // face ACD
    HalfEdge AC;
    AC.vert = c;
    AC.twin = 2;
    AC.face = 1;
    AC.next = 4;
    AC.prev = 5;
    half_edges.push_back(AC); // halfEdges[3]

    HalfEdge CD;
    CD.vert = d;
    CD.twin = 11;
    CD.face = 1;
    CD.next = 5;
    CD.prev = 3;
    half_edges.push_back(CD); // halfEdges[4]

    HalfEdge DA;
    DA.vert = a;
    DA.twin = 7;
    DA.face = 1;
    DA.next = 3;
    DA.prev = 4;
    half_edges.push_back(DA); // halfEdges[5]

    // face BAD
    HalfEdge BA;
    BA.vert = a;
    BA.twin = 0;
    BA.face = 2;
    BA.next = 7;
    BA.prev = 8;
    half_edges.push_back(BA); // halfEdges[6]

    HalfEdge AD;
    AD.vert = d;
    AD.twin = 5;
    AD.face = 2;
    AD.next = 8;
    AD.prev = 6;
    half_edges.push_back(AD); // halfEdges[7]

    HalfEdge DB;
    DB.vert = b;
    DB.twin = 10;
    DB.face = 2;
    DB.next = 6;
    DB.prev = 7;
    half_edges.push_back(DB); // halfEdges[8]

    // face CBD
    HalfEdge CB;
    CB.vert = b;
    CB.twin = 1;
    CB.face = 3;
    CB.next = 10;
    CB.prev = 11;
    half_edges.push_back(CB); // halfEdges[9]

    HalfEdge BD;
    BD.vert = d;
    BD.twin = 8;
    BD.face = 3;
    BD.next = 11;
    BD.prev = 9;
    half_edges.push_back(BD); // halfEdges[10]

    HalfEdge DC;
    DC.vert = c;
    DC.twin = 4;
    DC.face = 3;
    DC.next = 9;
    DC.prev = 10;
    half_edges.push_back(DC); // halfEdges[11]

    // create initial faces
    // --------------------
    Face ABC;
    ABC.he = 0;
    faces.push_back(std::move(ABC));

    Face ACD;
    ACD.he = 3;
    faces.push_back(std::move(ACD));

    Face BAD;
    BAD.he = 6;
    faces.push_back(std::move(BAD));

    Face CBD;
    CBD.he = 9;
    faces.push_back(std::move(CBD));
  }

  /**
   * @brief Add new face to hull. Reuses memory if disabled faces exist, otherwise allocated new memory
   * @return face index
   */
  size_t add_face() {
    if (disabled_faces.size()) {
      size_t idx = disabled_faces.back();

      auto &f = faces[idx];
      assert(f.is_disabled());

      // Clear any leftover conflict list pointer
      f.points_above_plane.reset();

      // Reset face state
      f.he = std::numeric_limits<size_t>::max(); // Set by caller
      f.farthest_point = 0;
      f.farthest_distance = 0.0f;
      f.visibility_checked_on_iteration = 0;
      f.is_visible_on_current_iteration = 0;
      f.in_face_stack = 0;
      f.horizon_edges_on_current_iteration = 0;

      disabled_faces.pop_back();
      return idx;
    }

    faces.emplace_back();
    return faces.size() - 1;
  }

  /**
   * @brief Add new edge to hull. Reuses memory if disabled edges exist, otherwise allocate new memory
   * @return edge index
   */
  size_t add_half_edge() {
    if (disabled_half_edges.size()) {

      size_t idx = disabled_half_edges.back();
      disabled_half_edges.pop_back();
      return idx;
    }

    half_edges.emplace_back();
    return half_edges.size() - 1;
  }

  /**
   * @brief Mark face as disabled. Reuse vector memory for new faces
   * @return pointer to conflict list vertices
   */
  std::unique_ptr<std::vector<size_t>> disable_face(size_t face_index) {

    auto &f = faces[face_index];
    f.disable();
    disabled_faces.push_back(face_index);
    return std::move(f.points_above_plane);
  }

  /**
   * @brief Mark edge as disabled
   */
  void disable_half_edge(size_t heIdx) {

    auto &he = half_edges[heIdx];
    he.disable();
    disabled_half_edges.push_back(heIdx);
  }

  std::vector<size_t> get_face_vertices(const Face &f) const {

    std::vector<size_t> vertices;

    if (f.is_disabled()) {
      return vertices;
    }

    size_t initial_half_edge = f.he;
    size_t current_half_edge = initial_half_edge;
    size_t safety_count = 0;
    const size_t max_iterations = half_edges.size();

    do {
      vertices.push_back(half_edges[current_half_edge].vert);
      current_half_edge = half_edges[current_half_edge].next;

      if (++safety_count > max_iterations) {
        std::cerr << "ERROR: Infinite loop in get_face_vertices" << std::endl;
        break;
      }
    } while (current_half_edge != initial_half_edge);

    return vertices;
  }

  // Deprecated - assumes triangular faces
  std::array<size_t, 3> get_face_vertices_tri(const Face &f) const {

    std::array<size_t, 3> v;

    const HalfEdge *he = &half_edges[f.he];
    v[0] = he->vert;
    he = &half_edges[he->next];
    v[1] = he->vert;
    he = &half_edges[he->next];
    v[2] = he->vert;
    return v;
  }

  std::vector<size_t> get_face_half_edges(const Face &f) const {

    std::vector<size_t> edges;

    if (f.is_disabled()) {
      return edges;
    }

    size_t initial_half_edge = f.he;
    size_t current_half_edge = initial_half_edge;
    size_t safety_count = 0;
    const size_t max_iterations = half_edges.size();

    do {
      edges.push_back(current_half_edge);
      current_half_edge = half_edges[current_half_edge].next;

      if (++safety_count > max_iterations) {
        std::cerr << "ERROR: Infinite loop in get_face_half_edges" << std::endl;
        break;
      }
    } while (current_half_edge != initial_half_edge);

    return edges;
  }

  // Deprecated - assumes triangular faces
  std::array<size_t, 3> get_face_half_edges_tri(const Face &f) const {
    return {f.he, half_edges[f.he].next, half_edges[half_edges[f.he].next].next};
  }

  std::array<size_t, 2> get_half_edge_vertices(const HalfEdge &he) const { return {half_edges[he.twin].vert, he.vert}; }
};
