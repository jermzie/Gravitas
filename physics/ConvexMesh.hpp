#pragma once

// NOT DONE
#include <unordered_map>
#include <vector>

#include "ConvexMeshBuilder.hpp"

/*
Half-edge mesh representation of convex hull,

*/

class ConvexMesh {
public:
  struct HalfEdge {
    size_t vert;
    size_t twin;
    size_t face;
    size_t next;
  };

  struct Face {
    size_t he; // Index of one of its half edges
    Plane plane;
  };

  std::vector<glm::vec3> vertices;
  std::vector<Face> faces;
  std::vector<HalfEdge> half_edges;

  ConvexMesh() = default;

  // Build half-edge mesh representation of convex hull
  ConvexMesh(const ConvexMeshBuilder &mesh_builder,
             const std::vector<glm::vec3> &point_cloud) {

    std::unordered_map<size_t, size_t> face_map;
    std::unordered_map<size_t, size_t> half_edge_map;
    std::unordered_map<size_t, size_t> vertex_map;

    size_t i = 0;
    for (const auto &f : mesh_builder.faces) {

      if (!f.is_disabled()) {

        faces.push_back(
            {static_cast<size_t>(f.he), static_cast<Plane>(f.plane)});
        // faces.push_back({ static_cast<size_t>(f.he) });
        face_map[i] = faces.size() - 1;

        const auto half_edge_indices = mesh_builder.get_face_half_edges(f);
        for (const auto he : half_edge_indices) {

          const size_t vertex_index_map = mesh_builder.half_edges[he].vert;

          if (vertex_map.count(vertex_index_map) == 0) {
            vertices.push_back(point_cloud[vertex_index_map]);
            vertex_map[vertex_index_map] = vertices.size() - 1;
          }
        }
      }
      i++;
    }

    i = 0;
    for (const auto &he : mesh_builder.half_edges) {
      if (!he.is_disabled()) {

        half_edges.push_back(
            {static_cast<size_t>(he.vert), static_cast<size_t>(he.twin),
             static_cast<size_t>(he.face), static_cast<size_t>(he.next)});
        half_edge_map[i] = half_edges.size() - 1;
      }
      i++;
    }

    for (auto &f : faces) {
      assert(half_edge_map.count(f.he) == 1);
      f.he = half_edge_map[f.he];
    }

    for (auto &he : half_edges) {
      he.face = face_map[he.face];
      he.twin = half_edge_map[he.twin];
      he.next = half_edge_map[he.next];
      he.vert = vertex_map[he.vert];
    }
  }

  std::array<size_t, 3> get_face_vertices(const Face &f) const {

    std::array<size_t, 3> v;

    const HalfEdge *he = &half_edges[f.he];
    v[0] = he->vert;
    he = &half_edges[he->next];
    v[1] = he->vert;
    he = &half_edges[he->next];
    v[2] = he->vert;
    return v;
  }

  std::array<size_t, 3> get_face_half_edges(const Face &f) const {
    return {f.he, half_edges[f.he].next,
            half_edges[half_edges[f.he].next].next};
  }

  std::array<size_t, 2> get_half_edge_vertices(const HalfEdge &he) const {
    return {half_edges[he.twin].vert, he.vert};
  }

  glm::vec3 compute_geometric_centroid() const {

    // Compute geometric center
    glm::vec3 centroid(0.0f);
    for (const auto &v : vertices) {
      centroid += v;
    }
    return centroid / static_cast<float>(vertices.size());

    /*
    // Shift all vertices so centroid is at origin
    for (auto &v : vertices) {
      v -= centroid;
    }

    // Return offset for rendering (to undo the shift)
    return centroid;
    */
  }

  const std::array<float, 6> get_extrema() const {

    std::array<size_t, 6> extrema_indices{0, 0, 0, 0, 0, 0};
    std::array<float, 6> extrema_vertices{vertices[0].x, vertices[0].x,
                                          vertices[0].y, vertices[0].y,
                                          vertices[0].z, vertices[0].z};

    for (size_t i = 1; i < vertices.size(); i++) {

      const glm::vec3 &pos = vertices[i];

      // X-axis
      if (pos.x > extrema_vertices[0]) {
        extrema_vertices[0] = pos.x;
        extrema_indices[0] = i;
      }

      else if (pos.x < extrema_vertices[1]) {
        extrema_vertices[1] = pos.x;
        extrema_indices[1] = i;
      }

      // Y-axis
      if (pos.y > extrema_vertices[2]) {
        extrema_vertices[2] = pos.y;
        extrema_indices[2] = i;
      }

      else if (pos.y < extrema_vertices[3]) {
        extrema_vertices[3] = pos.y;
        extrema_indices[3] = i;
      }

      // Z-axis
      if (pos.z > extrema_vertices[4]) {
        extrema_vertices[4] = pos.z;
        extrema_indices[4] = i;
      }

      else if (pos.z < extrema_vertices[5]) {
        extrema_vertices[5] = pos.z;
        extrema_indices[5] = i;
      }
    }

    return extrema_vertices;
  }
};
