#pragma once

#include <glm/glm.hpp>

#include <cmath>
#include <unordered_set>
#include <vector>

#include "aabb.hpp"
#include "halfedge_mesh.hpp"
#include "plane.hpp"
#include "ray.hpp"

/*
  Renderer-agnostic debug geometry. The engine appends world-space lines,
  points & triangles here each frame; a renderer reads the buffers and draws
  them however it likes (see demo Debugger for an OpenGL backend).
 */

class DebugDraw {
public:
  struct DebugVertex {
    glm::vec3 position;
    glm::vec4 color;
  };

  std::vector<DebugVertex> lines; // pairs of vertices
  std::vector<DebugVertex> points;
  std::vector<DebugVertex> triangles; // triples of vertices

  void new_frame() {
    lines.clear();
    points.clear();
    triangles.clear();
  }

  void draw_vertex(const glm::vec3 &pos, const glm::vec4 &color,
                   float size = 50.0f) {
    points.push_back({pos, color});
  }

  void draw_line(const glm::vec3 &start, const glm::vec3 &end,
                 const glm::vec4 &color) {
    lines.push_back({start, color});
    lines.push_back({end, color});
  }

  void draw_ray(const Ray &ray, float length, const glm::vec4 &color) {
    draw_line(ray.origin, (ray.origin + ray.direction * length), color);
  }

  void draw_triangle(const glm::vec3 &v0, const glm::vec3 &v1,
                     const glm::vec3 &v2, const glm::vec4 &color) {
    draw_line(v0, v1, color);
    draw_line(v1, v2, color);
    draw_line(v2, v0, color);
  }

  void draw_filled_triangle(const glm::vec3 &v0, const glm::vec3 &v1,
                            const glm::vec3 &v2, const glm::vec4 &color) {
    triangles.push_back({v0, color});
    triangles.push_back({v1, color});
    triangles.push_back({v2, color});
  }

  // draw polygon face w/ no triangles
  void draw_unordered_polygon(std::vector<glm::vec3> points,
                              const glm::vec4 &color) {
    int size = points.size();

    if (size < 3)
      return;

    for (int i = 0; i < size; i++) {
      glm::vec3 p0 = points[i];
      glm::vec3 p1 = points[(i + 1) % size];

      draw_line(p0, p1, glm::vec4(1.0, 0.0, 0.0, 1.0));
    }

    /*
    glm::vec3 centroid(0.0f);
    for (const auto &p : points)
      centroid += p;
    centroid /= static_cast<float>(points.size());
    */
  }

  // draw mesh with no triangles
  void draw_mesh(const HalfEdgeMesh &mesh, const glm::mat4 &transform,
                 const glm::vec4 &color) {

    std::unordered_set<size_t> visited_edges;

    for (const HalfEdgeMesh::HalfEdge &he : mesh.half_edges) {

      if (visited_edges.count(he.twin)) {
        continue;
      }

      visited_edges.insert(&he - &mesh.half_edges[0]);
      auto edge_vertices = mesh.get_half_edge_vertices(he);

      glm::vec3 p0 = glm::vec3(
          transform * glm::vec4(mesh.vertices[edge_vertices[0]], 1.0f));
      glm::vec3 p1 = glm::vec3(
          transform * glm::vec4(mesh.vertices[edge_vertices[1]], 1.0f));

      draw_line(p0, p1, color);
    }
  }

  void draw_aabb(const AABB &aabb, const glm::vec4 &color) {

    glm::vec3 corner_vertices[8] = {{aabb.min.x, aabb.min.y, aabb.min.z},
                                    {aabb.max.x, aabb.min.y, aabb.min.z},
                                    {aabb.max.x, aabb.max.y, aabb.min.z},
                                    {aabb.min.x, aabb.max.y, aabb.min.z},
                                    {aabb.min.x, aabb.min.y, aabb.max.z},
                                    {aabb.max.x, aabb.min.y, aabb.max.z},
                                    {aabb.max.x, aabb.max.y, aabb.max.z},
                                    {aabb.min.x, aabb.max.y, aabb.max.z}};

    // Bottom face
    for (int i = 0; i < 4; i++) {
      draw_line(corner_vertices[i], corner_vertices[(i + 1) % 4], color);
    }
    // Top face
    for (int i = 4; i < 8; i++) {
      draw_line(corner_vertices[i], corner_vertices[4 + (i + 1) % 4], color);
    }
    // Vertical edges
    for (int i = 0; i < 4; i++) {
      draw_line(corner_vertices[i], corner_vertices[i + 4], color);
    }
  }

  // draw translucent square face
  void draw_plane(const Plane &plane, float size, const glm::vec4 &border_color,
                  const glm::vec4 &fill_color, int grid_lines = 10) {

    // Build two orthogonal tangent vectors from the plane normal.
    // Pick an arbitrary "up" that isn't parallel to the normal.
    glm::vec3 n = glm::normalize(plane.normal);
    glm::vec3 ref = (std::abs(n.y) < 0.99f) ? glm::vec3(0.0f, 1.0f, 0.0f)
                                            : glm::vec3(1.0f, 0.0f, 0.0f);
    glm::vec3 t1 = glm::normalize(glm::cross(ref, n)); // tangent
    glm::vec3 t2 = glm::cross(n, t1);                  // bitangent

    glm::vec3 o = plane.point; // center of the drawn section

    // -- Border --
    glm::vec3 c0 = o + (-t1 - t2) * size;
    glm::vec3 c1 = o + (t1 - t2) * size;
    glm::vec3 c2 = o + (t1 + t2) * size;
    glm::vec3 c3 = o + (-t1 + t2) * size;
    draw_line(c0, c1, border_color);
    draw_line(c1, c2, border_color);
    draw_line(c2, c3, border_color);
    draw_line(c3, c0, border_color);

    // -- Grid lines --
    if (grid_lines > 0) {
      // Slightly dimmer colour for interior grid
      // glm::vec3 grid_color = color * 0.45f;
      glm::vec4 grid_color = glm::vec4(1, 1, 1, 1) * 0.45f;

      for (int i = 1; i < grid_lines; ++i) {
        float t = (static_cast<float>(i) / grid_lines) * 2.0f - 1.0f; // [-1, 1]

        // Lines parallel to t2
        glm::vec3 a = o + t1 * (t * size) - t2 * size;
        glm::vec3 b = o + t1 * (t * size) + t2 * size;
        draw_line(a, b, grid_color);

        // Lines parallel to t1
        glm::vec3 c = o - t1 * size + t2 * (t * size);
        glm::vec3 d = o + t1 * size + t2 * (t * size);
        draw_line(c, d, grid_color);
      }
    }

    // -- Normal indicator (short arrow from center) --
    draw_line(o, o + n * (size * 0.15f), border_color);

    draw_filled_triangle(c0, c1, c2, fill_color);
    draw_filled_triangle(c0, c2, c3, fill_color);
  }

  void draw_normal(const Plane &plane, const glm::vec4 &color) {

    glm::vec3 n = glm::normalize(plane.normal);
    glm::vec3 o = plane.point; // center of the drawn section
    draw_line(o, o + n, color);
  }
};
