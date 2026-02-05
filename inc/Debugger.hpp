#pragma once
#include <glad/glad.h>

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

// #include "../inc/Gui.hpp"
#include "../inc/Mesh.hpp"
#include "../inc/Model.hpp"
#include "../inc/Plane.hpp"
#include "../inc/Ray.hpp"
#include "../inc/Shader.hpp"
#include "../physics/AABB.hpp"
#include "../physics/ConvexMesh.hpp"
// #include "../physics/RigidBody.hpp"

class Debugger {

  struct DebugVertex {
    glm::vec3 position;
    glm::vec3 color;
  };

  unsigned int VAO, VBO;
  std::vector<DebugVertex> lines;
  std::vector<DebugVertex> points;
  std::vector<DebugVertex> triangles;

  Shader debug_shader;

public:
  void init() {
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void *)0);

    // Color attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void *)offsetof(DebugVertex, color));

    glBindVertexArray(0);

    debug_shader.init("debug.vert", "debug.frag");
  }

  void new_frame() {
    lines.clear();
    points.clear();
    triangles.clear();
  }

  void draw_vertex(const glm::vec3 &pos, const glm::vec3 &color, float size = 50.0f) { points.push_back({pos, color}); }

  void draw_line(const glm::vec3 &start, const glm::vec3 &end, const glm::vec3 &color) {
    lines.push_back({start, color});
    lines.push_back({end, color});
  }

  void draw_ray(const Ray &ray, float length, const glm::vec3 &color) {
    draw_line(ray.origin, (ray.origin + ray.direction * length), color);
  }

  void draw_triangle(const glm::vec3 &v0, const glm::vec3 &v1, const glm::vec3 &v2, const glm::vec3 &color) {
    draw_line(v0, v1, color);
    draw_line(v1, v2, color);
    draw_line(v2, v0, color);
  }

  // draw polygon face w/ no triangles
  static void draw_unordered_polygon(std::vector<glm::vec3> points, const glm::vec3 &color) {

    if (points.size() < 3)
      return;

    glm::vec3 centroid(0.0f);
    for (const auto &p : points)
      centroid += p;
    centroid /= static_cast<float>(points.size());
  }

  // draw mesh with no triangles
  void draw_mesh(const ConvexMesh &mesh, const glm::mat4 &transform, const glm::vec3 &color) {

    std::unordered_set<size_t> visited_edges;

    for (const ConvexMesh::HalfEdge &he : mesh.half_edges) {

      if (visited_edges.count(he.twin)) {
        continue;
      }

      visited_edges.insert(&he - &mesh.half_edges[0]);
      auto edge_vertices = mesh.get_half_edge_vertices(he);

      glm::vec3 p0 = glm::vec3(transform * glm::vec4(mesh.vertices[edge_vertices[0]], 1.0f));
      glm::vec3 p1 = glm::vec3(transform * glm::vec4(mesh.vertices[edge_vertices[1]], 1.0f));

      draw_line(p0, p1, color);
    }
  }

  void draw_aabb(const AABB &aabb, const glm::vec3 &color) {

    glm::vec3 corner_vertices[8] = {{aabb.min.x, aabb.min.y, aabb.min.z},
        {aabb.max.x, aabb.min.y, aabb.min.z},
        {aabb.max.x, aabb.max.y, aabb.min.z},
        {aabb.min.x, aabb.max.y, aabb.min.z},
        {aabb.min.x, aabb.min.y, aabb.max.z},
        {aabb.max.x, aabb.min.y, aabb.max.z},
        {aabb.max.x, aabb.max.y, aabb.max.z},
        {aabb.min.x, aabb.max.y, aabb.max.z}};

    // Bottom face
    for (int i = 0; i < 4; i++)
      draw_line(corner_vertices[i], corner_vertices[(i + 1) % 4], color);
    // Top face
    for (int i = 4; i < 8; i++)
      draw_line(corner_vertices[i], corner_vertices[4 + (i + 1) % 4], color);
    // Vertical edges
    for (int i = 0; i < 4; i++)
      draw_line(corner_vertices[i], corner_vertices[i + 4], color);
  }

  // draw translucent square face
  void draw_plane(const Plane &plane, float size, const glm::vec3 &color) {}

  void draw_normal() {}

  void render(const glm::mat4 &view, const glm::mat4 &projection, const glm::mat4 &model) {

    if (lines.empty() && points.empty()) {
      return; // Nothing to draw
    }

    // Add option to enable/disable
    // Disable depth test so debug always draws on top
    // glDisable(GL_DEPTH_TEST);

    debug_shader.use();
    // debug_shader.set_vec3("color", glm::vec3(1.0f, 0.0f, 0.0f));
    debug_shader.set_mat4("view", view);
    debug_shader.set_mat4("projection", projection);
    debug_shader.set_mat4("model", model);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    // Draw lines
    if (!lines.empty()) {

      glBufferData(GL_ARRAY_BUFFER, lines.size() * sizeof(DebugVertex), lines.data(), GL_DYNAMIC_DRAW);
      glLineWidth(2.0f);
      glDrawArrays(GL_LINES, 0, lines.size());
    }

    // Draw points
    if (!points.empty()) {
      glBufferData(GL_ARRAY_BUFFER, points.size() * sizeof(DebugVertex), points.data(), GL_DYNAMIC_DRAW);
      glPointSize(5.0f);
      glDrawArrays(GL_POINTS, 0, points.size());
    }

    glBindVertexArray(0);

    // Re-enable depth test
    // glEnable(GL_DEPTH_TEST);
  }
};
