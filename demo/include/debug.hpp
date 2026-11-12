#pragma once
#include <glad/glad.h>

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include <cstddef>

#include "debug_draw.hpp"
#include "shader.hpp"

// OpenGL backend for DebugDraw - uploads & draws the buffered geometry
class Debugger : public DebugDraw {

  unsigned int vao, vbo;

  Shader debug_shader;

public:
  void init() {
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

    // Position attribute
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void *)0);

    // Color attribute
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, sizeof(DebugVertex), (void *)offsetof(DebugVertex, color));

    glBindVertexArray(0);

    debug_shader.init("debug.vert", "debug.frag");
  }

  void render(const glm::mat4 &view, const glm::mat4 &projection, const glm::mat4 &model) {

    if (lines.empty() && points.empty()) {
      return; // Nothing to draw
    }

    // Add option to enable/disable
    // Disable depth test so debug always draws on top
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    debug_shader.use();
    debug_shader.set_mat4("view", view);
    debug_shader.set_mat4("projection", projection);
    debug_shader.set_mat4("model", model);

    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);

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

    if (!triangles.empty()) {
      glBufferData(GL_ARRAY_BUFFER, triangles.size() * sizeof(DebugVertex), triangles.data(), GL_DYNAMIC_DRAW);
      glDrawArrays(GL_TRIANGLES, 0, triangles.size());
    }

    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    // Re-enable depth test
    glEnable(GL_DEPTH_TEST);
  }
};
