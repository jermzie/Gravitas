#pragma once

#include <GLFW/glfw3.h>
#include <glad/glad.h>
#include <glm/glm.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

#include "../inc/Gui.hpp"
#include "../inc/Mesh.hpp"
#include "../inc/Model.hpp"
#include "../inc/Plane.hpp"
#include "../inc/Shader.hpp"
#include "../physics/RigidBody.hpp"

class Debugger {
  GLuint pointVAO, pointVBO;
  GLuint lineVAO, lineVBO;
  Model arrowHead; // loaded once

  Shader debugShader;

public:
  Debugger() {
    // --- Setup Point ---
    glm::vec3 point(0.0f);
    glGenVertexArrays(1, &pointVAO);
    glGenBuffers(1, &pointVBO);
    glBindVertexArray(pointVAO);
    glBindBuffer(GL_ARRAY_BUFFER, pointVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3), &point, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                          (void *)0);
    glEnableVertexAttribArray(0);

    // --- Setup Line (2 verts) ---
    glm::vec3 lineVerts[2] = {glm::vec3(0), glm::vec3(0, 1, 0)};
    glGenVertexArrays(1, &lineVAO);
    glGenBuffers(1, &lineVBO);
    glBindVertexArray(lineVAO);
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(lineVerts), lineVerts,
                 GL_DYNAMIC_DRAW); // dynamic so you can update
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3),
                          (void *)0);
    glEnableVertexAttribArray(0);

    // Arrow head is loaded from pyramid.obj
    arrowHead.load_model("pyramid.obj");
  }

  void drawPoint(glm::vec3 pos, glm::vec3 color, glm::mat4 VP) {
    debugShader.use();
    debugShader.setVec3("color", color);
    glm::mat4 model = glm::translate(glm::mat4(1.0f), pos);
    debugShader.setMat4("gWVP", VP * model);
    glBindVertexArray(pointVAO);
    glDrawArrays(GL_POINTS, 0, 1);
  }

  void drawVector(glm::vec3 start, glm::vec3 end, glm::vec3 color,
                  glm::mat4 VP) {
    debugShader.use();
    debugShader.setVec3("color", color);
    glm::vec3 lineVerts[2] = {start, end};
    glBindBuffer(GL_ARRAY_BUFFER, lineVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(lineVerts),
                    lineVerts); // update vertices
    glBindVertexArray(lineVAO);
    glDrawArrays(GL_LINES, 0, 2);

    // optionally draw arrow head at 'end'
  }
};

#endif
