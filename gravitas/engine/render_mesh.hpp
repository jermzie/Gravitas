#pragma once

#include <glm/glm.hpp>

#include <vector>

/*
  Renderer-agnostic triangle mesh describing a primitive's visible surface.
  Plain CPU data - a renderer uploads it however it likes (see demo
  Render.hpp for the OpenGL path).
 */

struct SurfaceVertex {
  glm::vec3 position;
  glm::vec3 normal;
  glm::vec2 uv;
};

struct SurfaceMesh {
  std::vector<SurfaceVertex> vertices;
  std::vector<unsigned int> indices; // triangle list
};
