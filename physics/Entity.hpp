#pragma once

#include "Debugger.hpp"
#include "Geometry.hpp"
#include "RigidBody.hpp"
#include <memory>

/*
 Atomic unit in physics engine
 */

class Entity {
public:
  std::unique_ptr<RigidBody> body; // physics object
  std::unique_ptr<Geometry> geom;  // topology

  void draw(Shader &shader, Debugger *debug = nullptr) {}
};
