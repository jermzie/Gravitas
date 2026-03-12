#pragma once

#include "ConvexMesh.hpp"
#include "RigidBody.hpp"

// START WITH POLYHEDRON AND SPHERES
enum GeometryType {
  POLYHEDRON,
  SPHERE,
  CYLINDER,
  CAPSULE,
  CONE,
};

class ConvexGeometry {
public:
  virtual AABB compute_AABB() = 0;
  virtual GeometryType get_type() const = 0;
};

class Sphere : ConvexGeometry {
public:
  Sphere() {}
  Sphere(const Sphere &s);
  Sphere(glm::vec3 center, float radius);

  GeometryType get_type() const override { return SPHERE; }
};

class Cylinder : ConvexGeometry {};
