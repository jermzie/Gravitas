#pragma once

#include "ConvexMesh.hpp"
#include "RigidBody.hpp"

// START WITH POLYHEDRON AND SPHERES
enum PrimitiveType {
  POLYHEDRON,
  SPHERE,
  CYLINDER,
  CAPSULE,
};

class ConvexPrimitive {
public:
  virtual AABB compute_AABB() = 0;
  virtual PrimitiveType get_type() const = 0;
  virtual RayHit raycast(const Ray &r) const = 0;

private:
  PrimitiveType type;
};

class Polyhedron : ConvexPrimitive {
public:
  Polyhedron() {}
  Polyhedron(const Polyhedron &p);
  Polyhedron(const std::vector<glm::vec3> &points);

  PrimitiveType get_type() const override { return POLYHEDRON; }
};

class Sphere : ConvexPrimitive {
public:
  Sphere() {}
  Sphere(const Sphere &s);
  Sphere(glm::vec3 center, float radius);

  PrimitiveType get_type() const override { return SPHERE; }
};

class Cylinder : ConvexPrimitive {
public:
  Cylinder() {}
  Cylinder(const Cylinder &c);
  Cylinder(glm::vec3 center, glm::vec3 dir, float height, float radius);

  PrimitiveType get_type() const override { return CYLINDER; }
};

class Capsule : ConvexPrimitive {
  Capsule() {}
  Capsule(const Capsule &c);
  Capsule(glm::vec3 center, glm::vec3 direction, float height, float radius);

  PrimitiveType get_type() const override { return CAPSULE; }
};
