#pragma once

#include <algorithm>
#include <memory>

#include "AABB.hpp"
#include "Config.hpp"
#include "HalfEdgeMesh.hpp"
#include "MassProperties.hpp"
#include "Model.hpp"
#include "Ray.hpp"
#include "Transform.hpp"

/*
  Interface for defining & transforming basic convex primitive shapes.
 */

enum PrimitiveType {
  POLYHEDRON,
  SPHERE,
  FRUSTUM,
  CAPSULE,
};

class Primitive {
protected:
  Transform local_xfrm;              // placement within the parent Geometry
  std::shared_ptr<Model> base_model; // untransformed base model

  // untransformed base frame centered at origin (local space)
  virtual AABB base_aabb() const = 0;
  virtual RayHit base_raycast(const Ray &r, float t_max) const = 0;
  virtual glm::vec3 base_support(glm::vec3 dir) const = 0;

public:
  virtual ~Primitive() = default;

  Primitive &translate(glm::vec3 offset) {
    local_xfrm.set_displacement(offset);
    return *this;
  }
  Primitive &rotate(glm::quat delta) {
    local_xfrm.set_rotation(delta);
    return *this;
  }
  virtual Primitive &scale(glm::vec3 factor) {
    local_xfrm.set_scale(factor);
    return *this;
  }

  const Transform &get_local_xfrm() const { return local_xfrm; }
  std::shared_ptr<Model> get_base_model() const { return base_model; }

  virtual PrimitiveType get_type() const = 0;
  virtual MassProperties compute_mass_properties(float density) const = 0;

  // placed within parent geometry frame - local_xfrm applied to base model
  AABB geom_aabb() const;
  RayHit geom_raycast(const Ray &r, float t_max = 500.0f) const;
  glm::vec3 geom_support(glm::vec3 dir) const;
};

class Polyhedron : public Primitive {
private:
  // special case - polyhedron can essentially represent any model topology
  HalfEdgeMesh topology;

  static const HalfEdgeMesh &get_cube_hull();
  static const HalfEdgeMesh &get_tetrahedron_hull();
  static std::shared_ptr<Model> get_cube_model();
  static std::shared_ptr<Model> get_tetrahedron_model();

protected:
  virtual AABB base_aabb() const override {
    return AABB(topology.get_extrema());
  }
  virtual RayHit base_raycast(const Ray &r, float t_max) const override;
  virtual glm::vec3 base_support(glm::vec3 dir) const override;

public:
  Polyhedron() { generate_cube(); }
  Polyhedron(const std::shared_ptr<Model> &model);

  virtual PrimitiveType get_type() const override { return POLYHEDRON; }
  virtual MassProperties compute_mass_properties(float density) const override;

  const HalfEdgeMesh &get_mesh() const { return topology; }

  // hardcoded primitive definitions
  void generate_cube();
  void generate_tetrahedron();
};

class Sphere : public Primitive {
private:
  float radius = 1.0f;

protected:
  virtual AABB base_aabb() const override {
    return AABB(glm::vec3(-radius), glm::vec3(radius));
  }
  virtual RayHit base_raycast(const Ray &r, float t_max) const override;
  virtual glm::vec3 base_support(glm::vec3 dir) const override;

public:
  Sphere() { generate_sphere(); }
  Sphere(float radius);

  // uniform scale only - non-uniform transforms sphere into an ellipsoid
  virtual Primitive &scale(glm::vec3 factor) override;

  virtual PrimitiveType get_type() const override { return SPHERE; }
  virtual MassProperties compute_mass_properties(float density) const override;

  float get_radius() const { return radius; }

  // hardcoded model definitions
  void generate_sphere(int rings = 16, int slices = 32, float radius = 1.0f);
};

// frustum defines both cone (r_top == 0) and cylinder (r_top == r_bot)
class Frustum : public Primitive {
private:
  float r_bot = 1.0f;
  float r_top = 0.5f;
  float half_height = 1.0f;

  static constexpr int min_slices = 16;

protected:
  virtual AABB base_aabb() const override {
    float r_max = std::max(r_bot, r_top);
    return AABB(glm::vec3(-r_max, -half_height, -r_max),
                glm::vec3(r_max, half_height, r_max));
  }
  virtual RayHit base_raycast(const Ray &r, float t_max) const override;
  virtual glm::vec3 base_support(glm::vec3 dir) const override;

public:
  Frustum() { generate_frustum(); }
  Frustum(float r_bot, float r_top, float half_height);

  // radial scale must be uniform (x == z) - otherwise the cross-section becomes
  // an ellipse; y is free and just stretches the height
  virtual Primitive &scale(glm::vec3 factor) override;

  virtual PrimitiveType get_type() const override { return FRUSTUM; }
  virtual MassProperties compute_mass_properties(float density) const override;

  float get_bottom_radius() const { return r_bot; }
  float get_top_radius() const { return r_top; }
  float get_half_height() const { return half_height; }

  // hardcoded model definitions
  void generate_frustum(int slices = 32, float r_bottom = 1.0f,
                        float r_top = 0.5f, float half_height = 1.0f);
};

// TODO: no impl for base_aabb/base_raycast/base_support and
// compute_mass_properties
class Capsule : public Primitive {
private:
  float radius = 0.5f;
  float half_height = 0.5f; // of the cylindrical section, excluding the caps

public:
  PrimitiveType get_type() const override { return CAPSULE; }

  float get_radius() const { return radius; }
  float get_half_height() const { return half_height; }

  // hardcoded model definitions
  void generate_capsule(int cap_rings = 8, int slices = 32, float radius = 0.5f,
                        float half_height = 0.5f);
};
