#pragma once

#include <algorithm>
#include <cassert>
#include <memory>
#include <string>
#include <vector>

#include "AABB.hpp"
#include "Log.hpp"
#include "HalfEdgeMesh.hpp"
#include "MassProperties.hpp"
#include "Ray.hpp"
#include "SurfaceMesh.hpp"
#include "Transform.hpp"

/*
  Interface for defining & transforming basic convex primitive shapes.
 */

namespace hull {
// convex hull of `points`, cached on disk under `cache_dir` keyed by the
// path & mtime of `source_file` (the file the points were loaded from)
HalfEdgeMesh load_or_build(const std::vector<glm::vec3> &points,
                           const std::string &source_file,
                           const std::string &cache_dir);
} // namespace hull

enum PrimitiveType {
  POLYHEDRON,
  SPHERE,
  FRUSTUM,
  CAPSULE,
  PRIM_COUNT,
};

class Primitive {
protected:
  Transform local_xfrm; // placement within the parent Geometry object
  // untransformed render surface - null if the caller supplies visuals
  std::shared_ptr<const SurfaceMesh> base_surface;

  // set by finalize() - shape & placement are frozen from then on
  bool finalized = false;

  // untransformed base frame centered at origin (local space)
  virtual AABB base_aabb() const = 0;
  virtual RayHit base_raycast(const Ray &r, float t_max) const = 0;
  virtual glm::vec3 base_support(glm::vec3 dir) const = 0;

  void assert_mutable() const {
    assert(!finalized && "Primitive modified after Geometry::finalize()");
  }

public:
  virtual ~Primitive() = default;

  Primitive &translate(glm::vec3 offset) {
    assert_mutable();
    local_xfrm.set_displacement(offset);
    return *this;
  }
  Primitive &rotate(glm::quat delta) {
    assert_mutable();
    local_xfrm.set_rotation(delta);
    return *this;
  }
  virtual Primitive &scale(glm::vec3 factor) {
    assert_mutable();
    local_xfrm.set_scale(factor);
    return *this;
  }

  // freeze shape & placement - called by Geometry::finalize() after the COM
  // shift. derived shapes bake anything that depends on local_xfrm here
  virtual void finalize() { finalized = true; }

  const Transform &get_local_xfrm() const { return local_xfrm; }
  const std::shared_ptr<const SurfaceMesh> &get_base_surface() const {
    return base_surface;
  }

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

  // geom_mesh() baked by finalize() - body-space hull for narrowphase
  HalfEdgeMesh geom_hull;

  static const HalfEdgeMesh &get_cube_hull();
  static const HalfEdgeMesh &get_tetrahedron_hull();
  static std::shared_ptr<const SurfaceMesh> get_cube_surface();
  static std::shared_ptr<const SurfaceMesh> get_tetrahedron_surface();

protected:
  virtual AABB base_aabb() const override {
    return AABB(topology.get_extrema());
  }
  virtual RayHit base_raycast(const Ray &r, float t_max) const override;
  virtual glm::vec3 base_support(glm::vec3 dir) const override;

public:
  Polyhedron() { generate_cube(); }
  // convex hull of an arbitrary point cloud
  explicit Polyhedron(const std::vector<glm::vec3> &points,
                      std::shared_ptr<const SurfaceMesh> surface = nullptr);
  // prebuilt hull, e.g. from hull::load_or_build()
  explicit Polyhedron(HalfEdgeMesh hull,
                      std::shared_ptr<const SurfaceMesh> surface = nullptr);

  virtual PrimitiveType get_type() const override { return POLYHEDRON; }
  virtual MassProperties compute_mass_properties(float density) const override;
  virtual void finalize() override;

  const HalfEdgeMesh &get_mesh() const { return topology; }

  // topology with parent * local_xfrm baked into vertices & face planes
  HalfEdgeMesh geom_mesh(const glm::mat4 &parent = glm::mat4(1.0f)) const;

  // geom_mesh() (parent = identity) baked at finalize - body-space hull for
  // narrowphase, keeps the body matrix rigid
  const HalfEdgeMesh &get_geom_mesh() const {
    assert(finalized && "get_geom_mesh() before Geometry::finalize()");
    return geom_hull;
  }

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

// capsule along +Y: every point within radius of the segment (0, +-h, 0)
class Capsule : public Primitive {
private:
  float radius = 0.5f;
  float half_height = 0.5f; // of cylindrical section, excluding caps

protected:
  virtual AABB base_aabb() const override {
    float y_max = half_height + radius;
    return AABB(glm::vec3(-radius, -y_max, -radius),
                glm::vec3(radius, y_max, radius));
  }
  virtual RayHit base_raycast(const Ray &r, float t_max) const override;
  virtual glm::vec3 base_support(glm::vec3 dir) const override;

public:
  Capsule() { generate_capsule(); }
  Capsule(float radius, float half_height);

  // uniform radial scale (x == z) - otherwise cross-section becomes an ellipse
  // y stretches the cylinder and turns the caps into half-spheroids
  virtual Primitive &scale(glm::vec3 factor) override;

  virtual PrimitiveType get_type() const override { return CAPSULE; }
  virtual MassProperties compute_mass_properties(float density) const override;

  float get_radius() const { return radius; }
  float get_half_height() const { return half_height; }

  // hardcoded model definitions
  void generate_capsule(int cap_rings = 8, int slices = 32, float radius = 0.5f,
                        float half_height = 0.5f);
};
