#pragma once

#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_trigonometric.hpp>
#include <glm/fwd.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <glm/gtx/scalar_multiplication.hpp>
#include <glm/gtx/string_cast.hpp>

#include <vector>

#include "AABB.hpp"
#include "CollisionGeometry.hpp"
#include "ConvexMesh.hpp"
#include "Debugger.hpp"
#include "MassProperties.hpp"
#include "Mesh.hpp"
#include "Model.hpp"
#include "QuickHull.hpp"
#include "Ray.hpp"
#include "Transform.hpp"

struct CollisionData {
  CollisionGeometry hull;
  AABB local_aabb;

  AABB get_world_aabb(const Transform &transform) const {
    return local_aabb.transform_arvo(local_aabb, transform.get_matrix());
  }
};

/*
typedef enum {
  SPHERE,
  CONE,
  CYLINDER,
  CAPSULE,
  POLYHEDRON,
} rb_type_t;
*/

typedef enum { STEEL, RUBBER, WOOD, GLASS, ICE } materials_t;

typedef struct {
  float density;
  float beta;
  float restitution_coeff;
  float friction_coeff;
  glm::vec3 lin_velocity;
  glm::vec3 ang_velocity;
  glm::vec3 position;
  glm::quat orientation;
  glm::vec3 color;
  Model *model;
} rb_config_t;

class RigidBody {
private:
  glm::vec3 force_net = glm::vec3(0.0f);
  glm::vec3 torque_net = glm::vec3(0.0f);

  // Rendering
  std::shared_ptr<Model> render_model; // Just for drawing
  glm::vec3 model_origin_offset;       // Offset from COM to model origin
  glm::vec3 geometric_origin_offset;   // Offset from COM to geometric origin
                                       //

  void update_transform() {
    transform.set_position(position);
    transform.set_orientation(orientation);
  }

public:
  float density;
  float baumgarte_factor = 0.165f;
  float restitution_coeff = 0.0f;
  float friction_coeff = 0.6f;

  glm::vec3 color = glm::vec3(1.0f);

  Transform transform;
  CollisionData collider;
  MassProperties properties;

  // FIXME: TEMPORARY MAKE PUBLIC
  glm::vec3 position;
  glm::quat orientation;
  glm::vec3 lin_velocity = glm::vec3(0.0f);
  glm::vec3 ang_velocity = glm::vec3(0.0f);
  int id;
  bool is_static = false;
  bool is_dragging = false;

  bool raycast(const Ray &world_ray, float &t) const;

  glm::mat4 get_render_matrix() const;

  glm::mat4 get_physics_matrix() const;

  const ConvexMesh &get_mesh() const;

  RigidBody(const std::shared_ptr<Model> &model, const float &density,
            const glm::vec3 &position,
            const glm::quat &orientation = glm::quat(1, 0, 0, 0),
            const bool &is_static = false);

  void apply_point_force(glm::vec3 f, glm::vec3 p);

  void integrate_velocities(float dt);
  void integrate_positions(float dt);

  void drag(glm::vec3 delta);
  void rotate(float x_offset, float y_offset);
  void draw(Shader &shader, Debugger *debug = nullptr);

  glm::mat3 get_world_inverse_inertia() const {
    glm::mat3 R = glm::mat3_cast(orientation);
    return R * properties.inv_inertia_tensor * glm::transpose(R);
  }

  glm::vec3 get_centre_of_mass() const { return position; }

  glm::vec3 get_local_centre_of_mass() const {
    return properties.centre_of_mass;
  }

  glm::vec3 get_local_geometric_centroid() const {
    return geometric_origin_offset;
  }

  glm::vec3 get_geometric_centroid() const {
    return glm::vec3(transform.get_matrix() *
                     glm::vec4(geometric_origin_offset, 1.0f));
  }
};
