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
#include "Debugger.hpp"
#include "HalfEdgeMesh.hpp"
#include "MassProperties.hpp"
#include "Mesh.hpp"
#include "Model.hpp"
#include "QuickHull.hpp"
#include "Ray.hpp"
#include "Transform.hpp"

class RigidBody {
private:
  glm::vec3 force_net = glm::vec3(0.0f);
  glm::vec3 torque_net = glm::vec3(0.0f);

public:
  float density;
  float baumgarte_factor = 0.165f; // how-fast-positional-errors-get-corrected
  float restitution_coeff = 0.0f;  // bounciness
  float friction_coeff = 0.6f;     // friction

  void update_dynamic_xfrm() {
    dynamic_xfrm.set_position(position);
    dynamic_xfrm.set_orientation(orientation);
  }

  Transform dynamic_xfrm;
  MassProperties properties;

  // FIXME: TEMPORARY MAKE PUBLIC
  glm::vec3 position;
  glm::quat orientation;
  glm::vec3 lin_velocity = glm::vec3(0.0f);
  glm::vec3 ang_velocity = glm::vec3(0.0f);
  glm::vec3 psuedo_lin_velocity = glm::vec3(0.0f);
  glm::vec3 psuedo_ang_velocity = glm::vec3(0.0f);

  // mass properties are expected in body (COM-centered) space - see
  // Geometry::finalize()
  RigidBody(const MassProperties &properties, float density,
            const glm::vec3 &position,
            const glm::quat &orientation = glm::quat(1, 0, 0, 0));

  // infinite mass - solver impulses leave the body untouched
  void make_static() {
    properties.inv_mass = 0.0f;
    properties.inv_inertia_tensor = glm::mat3(0.0f);
    lin_velocity = ang_velocity = glm::vec3(0.0f);
    psuedo_lin_velocity = psuedo_ang_velocity = glm::vec3(0.0f);
  }

  void apply_point_force(glm::vec3 f, glm::vec3 p);

  void integrate_velocities(float dt);
  void integrate_positions(float dt);

  glm::mat3 get_world_inverse_inertia() const {
    glm::mat3 R = glm::mat3_cast(orientation);
    return R * properties.inv_inertia_tensor * glm::transpose(R);
  }

  glm::vec3 get_centre_of_mass() const { return position; }
  glm::mat4 get_physics_matrix() const { return dynamic_xfrm.get_matrix(); }
};
