#include "RigidBody.hpp"
#include "MassProperties.hpp"

RigidBody::RigidBody(const MassProperties &properties, float density,
                     const glm::vec3 &position, const glm::quat &orientation) {

  this->properties = properties;
  this->position = position;
  this->density = density;
  this->orientation = orientation;

  update_dynamic_xfrm();
}

// Integral is just infinite sum of small intervals
void RigidBody::integrate_velocities(float dt) {

  glm::vec3 gravity(0.0f, -9.81f, 0.0f);
  force_net += gravity * properties.mass;

  // Integrate linear velocity
  // \Delta v(t) = \frac{1}{m} \int F(t) dt
  lin_velocity += properties.inv_mass * force_net * dt;

  // Integrate angular velocity
  // \Delta \omega (t) = I^{-1} \int \tau (t) dt
  glm::mat3 R = glm::mat3_cast(orientation);
  glm::mat3 world_inv_inertia =
      R * properties.inv_inertia_tensor * glm::transpose(R);
  ang_velocity += world_inv_inertia * torque_net * dt;

  // Reset forces
  force_net = glm::vec3(0.0f);
  torque_net = glm::vec3(0.0f);
}

void RigidBody::integrate_positions(float dt) {

  // LINEAR POS
  glm::vec3 dt_position = dt * (lin_velocity + psuedo_lin_velocity);
  position += dt_position;

  // ANGULAR POS
  glm::quat omega_quat(0.0f, (ang_velocity + psuedo_ang_velocity));
  glm::quat dt_orientation = 0.5f * (float)dt * omega_quat * orientation;
  orientation += dt_orientation;
  orientation = glm::normalize(orientation);

  update_dynamic_xfrm();
}
