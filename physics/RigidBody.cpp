#include "RigidBody.hpp"

bool RigidBody::raycast(const Ray &world_ray, float &t) const {

  glm::mat4 inv_transform = transform.get_inverse_matrix();
  // Ray local_ray = transform.world_to_local(world_ray);
  // return collision.mesh.ray_intersect(local_ray, t);

  Ray local_ray;
  local_ray.origin =
      glm::vec3(inv_transform * glm::vec4(world_ray.origin, 1.0f));
  local_ray.direction =
      glm::vec3(inv_transform * glm::vec4(world_ray.direction, 0.0f));

  // glm::vec3 local_hit_point;
  int hit = collider.hull.raycast(local_ray, t);

  if (hit != 0) {
    // glm::mat4 world_transform = const_cast<Transform
    // &>(transform).get_matrix();
    glm::mat4 world_transform = transform.get_matrix();
    glm::vec3 local_hit_point = local_ray.origin + local_ray.direction * t;
    // hit_point = glm::vec3(world_transform *
    // glm::vec4(local_hit_point, 1.0f));
    //  hit_point = glm::vec3(world_transform *
    //  glm::vec4(local_hit_point, 1.0f));
    return true;
  }

  return false;
}

glm::mat4 RigidBody::get_render_matrix() const {

  glm::mat4 base = transform.get_matrix(); // Local COM space -> World space
  glm::mat4 offset = glm::translate(glm::mat4(1.0f), -model_origin_offset);
  return base * offset; // Model -> COM -> World
}

glm::mat4 RigidBody::get_physics_matrix() const {
  // glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
  // glm::mat4 R = glm::mat4_cast(orientation);
  // return T * R;
  return transform.get_matrix(); // Local COM space -> World space
}

const ConvexMesh &RigidBody::get_mesh() const {
  return collider.hull.get_mesh();
}

RigidBody::RigidBody(const std::shared_ptr<Model> &model, const float &density,
                     const glm::vec3 &position, const glm::quat &orientation,
                     const bool &is_static) {

  this->render_model = model;
  this->position = position;
  this->density = density;
  this->orientation = glm::quat(1, 0, 0, 0);
  this->is_static = is_static;

  // 1. Build collision geometry
  ConvexMesh mesh = CollisionGeometry::get_mesh_cache(
      render_model->file_name, render_model->get_vertex_data());

  // 2. Compute mass properties
  properties = MassProperties::compute(mesh, density);
  model_origin_offset = properties.centre_of_mass;

  // 3. Shift vertices so COM centered at origin
  for (glm::vec3 &v : mesh.vertices) {
    v -= properties.centre_of_mass;
  }

  // 3.5 UPDATE PLANES TO MATCH NEW VERTEX POSITIONS
  for (auto &face : mesh.faces) {
    // Recalculate plane.point using shifted vertices
    auto verts = mesh.get_face_vertices(face);
    face.plane.point = mesh.vertices[verts[0]];

    // Recalculate plane.distance (assuming normal is still correct)
    face.plane.distance = -glm::dot(face.plane.normal, face.plane.point);
  }
  // 4. Recompute mass properties (COM should be ~(0, 0, 0))
  properties = MassProperties::compute(mesh, density);
  geometric_origin_offset = mesh.compute_geometric_centroid();

  // 5. Build collision geometry
  collider.hull = CollisionGeometry(mesh);
  collider.local_aabb = AABB(mesh.get_extrema());

  // 6. Transform into world space
  update_transform();
}

void RigidBody::apply_point_force(glm::vec3 f, glm::vec3 p) {
  force_net += f;
  glm::vec3 r = p - position;
  torque_net += glm::cross(r, f);
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

  if (is_static || is_dragging) {

    // DO SOMETHING
    return;
  } else {

    // LINEAR POS
    glm::vec3 dt_position = dt * lin_velocity;
    position += dt_position;

    // ANGULAR POS
    glm::quat omega_quat(0.0f, ang_velocity);
    glm::quat dt_orientation = 0.5f * (float)dt * omega_quat * orientation;
    orientation += dt_orientation;
    orientation = glm::normalize(orientation);

    update_transform();
  }
}

void RigidBody::drag(glm::vec3 delta) {

  // Drag rigid body
  position += delta;

  ang_velocity = glm::vec3(0.0f);
  lin_velocity = glm::vec3(0.0f);

  // Update transformation matrix
  update_transform();
}

void RigidBody::rotate(float x_offset, float y_offset) {

  x_offset *= 0.01f;
  y_offset *= 0.01f;

  // Movement along y-axis rotates about x-axis and vice versa
  glm::quat rot_x = glm::angleAxis(-y_offset, glm::vec3(1.0f, 0.0f, 0.0f));
  glm::quat rot_y = glm::angleAxis(x_offset, glm::vec3(0.0f, 1.0f, 0.0f));
  glm::quat rotation = rot_y * rot_x;

  // transform.set_rotation(rotation);

  orientation = rotation * orientation;
  orientation = glm::normalize(orientation);

  update_transform();
}

void RigidBody::draw(Shader &shader, Debugger *debug) {

  if (debug) {
    debug->draw_mesh(collider.hull.get_mesh(), transform.get_matrix(),
                     glm::vec4(0, 1, 0, 1));
  }
  render_model->draw(shader);
}
