#include "Entity.hpp"
#include "Geometry.hpp"
#include "Transform.hpp"
#include <glm/geometric.hpp>

void Entity::make_static() {
  is_static = true;
  body->make_static();
}

void Entity::drag(glm::vec3 delta) {

  body->position += delta;
  body->ang_velocity = glm::vec3(0.0f);
  body->lin_velocity = glm::vec3(0.0f);

  // update xfrm matrix
  body->update_dynamic_xfrm();
  is_dragging = true;
}

void Entity::rotate(const glm::vec3 &x_axis, const glm::vec3 &y_axis,
                    float x_offset, float y_offset) {

  // scale mouse offset
  x_offset *= 0.1f;
  y_offset *= 0.1f;

  // movement along y-axis rotates about x-axis and vice versa
  glm::quat rot_x = glm::angleAxis(-y_offset, x_axis);
  glm::quat rot_y = glm::angleAxis(x_offset, y_axis);
  glm::quat rotation = rot_y * rot_x;

  body->orientation = rotation * body->orientation;
  body->orientation = glm::normalize(body->orientation);
  body->ang_velocity = glm::vec3(0.0f);
  body->lin_velocity = glm::vec3(0.0f);

  // update xfrm matrix
  body->update_dynamic_xfrm();
  is_rotating = true;
}

void Entity::draw(Shader &shader, const glm::mat4 &parent,
                  const std::string &uniform, Debugger *debug) {

  /*
  original draw() fnc

  if (debug) {
    debug->draw_mesh(collider.hull.get_mesh(), dynamic_xfrm.get_matrix(),
                     glm::vec4(0, 1, 0, 1));
  }
  render_model->draw(shader);
  */

  // base model is untransformed - place it in the geometry frame, then the
  // parent (body) frame, same chain the raycast inverts
  for (const auto &child : geom->get_children()) {

    if (debug) {
      debug->draw_aabb(transform_arvo(child->geom_aabb(), parent),
                       glm::vec4(1.0f, 0.5f, 0.0f, 1.0f));
    }

    shader.set_mat4(uniform, parent * child->get_local_xfrm().get_matrix());
    child->get_base_model()->draw(shader);
  }
}

RayHit Entity::raycast(const Ray &world, float t_max) const {
  const Transform &xfrm = body->dynamic_xfrm;

  // world-space -> geometry-space
  RayHit query = geom->raycast(ray_inv_xfrm(xfrm, world), t_max);

  if (query.is_hit) {

    // convert query back to world-space
    query.hit_point = world.origin + world.direction * query.hit_dist;
    query.surface_norm =
        glm::normalize(xfrm.get_normal_matrix() * query.surface_norm);
  }

  return query;
}

AABB Entity::get_aabb() const {
  return transform_arvo(geom->get_aabb(), body->get_physics_matrix());
}

/*
DEPRECATED IMPLEMENTATION
bool RigidBody::raycast(const Ray &world_ray, float &t) const {

  glm::mat4 inv_transform = dynamic_xfrm.get_inverse_matrix();
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
    glm::mat4 world_transform = dynamic_xfrm.get_matrix();
    glm::vec3 local_hit_point = local_ray.origin + local_ray.direction * t;
    // hit_point = glm::vec3(world_transform *
    // glm::vec4(local_hit_point, 1.0f));
    //  hit_point = glm::vec3(world_transform *
    //  glm::vec4(local_hit_point, 1.0f));
    return true;
  }

  return false;
}
*/
