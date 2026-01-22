#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include "Transform.hpp"
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

// r = o + d*t
struct Ray {

  glm::vec3 origin;
  glm::vec3 direction;
  float t;
  float inverse_len_squared;

  Ray() = default;
  Ray(glm::vec3 origin, glm::vec3 direction)
      : origin(origin), direction(direction),
        inverse_len_squared(1 / (glm::length2(direction))) {}
};

inline float get_squared_distance_to_ray(const glm::vec3 &p, const Ray &r) {

  // vector between ray origin(o) to point p
  const glm::vec3 op = p - r.origin;

  // projection of vector op onto ray
  float project = glm::dot(op, r.direction);

  return glm::length2(op) - (project * project * r.inverse_len_squared);
}

// use raycasting to project mouse pos. as 3D ray for object picking
inline Ray screen_to_world_ray(float x_pos, float y_pos, int screen_width,
                               int screen_height, glm::mat4 projection_matrix,
                               glm::mat4 view_matrix) {

  // Screen Space --> Clip Space (NDC)
  // Scale screen coordinates to NDC [-1, 1]
  float x = (2.0f * (float)x_pos) / (float)screen_width - 1.0f;
  float y = 1.0f - (2.0f * (float)y_pos) / (float)screen_height;

  // Clip space ray
  glm::vec4 clip_start(x, y, -1.0f, 1.0f);
  glm::vec4 clip_end(x, y, 1.0, 1.0f);

  /*
  // Clip Space --> Camera Space
  // projection matrix transforms ...
  glm::mat4 inverseProjectionMatrix = glm::inverse(projection);

  glm::vec4 rayStart_camera = inverseProjectionMatrix * rayStart_clip;
  glm::vec4 rayEnd_camera = inverseProjectionMatrix * rayEnd_clip;


  rayStart_camera /= rayStart_camera.w;
  rayEnd_camera /= rayEnd_camera.w;

  // Camera Space --> World Space
  // view matrix transforms ...
  glm::mat4 inverseViewMatrix = glm::inverse(view);

  glm::vec4 rayStart_world = inverseProjectionMatrix * rayStart_camera;
  glm::vec4 rayEnd_world = inverseProjectionMatrix * rayEnd_camera;

  rayStart_world /= rayStart_world.w;
  rayEnd_world /= rayEnd_world.w;
  */

  // faster method -- just one inverse operation
  glm::mat4 inverse_vp_matrix = glm::inverse(projection_matrix * view_matrix);
  glm::vec4 world_start = inverse_vp_matrix * clip_start;
  glm::vec4 world_end = inverse_vp_matrix * clip_end;

  // divide by homogeneous component
  world_start /= world_start.w;
  world_end /= world_end.w;

  glm::vec3 world_direction(world_end - world_start);

  Ray out;
  out.origin = glm::vec3(world_start);
  out.direction = glm::normalize(world_direction);

  return out;
}

inline Ray local_to_world_ray(const Transform &transform, const Ray &ray) {

  glm::mat4 model_matrix = transform.get_matrix();
  glm::vec4 world_origin = model_matrix * glm::vec4(ray.origin, 1.0f);
  glm::vec4 world_direction = model_matrix * glm::vec4(ray.direction, 1.0f);

  Ray out;
  out.origin = glm::vec3(world_origin);
  out.direction = glm::vec3(world_direction);

  return out;
}

inline Ray world_to_local_ray(const Transform &transform, const Ray &ray) {

  glm::mat4 inverse_model_matrix = transform.get_inverse_matrix();
  glm::vec4 local_origin = inverse_model_matrix * glm::vec4(ray.origin, 1.0f);
  glm::vec4 local_direction =
      inverse_model_matrix * glm::vec4(ray.direction, 1.0f);

  Ray out;
  out.origin = glm::vec3(local_origin);
  out.direction = glm::vec3(local_direction);

  return out;
}

// Drag body along plane perpendicular to camera
inline bool ray_plane_intersect(const Ray &ray, const glm::vec3 &normal,
                                const glm::vec3 &point, float &t) {

  // Calculate ray-plane intersection
  float denom = glm::dot(normal, ray.direction);

  // Check if ray is not parallel to plane -- avoid zero division
  if (std::abs(denom) < 1e-6) {
    return false;
  }

  t = glm::dot(normal, point - ray.origin) / denom;

  // return t >= 0;

  // min max intersect distances
  return (t >= 0.1f && t < 50.0f);
}
