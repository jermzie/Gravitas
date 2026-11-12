#pragma once

#define GLM_ENABLE_EXPERIMENTAL
#include "Plane.hpp"
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

// result from raycasting
struct RayHit {
  bool is_hit;
  glm::vec3 hit_point;
  glm::vec3 surface_norm;
  float hit_dist;
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

  return Ray(glm::vec3(world_start), glm::normalize(world_direction));
}

// direction uses w = 0 (no translation) and is NOT re-normalized, so the ray
// parameter t is identical in both spaces - hits and t_max pass through as-is
inline Ray ray_xfrm(const Transform &xfrm, const Ray &r) {

  glm::mat4 mat = xfrm.get_matrix();
  return Ray(glm::vec3(mat * glm::vec4(r.origin, 1.0f)),
             glm::vec3(mat * glm::vec4(r.direction, 0.0f)));
}

inline Ray ray_inv_xfrm(const Transform &xfrm, const Ray &r) {
  glm::mat4 inv = xfrm.get_inverse_matrix();
  return Ray(glm::vec3(inv * glm::vec4(r.origin, 1.0f)),
             glm::vec3(inv * glm::vec4(r.direction, 0.0f)));
}

// Drag body along plane perpendicular to camera
/*
inline bool ray_plane_intersect(const Ray &ray, const glm::vec3 &normal,
                                const glm::vec3 &point, float &t,
                                float t_max = 500.0f) {
*/
inline bool ray_plane_intersect(const Ray &world, const Plane &plane,
                                RayHit &query, float t_max = 500.0f) {

  // Calculate ray-plane intersection
  float denom = glm::dot(plane.normal, world.direction);

  // Check if ray is not parallel to plane -- avoid zero division
  if (std::abs(denom) < 1e-6) {
    return false;
  }

  float t = glm::dot(plane.normal, plane.point - world.origin) / denom;

  query.hit_dist = t;
  query.is_hit = (t >= 0.1f && t < t_max); // min max intersect distances
  query.hit_point = world.origin + world.direction * t;
  return query.is_hit;
}
