#pragma once

// NEEDS TESTING
//
#include <glm/geometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <glm/matrix.hpp>

class Transform {

private:
  glm::vec3 position{0.0f};
  glm::quat orientation{1.0f, 0.0f, 0.0f, 0.0f};
  glm::vec3 scale{1.0f};

  mutable glm::mat4 cached_matrix{1.0f};
  mutable glm::mat4 cached_inverse{1.0f};
  mutable glm::mat3 cached_normal{1.0f};
  mutable bool dirty = true;

public:
  glm::mat4 get_matrix() const {
    if (dirty) {
      glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
      glm::mat4 R = glm::mat4_cast(orientation);
      glm::mat4 S = glm::scale(glm::mat4(1.0f), scale);
      cached_matrix = T * R * S;
      cached_inverse = glm::inverse(cached_matrix);
      cached_normal = glm::transpose(cached_inverse);
      dirty = false;
    }
    return cached_matrix;
  }

  glm::mat4 get_inverse_matrix() const {
    if (dirty) {
      get_matrix();
    }
    return cached_inverse;
  }

  glm::mat3 get_normal_matrix() const {
    if (dirty) {
      get_matrix();
    }
    return cached_normal;
  }

  glm::vec3 local_to_world(glm::vec3 local) const { return position + (orientation * (scale * local)); }

  glm::vec3 world_to_local(glm::vec3 world) const {
    glm::vec3 relative = world - position;
    return glm::inverse(orientation) * (relative / scale);
  }

  Transform() = default;

  void set_position(glm::vec3 pos) {
    position = pos;
    dirty = true;
  }

  void set_displacement(glm::vec3 delta) {
    position += delta;
    dirty = true;
  }

  void set_scale(glm::vec3 s) {
    scale = s;
    dirty = true;
  }

  void set_orientation(glm::quat rot) {
    orientation = rot;
    dirty = true;
  }

  void set_rotation(glm::quat delta) {
    orientation = glm::normalize(delta * orientation);
    dirty = true;
  }
};
