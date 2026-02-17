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

public:
  glm::mat4 get_matrix() const {

    glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
    glm::mat4 R = glm::mat4_cast(orientation);
    glm::mat4 S = glm::scale(glm::mat4(1.0f), scale);
    return T * R * S;
  }
  glm::mat4 get_matrix() {

    glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
    glm::mat4 R = glm::mat4_cast(orientation);
    glm::mat4 S = glm::scale(glm::mat4(1.0f), scale);
    return T * R * S;
  }

  glm::mat4 get_inverse_matrix() const { return glm::inverse(get_matrix()); }

  glm::vec3 local_to_world(glm::vec3 local) const {
    return position + (orientation * (scale * local));
  }

  glm::vec3 world_to_local(glm::vec3 world) const {
    glm::vec3 relative = world - position;
    return glm::inverse(orientation) * (relative / scale);
  }

  Transform() = default;

  void set_position(glm::vec3 pos) { position = pos; }

  void set_displacement(glm::vec3 delta) { position += delta; }

  void set_scale(glm::vec3 s) { scale = s; }

  void set_orientation(glm::quat rot) { orientation = rot; }

  void set_rotation(glm::quat delta) {
    orientation = delta * orientation;
    orientation = glm::normalize(orientation);
  }
};
