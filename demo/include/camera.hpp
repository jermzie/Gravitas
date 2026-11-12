#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <stdio.h>

enum CameraMovement { FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN };

// Default camera values
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 25.0f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

class Camera {
public:
  // camera attributes
  glm::vec3 position;
  glm::vec3 front_vector;
  glm::vec3 up_vector;
  glm::vec3 right_vector;
  glm::vec3 world_up;

  // euler angles
  float yaw_angle;
  float pitch_angle;

  // camera options
  float movement_speed;
  float mouse_sensitivity;
  float zoom;

  Camera(glm::vec3 pos = glm::vec3(0.0f),
         glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = YAW,
         float pitch = PITCH, glm::vec3 front = glm::vec3(0.0f, 0.0f, -1.0f),
         float speed = SPEED, float mouse_sens = SENSITIVITY,
         float mouse_zoom = ZOOM) {

    this->front_vector = front;
    this->movement_speed = speed;
    this->mouse_sensitivity = mouse_sens;
    this->zoom = mouse_zoom;
    this->position = pos;
    this->world_up = up;
    this->yaw_angle = yaw;
    this->pitch_angle = pitch;

    update_camera_vectors();
  }

  void reset(glm::vec3 pos, glm::vec3 front,
             glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f), float yaw = YAW,
             float pitch = PITCH) {

    this->position = pos;
    this->front_vector = front;
    this->up_vector = up;
    this->yaw_angle = yaw;
    this->pitch_angle = pitch;

    update_camera_vectors();
  }

  glm::mat4 get_view_matrix() {
    return glm::lookAt(position, position + front_vector, up_vector);
  }

  void process_keyboard(CameraMovement direction, float dt) {
    float velocity = movement_speed * dt;
    if (direction == FORWARD)
      position += front_vector * velocity;
    if (direction == BACKWARD)
      position -= front_vector * velocity;
    if (direction == LEFT)
      position -= right_vector * velocity;
    if (direction == RIGHT)
      position += right_vector * velocity;
    if (direction == UP)
      position += world_up * velocity;
    if (direction == DOWN)
      position -= world_up * velocity;
  }

  void process_mouse_movement(float x_offset, float y_offset,
                              bool limit_pitch = true) {
    x_offset *= mouse_sensitivity;
    y_offset *= mouse_sensitivity;

    yaw_angle += x_offset;
    pitch_angle += y_offset;

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (limit_pitch) {
      if (pitch_angle > 89.0f)
        pitch_angle = 89.0f;
      if (pitch_angle < -89.0f)
        pitch_angle = -89.0f;
    }

    // update Front, Right and Up Vectors using the updated Euler angles
    update_camera_vectors();
  }

  // processes input received from a mouse scroll-wheel event. Only requires
  // input on the vertical wheel-axis
  void process_mouse_scroll(float y_offset) {
    zoom -= (float)(y_offset);
    if (zoom < 1.0f)
      zoom = 1.0f;
    if (zoom > 45.0f)
      zoom = 45.0f;
  }

  // calculates the front vector from the updated Euler Angles
  void update_camera_vectors(bool limit_pitch = true) {

    // make sure that when pitch is out of bounds, screen doesn't get flipped
    if (limit_pitch) {
      if (pitch_angle > 89.0f)
        pitch_angle = 89.0f;
      if (pitch_angle < -89.0f)
        pitch_angle = -89.0f;
    }

    // calculate the new Front vector
    glm::vec3 new_front;
    new_front.x = cos(glm::radians(yaw_angle)) * cos(glm::radians(pitch_angle));
    new_front.y = sin(glm::radians(pitch_angle));
    new_front.z = sin(glm::radians(yaw_angle)) * cos(glm::radians(pitch_angle));
    front_vector = glm::normalize(new_front);

    // also re-calculate the Right and Up vector
    right_vector = glm::normalize(glm::cross(front_vector, world_up));
    up_vector = glm::normalize(glm::cross(right_vector, front_vector));
  }
};
