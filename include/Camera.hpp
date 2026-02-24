#pragma once

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <stdio.h>

// Defines several possible options for camera movement. Used as abstraction to
// stay away from window-system specific input methods
enum CameraMovement { FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN };

// Default camera values
const float YAW = -90.0f;
const float PITCH = 0.0f;
const float SPEED = 5.0f;
const float SENSITIVITY = 0.1f;
const float ZOOM = 45.0f;

// An abstract camera class that processes input and calculates the
// corresponding Euler Angles, Vectors and Matrices for use in OpenGL
class Camera {
public:
  // camera Attributes
  glm::vec3 position;
  glm::vec3 front_vector;
  glm::vec3 up_vector;
  glm::vec3 right_vector;
  glm::vec3 world_up;
  // euler Angles
  float yaw_angle;
  float pitch_angle;
  // camera options
  float movement_speed;
  float mouse_sensitivity;
  float zoom;

  void debug() {
    printf("POSITION: (%.2f, %.2f, %.2f)\n", position.x, position.y, position.z);
    printf("FRONT: (%.2f, %.2f, %.2f)\n", front_vector.x, front_vector.y, front_vector.z);
    printf("UP: (%.2f, %.2f, %.2f)\n", up_vector.x, up_vector.y, up_vector.z);
    printf("WORLD UP: (%.2f, %.2f, %.2f)\n", world_up.x, world_up.y, world_up.z);
    printf("YAW: %.2f\n", yaw_angle);
    printf("PITCH: %.2f\n", pitch_angle);
  }

  // constructor with vectors
  Camera(glm::vec3 pos = glm::vec3(0.0f, 0.0f, 3.0f),
      glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
      float yaw = YAW,
      float pitch = PITCH)
      : front_vector(glm::vec3(0.0f, 0.0f, -1.0f)), movement_speed(SPEED), mouse_sensitivity(SENSITIVITY), zoom(ZOOM) {
    this->position = pos;
    this->world_up = up;
    this->yaw_angle = yaw;
    this->pitch_angle = pitch;
    update_camera_vectors();
  }

  // constructor with scalar values
  Camera(float posX, float posY, float posZ, float upX, float upY, float upZ, float yaw, float pitch)
      : front_vector(glm::vec3(0.0f, 0.0f, -1.0f)), movement_speed(SPEED), mouse_sensitivity(SENSITIVITY), zoom(ZOOM) {
    this->position = glm::vec3(posX, posY, posZ);
    this->world_up = glm::vec3(upX, upY, upZ);
    this->yaw_angle = yaw;
    this->pitch_angle = pitch;
    update_camera_vectors();
  }

  void reset(glm::vec3 pos,
      glm::vec3 front,
      glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f),
      float yaw = YAW,
      float pitch = PITCH) {

    this->position = pos;
    this->front_vector = front;
    this->up_vector = up;
    this->yaw_angle = yaw;
    this->pitch_angle = pitch;

    update_camera_vectors();
  }

  // returns the view matrix calculated using Euler Angles and the LookAt Matrix
  glm::mat4 get_view_matrix() { return glm::lookAt(position, position + front_vector, up_vector); }

  // processes input received from any keyboard-like input system. Accepts input
  // parameter in the form of camera defined ENUM (to abstract it from windowing
  // systems)
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
    // position += up_vector * velocity;
  }

  // processes input received from a mouse input system. Expects the offset
  // value in both the x and y direction.
  void process_mouse_movement(float x_offset, float y_offset, GLboolean limit_pitch = true) {
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

  // calculates the front vector from the Camera's (updated) Euler Angles
  void update_camera_vectors(GLboolean limit_pitch = true) {

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
    right_vector = glm::normalize(glm::cross(front_vector, world_up)); // normalize the vectors, because their length
                                                                       // gets closer to 0 the more you look up or
                                                                       // down which results in slower movement.
    up_vector = glm::normalize(glm::cross(right_vector, front_vector));
  }
};
