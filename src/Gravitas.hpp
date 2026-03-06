#pragma once

#ifndef PROJECT_SOURCE_DIR
#error "PROJECT_SOURCE_DIR is not defined"
#endif

#include <glad/glad.h>

#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <algorithm>
#include <iostream>
#include <vector>

// Rendering
#include "Camera.hpp"
#include "Mesh.hpp"
#include "Model.hpp"
#include "Shader.hpp"

// Debugging
#include "Debugger.hpp"
#include "Gui.hpp"

// Physics
#include "PhysicsEngine.hpp"
#include "RigidBody.hpp"

#include "Plane.hpp"
#include "Ray.hpp"

class Gravitas {
public:
  Gravitas(unsigned int SCREEN_WIDTH = 800, unsigned int SCREEN_HEIGHT = 800);
  ~Gravitas();

  bool init();
  void run();

private:
  // window & context
  GLFWwindow *window = nullptr;
  const char *glsl_version = "#version 330 core";
  unsigned int SCREEN_WIDTH, SCREEN_HEIGHT;

  // timing
  float fixed_dt = 1.0f / 60.0f; // 60 steps per second
  float dt = 0.0f;
  float curr_frame = 0.0f;
  float prev_frame = 0.0f;

  // camera & input
  Camera camera{glm::vec3(0.0f, -25.0f, 20.0f)}; // initial camera pos.
  bool first_mouse = true;                       // initial mouse jittering
  float prev_x, prev_y;                          // previous mouse pos.

  // mouse picking/dragging
  struct MouseState {
    bool is_down = false;
    bool first_mouse = true;
    float x, y, x_offset, y_offset;
  } left_mouse_btn, right_mouse_btn;
  Plane drag_plane;
  bool is_dragging = false;
  bool is_rotating = false;
  int selected_object = -1;
  glm::vec3 old_position;
  glm::vec3 new_position;

  bool dragXZ = false;
  bool dragYZ = false;
  bool dragXY = false;

  // Physics
  PhysicsEngine scene;
  Gui gui;
  Debugger debug;

  bool wireFrame = false;
  bool defaultView = false;

  // shaders
  Shader basicShader;
  Shader picking_shader;
  Shader outline_shader;
  Shader lighting_shader;
  Shader default_shader;

  // Spectral-10 color palette (OpenGL color range 0.0 - 1.0)
  // https://loading.io/color/feature/Spectral-10/
  glm::vec3 color_palette[10] = {
      glm::vec3(158, 1, 66) / 255.0f,
      glm::vec3(213, 62, 79) / 255.0f,
      glm::vec3(244, 109, 67) / 255.0f,
      glm::vec3(253, 174, 97) / 255.0f,
      glm::vec3(254, 224, 139) / 255.0f,
      glm::vec3(230, 245, 152) / 255.0f,
      glm::vec3(171, 221, 164) / 255.0f,
      glm::vec3(102, 194, 165) / 255.0f,
      glm::vec3(50, 136, 189) / 255.0f,
      glm::vec3(94, 79, 162) / 255.0f,
  };

  // transformations
  glm::mat4 projection;
  glm::mat4 view;

  // initialization
  bool init_glfw();
  bool init_glad();
  void init_callbacks();
  void init_scene();

  // main loop
  void process_inputs();
  void update();
  void render();

  // callback handlers
  void on_frame_buf(int width, int height);
  void on_scroll(double xoffset, double yoffset);
  void on_key(int key, int scancode, int action, int mods);
  void on_mouse_move(double xpos, double ypos);
  void on_mouse_press(int button, int action, int mods);

  // static callback wrappers
  static void frame_buf_size_cb(GLFWwindow *window, int width, int height);
  static void scroll_cb(GLFWwindow *window, double xoffset, double yoffset);
  static void key_cb(GLFWwindow *window, int key, int scancode, int action, int mods);
  static void mouse_btn_cb(GLFWwindow *window, int button, int action, int mods);
  static void cursor_pos_cb(GLFWwindow *window, double xpos, double ypos);
};
