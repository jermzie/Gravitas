#pragma once

#include <glad/glad.h>

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include "Debugger.hpp"
#include "PhysicsEngine.hpp"
#include "RigidBody.hpp"
#include "Shader.hpp"

class Gui {
private:
  PhysicsEngine *scene = nullptr;

public:
  void init(GLFWwindow *window, const char *glsl_version) {
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Setup platform-specific bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Setup imgui dark mode
    ImGui::StyleColorsDark();
  }

  void new_frame() {
    // Feed inputs to imgui, start new frame
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
  }

  void render() {
    // Render GUI
    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  }

  // menu for selecting and editing rigid body properties
  void drop_down_menu(PhysicsEngine *scene = nullptr) {}

  void set_physics_engine(PhysicsEngine *scene = nullptr) {
    this->scene = scene;
  }

  virtual void update() {

    ImGui::Begin("Scene Editor"); // Create a window called "Hello, world!" and
                                  // append into it.

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);

    if (scene != nullptr) {
      ImGui::Text("Broadphase Collision Pairs: %ld",
                  scene->potential_collisions.size());
      ImGui::Text("Narrowphase Contact Manifolds: %ld", scene->contacts.size());

      ImGui::SliderFloat("Beta", &scene->global_beta, 0.0f, 1.0f);
      ImGui::SliderFloat("Restitution", &scene->global_restitution, 0.0f, 1.0f);
      ImGui::SliderFloat("Friction", &scene->global_friction, 0.0f, 1.0f);

      drop_down_menu(scene);
    }

    ImGui::End();
  }

  void shutdown() {
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
  }
};
