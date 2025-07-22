#pragma once
#ifndef PICKING_TEST_HPP
#define PICKING_TEST_HPP

#ifndef PROJECT_SOURCE_DIR
#error "PROJECT_SOURCE_DIR is not defined"
#endif


#include <iostream>
#include <vector>

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "./inc/model.h"
#include "./inc/mesh.h"
#include "./inc/camera.h"
#include "./inc/shader.h"
#include "./inc/use-imgui.h"
#include "./inc/MousePicking.hpp"
#include "./physics/PhysicsEngine.h"
#include "./physics/RigidBody.h"

class App {
public:

	App(unsigned int SCREEN_WIDTH = 800, unsigned int SCREEN_HEIGHT = 600);
	~App();

	bool Init();
	vooid Run();

private:

	// window & context
	GLFWwindow* window = nullptr;
	const char* glsl_version = "#version 330 core";
	unsigned int SCREEN_WIDTH, SCREEN_HEIGHT;

	// timing
	float deltaTime = 0.0f;
	float lastFrame = 0.0f;

	// camera & input
	Camera camera;
	bool firstMouse = true;
	float lastX, lastY;

	// mouse picking/dragging
	struct MouseState { bool isDown = false; float x, y; } leftMouseButton, rightMouseButton;
	bool isDragging = false;
	int selected = -1;


	// rendering & scene
	PhysicsEngine scene;
	UseImGui gui;
	bool wireFrame = false;

	Shader* basicShader = nullptr;
	Shader* pickingShader = nullptr;
	Shader* outlineShader = nullptr;


	// initialization
	void InitGLFW();
	void InitGLAD();
	void InitCallbacks();
	void InitScene();

	// main loop
	void ProcessInput(GLFWwindow* window);
	void Update();
	void Render();

	// callback handlers
	void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
	void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
	void KeyToggleCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
	void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);

};