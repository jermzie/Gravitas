#ifndef GRAVITAS_HPP
#define GRAVITAS_HPP

#ifndef PROJECT_SOURCE_DIR
#error "PROJECT_SOURCE_DIR is not defined"
#endif

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <iostream>
#include <vector>

#include "../inc/Model.hpp"
#include "../inc/Mesh.hpp"
#include "../inc/Camera.hpp"
#include "../inc/Shader.hpp"
#include "../inc/ImGUI.hpp"
#include "../inc/MousePicking.hpp"
#include "../inc/Plane.hpp"
#include "../physics/PhysicsEngine.hpp"
#include "../physics/RigidBody.hpp"

class App {
public:

	App(unsigned int SCREEN_WIDTH = 800, unsigned int SCREEN_HEIGHT = 800);
	~App();

	bool Init();
	void Run();



private:

	// window & context
	GLFWwindow* window = nullptr;
	const char* glsl_version = "#version 330 core";
	unsigned int SCREEN_WIDTH, SCREEN_HEIGHT;

	// timing
	float deltaTime = 0.0f;
	float currentFrame= 0.0f;
	float lastFrame = 0.0f;

	// camera & input
	Camera camera{ glm::vec3(0.0f, 0.0f, 3.0f) };    // initial camera pos.
	bool firstMouse = true;
	float lastX, lastY;

	// mouse picking/dragging
	MousePicking picker;
	Plane dragPlane;
	float dragSensitivity = 1.0f;

	glm::vec4 objectViewSpacePos;

	struct MouseState { bool isDown = false; bool firstMouse = true; float x, y, xOffset, yOffset; } leftMouseButton, rightMouseButton;

	glm::vec3 initialIntersection;
	glm::vec3 initialObjectPos;
	glm::vec3 oldPos;
	glm::vec3 newPos;
	glm::vec3 initialHitPoint;
	bool isDragging = false;
	int selectedObjectId = -1;



	// rendering & scene
	PhysicsEngine scene;
	ImGUI gui;
	bool wireFrame = false;
	bool defaultView = false;

	Shader basicShader;
	Shader pickingShader;
	Shader outlineShader;

	// initialization
	bool InitGLFW();
	bool InitGLAD();
	void InitCallbacks();
	void InitScene();

	// main loop
	void ProcessInput();
	void Update();
	void Render();


	//void DragObject();

	// callback handlers
	void OnFramebufferSize(int width, int height);
	void OnScroll(double xoffset, double yoffset);
	void OnKey(int key, int scancode, int action, int mods);
	void OnMouseMove(double xpos, double ypos);
	void OnMouseButton(int button, int action, int mods);

	// static callback wrappers
	static void FramebufferSizeCallback(GLFWwindow* window, int width, int height);
	static void ScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
	static void KeyToggleCallback(GLFWwindow* window, int key, int scancode, int action, int mods);
	static void MouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
	static void MouseMoveCallback(GLFWwindow* window, double xpos, double ypos);
};

#endif
