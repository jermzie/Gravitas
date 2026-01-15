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
#include <algorithm>
#include <vector>

// Rendering
#include "../inc/Model.hpp"
#include "../inc/Mesh.hpp"
#include "../inc/Camera.hpp"
#include "../inc/Shader.hpp"

// Debugging
#include "../inc/Gui.hpp"

// Physics
#include "../physics/PhysicsEngine.hpp"
#include "../physics/RigidBody.hpp"

#include "../inc/Plane.hpp"
#include "../inc/Ray.hpp"

class Gravitas {
public:

	Gravitas(unsigned int SCREEN_WIDTH = 800, unsigned int SCREEN_HEIGHT = 800);
	~Gravitas();

	bool Init();
	void Run();



private:

	// window & context
	GLFWwindow* window = nullptr;
	const char* glsl_version = "#version 330 core";
	unsigned int SCREEN_WIDTH, SCREEN_HEIGHT;

	// timing
	float fixedTimeStep = 1.0f / 60.0f;		// 60 steps per second
	float deltaTime = 0.0f;
	float currentFrame= 0.0f;
	float lastFrame = 0.0f;

	// camera & input
	Camera camera{ glm::vec3(0.0f, 0.0f, 20.0f) };			// initial camera pos.
	bool firstMouse = true;									// initial mouse jittering
	float lastX, lastY;										// previous mouse pos. 					

	// mouse picking/dragging
	struct MouseState { bool isDown = false; bool firstMouse = true; float x, y, x_offset, y_offset; } leftMouseButton, rightMouseButton;
	Plane dragPlane;
	bool isDragging = false;
	bool isRotating = false;
	int selectedObjectId = -1;
	glm::vec3 oldPos;
	glm::vec3 newPos;

	bool dragXZ = false;
	bool dragYZ = false;
	bool dragXY = false;


	// Physics
	PhysicsEngine scene;
	Gui gui;

	bool wireFrame = false;
	bool defaultView = false;
	
	// shaders
	Shader basicShader;
	Shader pickingShader;
	Shader outlineShader;
	Shader lightingShader;
	Shader defaultShader;

	// transformations
	glm::mat4 projection;
	glm::mat4 view;

	// initialization
	bool InitGLFW();
	bool InitGLAD();
	void InitCallbacks();
	void InitScene();

	// main loop
	void ProcessInput();
	void Update();
	void Render();


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
