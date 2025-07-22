// osmium-test.cpp : Defines the entry point for the application.
//
#define _HAS_STD_BYTE 0
#define NOMINMAX

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

// core
#include "./inc/model.h"
#include "./inc/mesh.h"
#include "./inc/camera.h"
#include "./inc/shader.h"
#include "./inc/ray.h"

// debugging
#include "./inc/use-imgui.h"

// mouse picking
#include "./inc/MousePicking.hpp"

// physics
#include "./physics/PhysicsEngine.h"
#include "./physics/RigidBody.h"



void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void scroll_callback(GLFWwindow* window, double xoffset, double yoffset);
void toggle_callback(GLFWwindow* window, int key, int scancode, int action, int mods);
void mouse_press_callback(GLFWwindow* window, int button, int action, int mods);
void processInput(GLFWwindow* window);


// GLOBAL VARIABLES
// ---------------------------------------------------------------------------------------------

// window dims.
const unsigned int SCR_WIDTH = 800;
const unsigned int SCR_HEIGHT = 600;

// camera
Camera camera(glm::vec3(0.0f, 0.0f, 3.0f));
float lastX = SCR_WIDTH / 2.0f;
float lastY = SCR_HEIGHT / 2.0f;
bool firstMouse = true;

// mouse picking/dragging
struct MouseState {
    bool isPressed = false;
    float x = 0.0f;
    float y = 0.0f;
} leftMouseButton, rightMouseButton;

bool isDragging = false;



// timing
float deltaTime = 0.0f;
float lastFrame = 0.0f;

// toggle modes
bool wireFrame = false;
bool enableMouse = false;

PhysicsEngine Scene;

// ---------------------------------------------------------------------------------------------

int main()
{
    // InitGLFW()
    // initialize and configure glfw
    // ------------------------------
    glfwInit();
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == NULL)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }


    // InitCallbacks()
    // glfw initialize callbacks
    // -------------------------
    glfwMakeContextCurrent(window);
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwSetCursorPosCallback(window, mouse_move_callback);
    glfwSetScrollCallback(window, scroll_callback);
    glfwSetKeyCallback(window, toggle_callback);
    glfwSetMouseButtonCallback(window, mouse_press_callback);


    // tell GLFW to capture our mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // glad: load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return -1;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);      // Final color = FragColor.rgb * FragColor.a + FrameBuff.rgb * (1 - FragColor.a)
    // FragColor.a close to 1 --> FragColor
    // FragColor.a close to 0 --> FrameBuff color


// initialize scene & models & shaders
// -----------------------------------
    Shader basicShader("osm.vert", "osm.frag");
    Shader pickingColor("pickingColor.vert", "pickingColor.frag");


    Model cubeModel("cube.obj");
    Model ballModel("ball.obj");
    Model cylinderModel("cilindru.obj");


    RigidBody cube(cubeModel, 5.0, glm::vec3(0.0, 0.005, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    RigidBody cyl(cylinderModel, 5.0, glm::vec3(10.0, 1.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));

    Scene.addRigidBody(cube);
    Scene.addRigidBody(cyl);

    UseImGui myImGui;
    myImGui.Init(window, glsl_version);



    // MAIN LOOP
    // ---------------------------------------------------------------------------------------------

    while (!glfwWindowShouldClose(window))
    {
        // per-frame time logic
        // --------------------
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;


        // poll IO events (keys pressed/released, mouse moved etc.)
        // --------------------------------------------------------
        glfwPollEvents();
        processInput(window); // manually checks state via glfwGetKey

        if (myImGui.darkMode) {
            glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        }
        else {
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
        }
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);



        // view/projection transformations
        glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
        glm::mat4 view = camera.GetViewMatrix();

        Scene.step(deltaTime);



        // raycast mouse coords into scene
        MousePicking picker;
        picker.ScreenToWorldRay(lastX, lastY, SCR_WIDTH, SCR_HEIGHT, projection, view);

        for (unsigned int i = 0; i < Scene.rigidBodies.size(); i++) {



            glm::mat4 world = glm::translate(glm::mat4(1.0), Scene.rigidBodies[i].getCentreOfMass());

            pickingColor.use();

            glm::mat4 model = glm::mat4(1.0);
            model = glm::translate(model, Scene.rigidBodies[i].collider.GetCentroid());
            model = glm::scale(Scene.rigidBodies[i].collider.GetScale());

            pickingColor.setMat4("gWVP", projection * view * model);



            // if ray intersects a body's bounding sphere
            // only check intersects on mouse clicks -- else performance tanks 6x in fps when 
            if (leftMouseButton.isPressed) {


                // if intersects, we should
                // 1. stop the body from moving
                // 2. be able to move the 3D
                if (Scene.rigidBodies[i].collided(picker.GetRay())) {

                    std::cout << "Ray origin: [" << picker.GetRay().origin.x << ", " << picker.GetRay().origin.y << ", " << picker.GetRay().origin.z << "]" << std::endl;
                    std::cout << "Ray direction: [" << picker.GetRay().direction.x << ", " << picker.GetRay().direction.y << ", " << picker.GetRay().direction.z << "]" << std::endl;


                    picker.DragObject(camera, picker.GetRay(), Scene.rigidBodies[i]);
                    Scene.rigidBodies[i].getModel().Draw(pickingColor);
                }

            }


            Scene.rigidBodies[i].getBounds().Draw(pickingColor);


            basicShader.use();
            basicShader.setMat4("gWVP", projection * view * world);

            Scene.rigidBodies[i].getModel().Draw(basicShader);
        }

        myImGui.NewFrame();
        myImGui.Update();
        myImGui.Render();



        // swap buffers
        glfwSwapBuffers(window);
    }

    // terminate, clearing all previously allocated GLFW resources.
    myImGui.Shutdown();
    glfwTerminate();
    return 0;
}
// ---------------------------------------------------------------------------------------------




// INPUTS
// ---------------------------------------------------------------------------------------------

// process all input: query GLFW whether relevant keys are pressed/released this frame and react accordingly
void processInput(GLFWwindow* window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }


    // rightMouseButton enables WASD movement
    if (rightMouseButton.isPressed) {

        if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
            camera.ProcessKeyboard(FORWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
            camera.ProcessKeyboard(BACKWARD, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
            camera.ProcessKeyboard(LEFT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
            camera.ProcessKeyboard(RIGHT, deltaTime);
        if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
            camera.ProcessKeyboard(UP, deltaTime);

    }
}
// ---------------------------------------------------------------------------------------------




// CALLBACK FUNCTIONS
// ---------------------------------------------------------------------------------------------

void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    // make sure the viewport matches the new window dimensions; note that width and 
    // height will be significantly larger than specified on retina displays.
    glViewport(0, 0, width, height);
}

void mouse_press_callback(GLFWwindow* window, int button, int action, int mods)
{
    // retrieve cursor coords.
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    float xpos = static_cast<float>(x);
    float ypos = static_cast<float>(y);


    // leftMouseButton controls picking/dragging
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {

        leftMouseButton.isPressed = true;
        leftMouseButton.x = xpos;
        leftMouseButton.y = ypos;

        MousePicking picker;
        picker.ScreenToWorldRay(lastX, lastY, SCR_WIDTH, SCR_HEIGHT, projection, view);

        if (isDragging) {

            // drag object
        }
        else {

            // no object selected
            // check for collisions

            for (auto& body : Scene.rigidBodies) {
                if (Scene.rigidBodies[i].collided(picker.GetRay())) {

                }
            }

            isDragging = true;
        }

    }
    // rightMouseButton controls camera/wasd movement
    else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {

        rightMouseButton.isPressed = true;

        // process mouse input for camera movement
        if (firstMouse)
        {
            lastX = xpos;
            lastY = ypos;
            firstMouse = false;
        }

        float xoffset = xpos - lastX;
        float yoffset = lastY - ypos; // reversed since y-coordinates go from bottom to top

        lastX = xpos;
        lastY = ypos;

        camera.ProcessMouseMovement(xoffset, yoffset);
    }

    else {
        leftMouseButton.isPressed = false;
        rightMouseButton.isPressed = false;
    }
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}

void toggle_callback(GLFWwindow* window, int key, int scancode, int action, int mods) {

    // Toggle wireframe mode when key 1 is pressed.
    if (key == GLFW_KEY_1 && action == GLFW_PRESS) {
        wireFrame = !wireFrame;
        if (wireFrame)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    // Toggle mouse mode when key 2 is pressed.
    if (key == GLFW_KEY_2 && action == GLFW_PRESS) {
        enableMouse = !enableMouse;
        if (enableMouse)
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_CAPTURED);
        else
            glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    }

}
