#include "Gravitas.hpp"

App::App(unsigned int width, unsigned int height) : SCREEN_WIDTH(width), SCREEN_HEIGHT(height) {

    lastX = width / 2.0f;
    lastY = height / 2.0f;
}

App::~App() {
    gui.Shutdown();
    glfwTerminate();
}

bool App::Init() {

    if (!InitGLFW())
        return false;

    if (!InitGLAD())
        return false;

    InitCallbacks();
    InitScene();
    gui.Init(window, glsl_version);

    return window != nullptr;
}

void App::Run() {
    while (!glfwWindowShouldClose(window)) {

        // per-frame time logic
        currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        // poll i/o events
        glfwPollEvents();
        ProcessInput();

        // update model physics & positions
        Update();

        // render scene 
        Render();


        // imgui shit 
        gui.NewFrame();
        gui.Update();
        gui.Render();

        glfwSwapBuffers(window);

        // VSYNC disabled; limits fps to refresh rate (144hz)
        glfwSwapInterval(1);
    }
}

bool App::InitGLFW() {

    // initialize and configure glfw
    // ------------------------------
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    // glfw window creation
    // --------------------
    window = glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "LearnOpenGL", NULL, NULL);
    if (window == nullptr)
    {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(window);                                     // make opengl context current immediately after window creation
    glfwSetWindowUserPointer(window, this);                             // Add this line
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_CAPTURED);        // tell glfw to capture our mouse


    return true;
}

bool App::InitGLAD() {

    // load all OpenGL function pointers
    // ---------------------------------------
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cout << "Failed to initialize GLAD" << std::endl;
        return false;
    }

    // configure global opengl state
    // -----------------------------
    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LESS);
    glEnable(GL_STENCIL_TEST);
    glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);  // pass/fail action for stencil test

    // glEnable(GL_BLEND);
    // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);      // Final color = FragColor.rgb * FragColor.a + FrameBuff.rgb * (1 - FragColor.a)
    // FragColor.a close to 1 --> FragColor
    // FragColor.a close to 0 --> FrameBuff color

    return true;
}

void App::InitCallbacks() {

    // glfw initialize callbacks
    // -------------------------
    glfwSetFramebufferSizeCallback(window, &App::FramebufferSizeCallback);
    glfwSetScrollCallback(window, &App::ScrollCallback);
    glfwSetKeyCallback(window, &App::KeyToggleCallback);
    glfwSetMouseButtonCallback(window, &App::MouseButtonCallback);
    glfwSetCursorPosCallback(window, &App::MouseMoveCallback);

}

void App::InitScene() {

    defaultShader.init("lightObject.vert", "lightObject.frag");
    lightingShader.init("lightSource.vert", "lightSource.frag");
    pickingShader.init("pickingColor.vert", "pickingColor.frag");
    outlineShader.init("stencilOutline.vert", "stencilOutline.frag");

    Model tetraModel("tetrahedron.obj");
    Model cubeModel("cube.obj");
    Model ballModel("ball.obj");
    //Model suzanneModel("suzanne.obj");
    Model cylinderModel("cilindru.obj");
    Model suzanneModel("suzanne.obj");
    Model teapotModel("teapot.obj");
    Model bunnyModel("stanford-bunny.obj");

    RigidBody light(cubeModel, 1.0, glm::vec3(1.0, 2.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    //RigidBody tetra(tetraModel, 5.0, glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    //RigidBody cube(cubeModel, 5.0, glm::vec3(0.0, 5.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(1.0, 0.0, 0.0));
    RigidBody cyl(cylinderModel, 5.0, glm::vec3(0.0, 8.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(10.0, 0.0, 0.0));
    //RigidBody ball(ballModel, 5.0, glm::vec3(0.0, 0.005, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    //RigidBody suzanne(suzanneModel, 1.0, glm::vec3(0.0, 2.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(100.0, 100.0, 100.0));
    //RigidBody suzanne2(suzanneModel, 5.0, glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    //RigidBody suzanne3(suzanneModel, 5.0, glm::vec3(0.0, 5.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    RigidBody teapot(teapotModel, 5.0, glm::vec3(0.0, 1.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(10.0, 0.0, 0.0));
    RigidBody bunny(bunnyModel, 5.0, glm::vec3(0.0, 0.005, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
       
    //suzanne.printDebug();

    scene.addRigidBody(std::move(light));
    //scene.addRigidBody(std::move(tetra));
    //scene.addRigidBody(std::move(cube));
    scene.addRigidBody(std::move(cyl));
    //scene.addRigidBody(std::move(ball));
    //scene.addRigidBody(std::move(suzanne));
    //scene.addRigidBody(std::move(suzanne2));
    //scene.addRigidBody(std::move(suzanne3));
    scene.addRigidBody(std::move(teapot));
    scene.addRigidBody(std::move(bunny));

   
   
    
}

void App::ProcessInput()
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, true);
    }

    // rightMouseButton enables WASD movement
    if (rightMouseButton.isDown) {

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
    
    // Use QWE keys to select cartesian drag plane (tmp soln)
    // maybe toggle???
    if (glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS){                // XZ-plane
        dragPlane.normal = glm::vec3(0.0f,1.0f,0.0f);
        //dragPlane.point = initialObjectPos;
    }
    else if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS) {         // YZ-plane
        dragPlane.normal = glm::vec3(1.0f, 0.0f, 0.0f);
        //dragPlane.point = initialObjectPos;
    }
    else if (glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS) {        // XY-plane
        dragPlane.normal = glm::vec3(0.0f, 0.0f, 1.0f);
        //dragPlane.point = initialObjectPos;
    }
    /*
    else if (glfwGetKey(window, GLFW_KEY_R) == GLFW_PRESS) {        // Rotate picked object (no dragging)
        isRotating = true;
    }
    */

    
}


// UPDATE
// ...............................................................................................................................
void App::Update() {

    // view & projection transformations
    glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCREEN_WIDTH / (float)SCREEN_HEIGHT, 0.01f, 500.0f);
    glm::mat4 view = camera.GetViewMatrix();


    // update picking & dragging
    if (leftMouseButton.isDown) {

        // convert 2D mouse coords to world space coords
        Ray ray = picker.ScreenToWorldRay(leftMouseButton.x, leftMouseButton.y, SCREEN_WIDTH, SCREEN_HEIGHT, projection, view);

        if (isDragging && selectedObjectId != -1) {

            // find ray-plane intersection

            float t;
            if (picker.RayPlaneIntersection(ray, dragPlane.normal, dragPlane.point, t)) {

                // new point of intersection
                newPos = ray.origin + ray.direction * t;
                glm::vec3 displacement = newPos - oldPos;
            
                // update positions + velocities???
                scene.rigidBodies[selectedObjectId].drag(displacement);

                oldPos = newPos;


            }
        }

        else if (isRotating && selectedObjectId != -1) {

        }

        else {

            // check ray-hull intersections for all bodies
            for (int i = 0; i < scene.rigidBodies.size(); ++i) {

                glm::vec3 hitPoint;
                if (scene.rigidBodies[i].collided(ray, hitPoint)) {

                    selectedObjectId = i;

                    if (isRotating) {

                        // rotate model w/ mouse
                        // 1. initial model selection w/ mouse picking
                        // 2. as long as 'R' key pressed, no picking/dragging/ray-model intersections, rotate same selected model
                        // 3. 
                        // how to choose rotation axis??

                    }
                    else {

                        isDragging = true;
                        scene.rigidBodies[i].disable();

                        // record initial object pos
                        initialObjectPos = scene.rigidBodies[i].getCentreOfMass();
                        oldPos = hitPoint;

                        // define drag plane
                        dragPlane.normal = -camera.Front;
                        dragPlane.point = hitPoint;
                    }

                    break;
                }
            }
        }
    }
 
 

    // use fixed timesteps for consistency
    float physicsDT = std::clamp(deltaTime, 1.0f / 300.0f, 1.0f / 60.0f);

    // update physics
    scene.step(physicsDT);


}



// RENDER
// ...............................................................................................................................
void App::Render() {


    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);



    // view & projection transformations
    glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCREEN_WIDTH / (float)SCREEN_HEIGHT, 0.01f, 500.0f);
    glm::mat4 view = camera.GetViewMatrix();


    for (int i = 0; i < scene.rigidBodies.size(); ++i) {


        WorldTransform& objectTrans = scene.rigidBodies[i].getWorldTransform();
        //glm::vec3 position = objectTrans.GetPosition();
 


        if (i == selectedObjectId) {

            
            // first pass -- render object normally & write to stencil buffer
            glStencilFunc(GL_ALWAYS, 1, 0xFF);
            glStencilMask(0xFF);
            glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

            pickingShader.use();
            pickingShader.setMat4("gWVP", projection * view * objectTrans.GetMatrix());
            scene.rigidBodies[i].draw(pickingShader);


            // draw outline
            // second pass -- render scaled down object & disable stencil writing
            glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
            glStencilMask(0x00);
            glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
            glDisable(GL_DEPTH_TEST);

            outlineShader.use();
            glm::mat4 orginalMat4 = objectTrans.GetMatrix();
            glm::mat4 outlineMat4 = glm::scale(orginalMat4, glm::vec3(1.05f));

            outlineShader.setMat4("gWVP", projection * view * outlineMat4);
            scene.rigidBodies[i].draw(outlineShader);

            glStencilFunc(GL_ALWAYS, 0, 0xFF);
            glStencilMask(0xFF);
            glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
            glEnable(GL_DEPTH_TEST);


        }
        else {

            glStencilFunc(GL_ALWAYS, 0, 0xFF);
            glStencilMask(0xFF);

            // light source
            if (i == 0) {
                lightingShader.use();
                lightingShader.setMat4("gWVP", projection * view * objectTrans.GetMatrix());
                scene.rigidBodies[i].draw(lightingShader);

            }

            else {

                defaultShader.use();

                // vert
               // defaultShader.setMat4("gWVP", projection * view * objectTrans.GetMatrix());
                defaultShader.setMat4("projection", projection);
                defaultShader.setMat4("view", view);
                defaultShader.setMat4("model", objectTrans.GetMatrix());

                // frag
                defaultShader.setVec3("objectColor", 1.0f, 0.5f, 0.31f);
                defaultShader.setVec3("lightColor", 1.0f, 1.0f, 0.773f);
                defaultShader.setVec3("lightPos", scene.rigidBodies[0].getCentreOfMass());
                defaultShader.setVec3("viewPos", camera.Position);

                scene.rigidBodies[i].draw(defaultShader);

            }
        }
        
        // probably make a imgui toggle for this
        // draw wireframe collision mesh
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        pickingShader.use();
        WorldTransform& colliderTrans = scene.rigidBodies[i].hull.getWorldTransform();
        //glm::vec3 colliderPosition = colliderTrans.GetPosition();
        //colliderTrans.SetPosition(colliderPosition * PHYSICS_SCALE);
        pickingShader.setMat4("gWVP", projection * view * colliderTrans.GetMatrix());
        scene.rigidBodies[i].hull.draw(pickingShader);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        

        // reset to default state after rendering
        glStencilMask(0xFF);
        glStencilFunc(GL_ALWAYS, 0, 0xFF);
        glEnable(GL_DEPTH_TEST);

    }
}

void App::OnFramebufferSize(int width, int height) { 

	SCREEN_WIDTH = width;
	SCREEN_HEIGHT = height;
	glViewport(0, 0, width, height);
	
	// BAD Solution for dynamic window resizing
	// Render();
}

void App::OnScroll(double xOffset, double yOffset) { camera.ProcessMouseScroll(static_cast<float>(yOffset)); }
void App::OnKey(int key, int scancode, int action, int mods) {

    // Toggle wireframe mode when key 1 is pressed.
    if (key == GLFW_KEY_1 && action == GLFW_PRESS) {
        wireFrame = !wireFrame;
        if (wireFrame)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    /*
    if (key == GLFW_KEY_2 && action == GLFW_PRESS) {
        defaultView = !defaultView;
        if (defaultView) {

            // defaultView derived from trial & error
            camera.Position = glm::vec3(15.0f, 19.0f, 16.0f);
            camera.Front = glm::vec3(-0.4f, -0.7f, -0.6f);
        }
        else {
            camera.Position = glm::vec3(0.0f, 0.0f, 3.0f);
            camera.Front = glm::vec3(0.0f, 0.0f, -1.0f);
        }
    }
    */
}

void App::OnMouseMove(double xpos, double ypos) {

    float xPos = static_cast<float>(xpos);
    float yPos = static_cast<float>(ypos);

    if (rightMouseButton.isDown) {

        // process mouse input
        if (firstMouse)
        {
            lastX = xPos;
            lastY = yPos;
            firstMouse = false;
        }

        float xOffset = xPos - lastX;
        float yOffset = lastY - yPos; // flip since y-axis goes from bottom to top

        lastX = xPos;
        lastY = yPos;


        camera.ProcessMouseMovement(xOffset, yOffset);
    }

    if (leftMouseButton.isDown) {


        leftMouseButton.xOffset = xPos - lastX;
        leftMouseButton.yOffset = lastY - yPos;


        lastX = xPos;
        lastY = yPos;

        leftMouseButton.x = xPos;
        leftMouseButton.y = yPos;


        //std::cout << "MOUSE COORDS: " << leftMouseButton.x << " " << leftMouseButton.y << "\n";
    }

}
void App::OnMouseButton(int button, int action, int mods) {

    // retrieve cursor coords.
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    float xPos = static_cast<float>(x);
    float yPos = static_cast<float>(y);

    // leftMouseButton controls picking/dragging
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {

        // capture cursor for picking
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_CAPTURED);

        leftMouseButton.isDown = true;
        leftMouseButton.x = xPos;
        leftMouseButton.y = yPos;


    }
    // rightMouseButton controls camera/wasd movement
    else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {

        // disable cursor for camera movement 
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        rightMouseButton.isDown = true;
        firstMouse = true;
        rightMouseButton.x = xPos;
        rightMouseButton.y = yPos;

    }
    // handle button release
    else if (action == GLFW_RELEASE) {

        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            leftMouseButton.isDown = false;
            
            if (selectedObjectId != -1) {
                scene.rigidBodies[selectedObjectId].isDragging = false;
            }

            isDragging = false; // stop dragging on release
            //scene.rigidBodies[selectedObjectId].isDragging = false;
            leftMouseButton.firstMouse = true;
            selectedObjectId = -1;  // reset selected object index on release

            if (!rightMouseButton.isDown) {

                // normal cursor if no buttons are pressed
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

            }
        }

        else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
            rightMouseButton.isDown = false;

            if (!leftMouseButton.isDown) {

                // normal cursor if no buttons are pressed
                glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
            }
        }
    }
}


void App::FramebufferSizeCallback(GLFWwindow* w, int a, int b) {
    static_cast<App*>(glfwGetWindowUserPointer(w))->OnFramebufferSize(a, b);
}
void App::ScrollCallback(GLFWwindow* w, double x, double y) {
    static_cast<App*>(glfwGetWindowUserPointer(w))->OnScroll(x, y);
}
void App::KeyToggleCallback(GLFWwindow* w, int k, int s, int a, int m) {
    static_cast<App*>(glfwGetWindowUserPointer(w))->OnKey(k, s, a, m);
}
void App::MouseButtonCallback(GLFWwindow* w, int b, int a, int m) {
    static_cast<App*>(glfwGetWindowUserPointer(w))->OnMouseButton(b, a, m);
}
void App::MouseMoveCallback(GLFWwindow* w, double x, double y) {
    static_cast<App*>(glfwGetWindowUserPointer(w))->OnMouseMove(x, y);
}


int main() {
    App physicsTest;
    if (!physicsTest.Init()) return -1;
    physicsTest.Run();
    return 0;
}
