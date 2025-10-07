#include "Gravitas.hpp"

Gravitas::Gravitas(unsigned int width, unsigned int height) : SCREEN_WIDTH(width), SCREEN_HEIGHT(height) {

    lastX = width / 2.0f;
    lastY = height / 2.0f;
}

Gravitas::~Gravitas() {
    gui.Shutdown();
    glfwTerminate();
}

bool Gravitas::Init() {

    if (!InitGLFW())
        return false;

    if (!InitGLAD())
        return false;

    InitCallbacks();
    InitScene();
    gui.Init(window, glsl_version);

    return window != nullptr;
}

void Gravitas::Run() {
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

bool Gravitas::InitGLFW() {

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

bool Gravitas::InitGLAD() {

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

void Gravitas::InitCallbacks() {

    // glfw initialize callbacks
    // -------------------------
    glfwSetFramebufferSizeCallback(window, &Gravitas::FramebufferSizeCallback);
    glfwSetScrollCallback(window, &Gravitas::ScrollCallback);
    glfwSetKeyCallback(window, &Gravitas::KeyToggleCallback);
    glfwSetMouseButtonCallback(window, &Gravitas::MouseButtonCallback);
    glfwSetCursorPosCallback(window, &Gravitas::MouseMoveCallback);

}

void Gravitas::InitScene() {

    defaultShader.init("lightObject.vert", "lightObject.frag");
    lightingShader.init("lightSource.vert", "lightSource.frag");
    pickingShader.init("pickingColor.vert", "pickingColor.frag");
    outlineShader.init("stencilOutline.vert", "stencilOutline.frag");

    Model tetraModel("tetrahedron.obj");
    Model cubeModel("cube.obj");
    Model ballModel("ball.obj");
    Model cylinderModel("cilindru.obj");
    Model suzanneModel("suzanne.obj");
    Model teapotModel("teapot.obj");
    Model davidModel("david.obj");

    RigidBody light(cubeModel, 1.0, glm::vec3(1.0, 1.5, 4.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    //RigidBody tetra(tetraModel, 5.0, glm::vec3(-0.5, 1.0, 4.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    RigidBody tetra(tetraModel, 5.0, glm::vec3(1.5, 1.0, 4.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    RigidBody cube(cubeModel, 5.0, glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    RigidBody cyl(cylinderModel, 5.0, glm::vec3(2.0, 2.0, 2.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    RigidBody ball(ballModel, 5.0, glm::vec3(3.0, 3.0, 3.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.5, 0.5, 0.5));
    RigidBody suzanne(suzanneModel, 1.0, glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.5, 0.5, 0.5));
    //RigidBody suzanne2(suzanneModel, 5.0, glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    //RigidBody suzanne3(suzanneModel, 5.0, glm::vec3(0.0, 5.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));
    RigidBody teapot(teapotModel, 5.0, glm::vec3(0.0, 1.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.5, 0.5, 0.5));
    //RigidBody david(davidModel, 5.0, glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0), glm::vec3(0.0, 0.0, 0.0));


    scene.addRigidBody(std::move(light));
    //scene.addRigidBody(std::move(tetra));
    scene.addRigidBody(std::move(cube));
    //scene.addRigidBody(std::move(cyl));
    //scene.addRigidBody(std::move(ball));
    //scene.addRigidBody(std::move(suzanne));
    //scene.addRigidBody(std::move(teapot));
    //scene.addRigidBody(std::move(david));

   
   
    
}

void Gravitas::ProcessInput()
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
}


// UPDATE
// ...............................................................................................................................
void Gravitas::Update() {

    // view & projection transformations
    glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCREEN_WIDTH / (float)SCREEN_HEIGHT, 0.01f, 500.0f);
    glm::mat4 view = camera.GetViewMatrix();

    if (leftMouseButton.isDown) {

        // convert 2D mouse coords to world space coords
        Ray ray = picker.ScreenToWorldRay(leftMouseButton.x, leftMouseButton.y, SCREEN_WIDTH, SCREEN_HEIGHT, projection, view);


        // OBJECT SELECTED AND CURRENTLY ROTATING
        if (isRotating && selectedObjectId != -1) {

            scene.bodies[selectedObjectId].rotate(leftMouseButton.xOffset, leftMouseButton.yOffset);


            // per-frame offsets -- rotations don't continue when mouse isn't moving
            leftMouseButton.xOffset = 0.0f;
            leftMouseButton.yOffset = 0.0f;

        }

        // CURRENTLY DRAGGING
        else if (isDragging && selectedObjectId != -1) {

            // find ray-plane intersection
            float t;
            if (picker.RayPlaneIntersection(ray, dragPlane.normal, dragPlane.point, t)) {

                // new point of intersection
                newPos = ray.origin + ray.direction * t;
                glm::vec3 displacement = newPos - oldPos;
            
                // update positions + velocities???
                scene.bodies[selectedObjectId].drag(displacement);

                oldPos = newPos;
            }
        }

        // NO DRAGGING OR ROTATING
        // INITIAL OBJECT SELECTION
        else {

            // check ray-hull intersections for all bodies
            for (int i = 0; i < scene.bodies.size(); ++i) {

                glm::vec3 hitPoint;
                if (scene.bodies[i].collided(ray, hitPoint)) {

                    selectedObjectId = i;
                    scene.bodies[i].disable();

                    // record initial object pos
                    oldPos = hitPoint;

                    // define drag plane
                    dragPlane.normal = -camera.Front;
                    dragPlane.point = hitPoint;

                    break;
                }
            }

            // cannot translate & rotate object at same time
            isDragging = !isRotating;
        }
    }
 


    // use fixed timesteps for consistency
    //float physicsDT = std::clamp(deltaTime, 1.0f / 300.0f, 1.0f / 60.0f);

    // update physics
    scene.step(fixedTimeStep);


}



// RENDER
// ...............................................................................................................................
void Gravitas::Render() {


    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);



    // view & projection transformations
    glm::mat4 projection = glm::perspective(glm::radians(camera.Zoom), (float)SCREEN_WIDTH / (float)SCREEN_HEIGHT, 0.01f, 500.0f);
    glm::mat4 view = camera.GetViewMatrix();


    for (int i = 0; i < scene.bodies.size(); ++i) {

        
        WorldTransform& bodyTrans = scene.bodies[i].getWorldTransform();
        //glm::vec3 position = objectTrans.GetPosition();
 
        /*
        glm::vec3 com = scene.bodies[i].getCentreOfMass();
        glm::vec3 cen = scene.bodies[i].hull.getCentroid();

        std::cout << "(" << com.x << ", " << com.y << ", " << com.z << ")         ";
        std::cout << "(" << cen.x << ", " << cen.y << ", " << cen.z << ")\n";
        */

        if (i == selectedObjectId) {
            
            // first pass -- render object normally & write to stencil buffer
            glStencilFunc(GL_ALWAYS, 1, 0xFF);
            glStencilMask(0xFF);
            glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

           /* pickingShader.use();
            pickingShader.setMat4("gWVP", projection * view * bodyTrans.GetMatrix());
            scene.bodies[i].draw(pickingShader);*/
            
            defaultShader.use();

            // vert uniforms
           // defaultShader.setMat4("gWVP", projection * view * bodyTrans.GetMatrix());
            defaultShader.setMat4("projection", projection);
            defaultShader.setMat4("view", view);
            defaultShader.setMat4("model", bodyTrans.GetMatrix());

            // frag uniforms
            defaultShader.setVec3("objectColor", 0.0f, 1.0f, 0.0f);
            defaultShader.setVec3("lightColor", 1.0f, 1.0f, 0.773f);
            //defaultShader.setVec3("lightColor", 0.0f, 1.0f, 0.0f);
            defaultShader.setVec3("lightPos", scene.bodies[0].getCentreOfMass());
            defaultShader.setVec3("viewPos", camera.Position);

            scene.bodies[i].draw(defaultShader);
            

            // draw outline
            // second pass -- render scaled down object & disable stencil writing
            glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
            glStencilMask(0x00);
            glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
            glDisable(GL_DEPTH_TEST);

            /*
            outlineShader.use();
            glm::vec3 point = scene.bodies[i].getCentreOfMass();
            glm::mat4 originalMat4 = bodyTrans.GetMatrix();
            glm::mat4 t1 = glm::translate(glm::mat4(1.0f), point);
            glm::mat4 s = glm::scale(glm::mat4(1.0f), glm::vec3(1.1f));
            glm::mat4 t0 = glm::translate(glm::mat4(1.0f), -point);
            glm::mat4 outlineMat4 = t1 * s * t0 * originalMat4;
            
            outlineShader.setMat4("projection", projection);
            outlineShader.setMat4("view", view);
            outlineShader.setMat4("model", outlineMat4);

            //outlineShader.setMat4("gWVP", projection * view * originalMat4);
            scene.bodies[i].draw(outlineShader);
            */

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
                lightingShader.setMat4("gWVP", projection * view * bodyTrans.GetMatrix());
                scene.bodies[i].draw(lightingShader);

            }
            else {

                defaultShader.use();

                // vert uniforms
               // defaultShader.setMat4("gWVP", projection * view * bodyTrans.GetMatrix());
                defaultShader.setMat4("projection", projection);
                defaultShader.setMat4("view", view);
                defaultShader.setMat4("model", bodyTrans.GetMatrix());

                // frag uniforms
                defaultShader.setVec3("objectColor", 1.0f, 0.5f, 0.31f);
                defaultShader.setVec3("lightColor", 1.0f, 1.0f, 0.773f);
                defaultShader.setVec3("lightPos", scene.bodies[0].getCentreOfMass());
                defaultShader.setVec3("viewPos", camera.Position);

                scene.bodies[i].draw(defaultShader);

            }
        }
        

        /*
        // probably make a imgui toggle for this
        // draw wireframe collision mesh
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        pickingShader.use();
        WorldTransform& colliderTrans = scene.bodies[i].hull.getWorldTransform();
        //glm::vec3 colliderPosition = colliderTrans.GetPosition();
        //colliderTrans.SetPosition(colliderPosition * PHYSICS_SCALE);
        pickingShader.setMat4("gWVP", projection * view * colliderTrans.GetMatrix());
        scene.bodies[i].hull.draw(pickingShader);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        */

        // reset to default state after rendering
        glStencilMask(0xFF);
        glStencilFunc(GL_ALWAYS, 0, 0xFF);
        glEnable(GL_DEPTH_TEST);


    }
}

void Gravitas::OnFramebufferSize(int width, int height) {

	SCREEN_WIDTH = width;
	SCREEN_HEIGHT = height;
	glViewport(0, 0, width, height);
	
	// BAD Solution for dynamic window resizing
	// Render();
}
void Gravitas::OnScroll(double xOffset, double yOffset) { camera.ProcessMouseScroll(static_cast<float>(yOffset)); }
void Gravitas::OnKey(int key, int scancode, int action, int mods) {

    static int activeKey = -1; 

    // Only handle press & release
    if (action == GLFW_PRESS) {

        // nothing active, claim key
        if (activeKey == -1) {
            if (key == GLFW_KEY_Q || key == GLFW_KEY_W || key == GLFW_KEY_E || key == GLFW_KEY_R || key == GLFW_KEY_1 || key == GLFW_KEY_2 || key == GLFW_KEY_3)
                activeKey = key;
        }
        // ignore additional presses while another key is active
    }
    else if (action == GLFW_RELEASE) {
        // If the released key is the active key, clear it
        if (key == activeKey) activeKey = -1;
    }

    // Update active flag
    if (activeKey == GLFW_KEY_Q) {           // XZ-plane
        dragPlane.normal = glm::vec3(0.0f, 1.0f, 0.0f);
    }
    else if (activeKey == GLFW_KEY_W) {      // YZ-plane
        dragPlane.normal = glm::vec3(1.0f, 0.0f, 0.0f);
    }
    else if (activeKey == GLFW_KEY_E) {      // XY-plane
        dragPlane.normal = glm::vec3(0.0f, 0.0f, 1.0f);
    }
    else if (activeKey == GLFW_KEY_R) {      // Rotate
        isRotating = true;
    }
    else if (activeKey == GLFW_KEY_1) {
        camera.Position = glm::vec3(0.0f, 0.0f, 20.0f);
        camera.Front = glm::vec3(0.0f, 0.0f, -1.0f);
    }
    else if (activeKey == GLFW_KEY_2) {
        camera.Position = glm::vec3(20.0f, 0.0f, 0.0f);
        camera.Front = glm::vec3(-1.0f, 0.0f, 0.0f);
    }
    else if (activeKey == GLFW_KEY_3) {
        camera.Position = glm::vec3(0.0f, 20.0f, 0.0f);
        camera.Front = glm::vec3(0.0f, -1.0f, 0.0f);
    }
    else {                                   // Reset
        isRotating = false;
        isDragging = false;
        selectedObjectId = -1;
        dragPlane.normal = -camera.Front;
    }


    /*
    // Toggle wireframe mode when key 1 is pressed.
    if (key == GLFW_KEY_1 && action == GLFW_PRESS) {
        wireFrame = !wireFrame;
        if (wireFrame)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    
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

void Gravitas::OnMouseMove(double xpos, double ypos) {

    float xPos = static_cast<float>(xpos);
    float yPos = static_cast<float>(ypos);

    if (rightMouseButton.isDown) {

        // process initial mouse input
        if (firstMouse) {

            lastX = xPos;
            lastY = yPos;
            firstMouse = false;
        }

        float xOffset = xPos - lastX;
        float yOffset = lastY - yPos; // flip since y-axis goes from bottom to top

        camera.ProcessMouseMovement(xOffset, yOffset);
    }

    if (leftMouseButton.isDown) {

        leftMouseButton.x = xPos;
        leftMouseButton.y = yPos;
        leftMouseButton.xOffset = xPos - lastX;
        leftMouseButton.yOffset = lastY - yPos;
    }

    lastX = xPos;
    lastY = yPos;
}

void Gravitas::OnMouseButton(int button, int action, int mods) {

    // retrieve cursor coords.
    double x, y;
    glfwGetCursorPos(window, &x, &y);

    float xPos = static_cast<float>(x);
    float yPos = static_cast<float>(y);

    // leftMouseButton controls picking/dragging movement
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {

        // capture cursor for picking
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_CAPTURED);

        leftMouseButton.isDown = true;
        leftMouseButton.x = xPos;
        leftMouseButton.y = yPos;


    }

    // rightMouseButton controls camera/wasd movement
    else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {

        // disable cursor
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

        rightMouseButton.isDown = true;
        firstMouse = true;
        rightMouseButton.x = xPos;
        rightMouseButton.y = yPos;

    }

    // handle release
    else if (action == GLFW_RELEASE) {

        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            leftMouseButton.isDown = false;
            
            if (selectedObjectId != -1) {
                scene.bodies[selectedObjectId].isStatic = false;
            }

            // stop dragging/rotating on release
            isDragging = false;
            isRotating = false;

            // reset object selection
            leftMouseButton.firstMouse = true;
            selectedObjectId = -1; 

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


void Gravitas::FramebufferSizeCallback(GLFWwindow* w, int a, int b) {
    static_cast<Gravitas*>(glfwGetWindowUserPointer(w))->OnFramebufferSize(a, b);
}
void Gravitas::ScrollCallback(GLFWwindow* w, double x, double y) {
    static_cast<Gravitas*>(glfwGetWindowUserPointer(w))->OnScroll(x, y);
}
void Gravitas::KeyToggleCallback(GLFWwindow* w, int k, int s, int a, int m) {
    static_cast<Gravitas*>(glfwGetWindowUserPointer(w))->OnKey(k, s, a, m);
}
void Gravitas::MouseButtonCallback(GLFWwindow* w, int b, int a, int m) {
    static_cast<Gravitas*>(glfwGetWindowUserPointer(w))->OnMouseButton(b, a, m);
}
void Gravitas::MouseMoveCallback(GLFWwindow* w, double x, double y) {
    static_cast<Gravitas*>(glfwGetWindowUserPointer(w))->OnMouseMove(x, y);
}


int main() {
    Gravitas physicsTest;
    if (!physicsTest.Init()) return -1;
    physicsTest.Run();
    return 0;
}
