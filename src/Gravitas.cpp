#include "Gravitas.hpp"
#include "RigidBody.hpp"
#include <random>

Gravitas::Gravitas(unsigned int width, unsigned int height) : SCREEN_WIDTH(width), SCREEN_HEIGHT(height) {

  prev_x = width / 2.0f;
  prev_y = height / 2.0f;
}

Gravitas::~Gravitas() {
  gui.shutdown();
  glfwTerminate();
}

bool Gravitas::init() {

  if (!init_glfw())
    return false;

  if (!init_glad())
    return false;

  init_callbacks();
  debug.init();
  init_scene();
  gui.init(window, glsl_version);

  return window != nullptr;
}

void Gravitas::run() {
  while (!glfwWindowShouldClose(window)) {

    debug.new_frame();

    // per-frame time logic
    curr_frame = static_cast<float>(glfwGetTime());
    dt = curr_frame - prev_frame;
    prev_frame = curr_frame;

    // poll i/o events
    glfwPollEvents();
    process_inputs();

    // update model physics & positions
    update();

    // render scene
    render();
    debug.render(view, projection, glm::mat4(1.0f));
    // camera.debug();

    // imgui shit
    gui.new_frame();
    gui.update();
    gui.render();

    glfwSwapBuffers(window);
    // glfwSwapInterval(0); // VSYNC disabled; uncaps FPS
  }
}

bool Gravitas::init_glfw() {

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
  window = glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Gravitas", NULL, NULL);
  if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(window);         // make opengl context current immediately after window creation
  glfwSetWindowUserPointer(window, this); // Add this line
  glfwSetInputMode(window, GLFW_CURSOR,
      GLFW_CURSOR_CAPTURED); // tell glfw to capture our mouse

  return true;
}

bool Gravitas::init_glad() {

  // load all OpenGL function pointers
  // ---------------------------------------
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cout << "Failed to initialize GLAD" << std::endl;
    return false;
  }

  // configure global opengl state
  // -----------------------------
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_STENCIL_TEST);
  glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
  glStencilOp(GL_KEEP, GL_KEEP,
      GL_REPLACE); // pass/fail action for stencil test

  // glEnable(GL_BLEND);
  // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);      // Final color =
  // FragColor.rgb * FragColor.a + FrameBuff.rgb * (1 - FragColor.a) FragColor.a
  // close to 1 --> FragColor FragColor.a close to 0 --> FrameBuff color

  return true;
}

void Gravitas::init_callbacks() {

  // glfw initialize callbacks
  // -------------------------
  glfwSetFramebufferSizeCallback(window, &Gravitas::frame_buf_size_cb);
  glfwSetScrollCallback(window, &Gravitas::scroll_cb);
  glfwSetKeyCallback(window, &Gravitas::key_cb);
  glfwSetMouseButtonCallback(window, &Gravitas::mouse_btn_cb);
  glfwSetCursorPosCallback(window, &Gravitas::cursor_pos_cb);
}

int random(int min, int max) {
  std::random_device rand_dev;
  std::mt19937 generator(rand_dev());
  std::uniform_int_distribution<int> distr(min, max);
  return distr(generator);
}
glm::vec3 rand_pos(int min = -50, int max = 50) {
  return glm::vec3(random(min, max), random(min, max), random(min, max));
}

void Gravitas::init_scene() {

  // Compile Shaders
  default_shader.init("lightObject.vert", "lightObject.frag");
  lighting_shader.init("lightSource.vert", "lightSource.frag");
  picking_shader.init("pickingColor.vert", "pickingColor.frag");
  outline_shader.init("stencilOutline.vert", "stencilOutline.frag");

  // Load Models
  Model tetra_model("tetrahedron.obj");
  Model icosa_model("icosahedron.obj");
  Model cube_model("cube.obj");
  Model ball_model("ball.obj");
  Model cylinder_model("cilindru.obj");
  Model suzanne_model("suzanne.obj");
  Model teapot_model("teapot.obj");
  Model david_model("david.obj");
  Model cow_model("cow.obj");
  Model bunny_model("stanford-bunny.obj");
  Model dragon_model("dragon.obj");

  // Initialize Bodies -- Physics Properties, Collision Geometry,
  // Transformations
  RigidBody light(cube_model, 10.0, glm::vec3(0.0, 0.0, 5.0), true);
  RigidBody tetra(tetra_model, 50.0, glm::vec3(1.5, 7.0, 4.0));
  RigidBody icosa(icosa_model, 100.0, glm::vec3(3.0, 2.0, 3.0));
  RigidBody cube(cube_model, 50.0, glm::vec3(0.0, 3.0, 0.0));
  RigidBody cyl(cylinder_model, 5.0, glm::vec3(2.0, 2.0, 2.0));
  //  RigidBody ball(ball_model, 5.0, glm::vec3(3.0, 3.0, 3.0));
  RigidBody suzanne(suzanne_model, 1.0, glm::vec3(0.0, 0.0, 0.0));
  //      RigidBody teapot(teapot_model, 5.0, glm::vec3(0.0, 1.0, 0.0));
  //         RigidBody david(david_model, 5.0, glm::vec3(0.0, 0.0, 0.0));
  //      RigidBody cow(cow_model, 20.0, glm::vec3(4.0f, 4.0f, 4.0f));
  //         RigidBody bunny(bunny_model, 20.0, glm::vec3(4.0f, 4.0f, 4.0f));
  //         RigidBody dragon(dragon_model, 10.0, glm::vec3(2.0f, -2.0f, 2.0f));

  // Building Scene -- Dynamic BVH Tree
  scene.add_rigid_body(std::move(light));
  scene.add_rigid_body(std::move(tetra));
  // scene.add_rigid_body(std::move(cube));
  scene.add_rigid_body(std::move(icosa));
  //   scene.add_rigid_body(std::move(cyl));
  //        scene.add_rigid_body(std::move(ball));
  scene.add_rigid_body(std::move(suzanne));
  //            scene.add_rigid_body(std::move(teapot));
  //                 scene.addRigidBody(std::move(david));
  //            scene.add_rigid_body(std::move(cow));
  //              scene.add_rigid_body(std::move(bunny));
  //              scene.add_rigid_body(std::move(dragon));

  scene.set_debugger(&debug);

  // 100 Cubes -- 144 FPS
  // 1000 Cubes -- 30 FPS

  for (int i = 0; i < 25; i++) {
    RigidBody cube(cube_model, 50.0, glm::vec3(0.0f, i * 2.0f, 0.0f));
    scene.add_rigid_body(std::move(cube));
  }

  for (int i = 0; i < scene.bodies.size(); i++) {
    scene.bodies[i].color = color_palette[i % 10];
  }

  /*
  for (int i = 0; i < 50; i++) {
    RigidBody cube(cube_model, 5.0, rand_pos(-9, 9));
    scene.add_rigid_body(std::move(cube));
  }
  */
}

void Gravitas::process_inputs() {
  if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS) {
    glfwSetWindowShouldClose(window, true);
  }

  // rightMouseButton enables WASD movement
  if (right_mouse_btn.is_down) {

    if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
      camera.process_keyboard(FORWARD, dt);
    if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
      camera.process_keyboard(BACKWARD, dt);
    if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
      camera.process_keyboard(LEFT, dt);
    if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
      camera.process_keyboard(RIGHT, dt);
    if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
      camera.process_keyboard(UP, dt);
  }
  if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
    camera.process_keyboard(FORWARD, dt);
  if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
    camera.process_keyboard(BACKWARD, dt);
  if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
    camera.process_keyboard(LEFT, dt);
  if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
    camera.process_keyboard(RIGHT, dt);
  if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS)
    camera.process_keyboard(UP, dt);
}

// UPDATE
// ...............................................................................................................................
void Gravitas::update() {

  // update camera transformations
  projection = glm::perspective(glm::radians(camera.zoom), (float)SCREEN_WIDTH / (float)SCREEN_HEIGHT, 0.01f, 500.0f);
  view = camera.get_view_matrix();

  // update picking/dragging
  if (left_mouse_btn.is_down) {

    // convert 2D mouse coords to world space coords
    Ray ray = screen_to_world_ray(left_mouse_btn.x, left_mouse_btn.y, SCREEN_WIDTH, SCREEN_HEIGHT, projection, view);
    // rotating
    if (is_rotating && selected_object != -1) {

      scene.bodies[selected_object].rotate(left_mouse_btn.x_offset, left_mouse_btn.y_offset);

      // per-frame offsets -- rotations don't continue when mouse isn't moving
      left_mouse_btn.x_offset = 0.0f;
      left_mouse_btn.y_offset = 0.0f;

    }

    // dragging
    else if (is_dragging && selected_object != -1) {

      // find ray-plane intersection
      float t;
      if (ray_plane_intersect(ray, drag_plane.normal, drag_plane.point, t)) {

        // update new point of intersection
        new_position = ray.origin + ray.direction * t;
        glm::vec3 displacement = new_position - old_position;

        // update positions + velocities???
        scene.bodies[selected_object].drag(displacement);

        old_position = new_position;
      }
    }

    // check initial object selection
    else {

      // BRUTE FORCE -- SLOW
      // check ray-hull intersections for all bodies
      // should use Broadphase to raycast bvh first
      /*
      for (int i = 0; i < scene.bodies.size(); ++i) {

        float t;
        if (scene.bodies[i].raycast(ray, t)) {

          glm::vec3 hit_point = ray.origin + ray.direction * t;
          selected_object = i;
          scene.bodies[i].disable();

          std::cout << "PICKED RIGID BODY " << scene.bodies[i].id << std::endl;
          // record initial object pos
          old_position = hit_point;

          // define drag plane
          drag_plane.normal = -camera.front_vector;
          drag_plane.point = hit_point;

          break;
        }
      }
      */

      float t;
      int id;
      if (scene.raycast(ray, id, t)) {

        glm::vec3 hit_point = ray.origin + ray.direction * t;
        selected_object = id;
        scene.bodies[id].is_dragging = true;

        std::cout << "PICKED RIGID BODY " << scene.bodies[id].id << std::endl;
        // record initial object pos
        old_position = hit_point;

        // define drag plane
        drag_plane.normal = -camera.front_vector;
        drag_plane.point = hit_point;
      }

      // cannot translate & rotate object at same time
      is_dragging = !is_rotating;
    }
  }

  // use fixed timesteps for consistency
  // float physicsDT = std::clamp(deltaTime, 1.0f / 300.0f, 1.0f / 60.0f);
  // update physics
  scene.step(fixed_dt);
}

// RENDER
// ...............................................................................................................................
void Gravitas::render() {

  // framebuffer stuff
  glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

  // update transformations
  projection = glm::perspective(glm::radians(camera.zoom), (float)SCREEN_WIDTH / (float)SCREEN_HEIGHT, 0.01f, 500.0f);
  view = camera.get_view_matrix();

  for (int i = 0; i < scene.bodies.size(); ++i) {

    glm::mat4 model_matrix = scene.bodies[i].get_render_matrix();

    // draw convex mesh
    // debug.draw_mesh(scene.bodies[i].get_mesh(), scene.bodies[i].get_physics_matrix(), glm::vec3(1.0f, 0.0f, 0.0f));

    // picking shader
    if (i == selected_object) {

      // first pass - render object normally & write to stencil
      // buffer
      glStencilFunc(GL_ALWAYS, 1, 0xFF);
      glStencilMask(0xFF);
      glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);

      default_shader.use();

      // vert uniforms
      default_shader.set_mat4("projection", projection);
      default_shader.set_mat4("view", view);
      default_shader.set_mat4("model", model_matrix);

      // frag uniforms
      default_shader.set_vec3("objectColor", scene.bodies[i].color);
      default_shader.set_vec3("lightColor", 1.0f, 1.0f, 1.0f);
      default_shader.set_vec3("lightPos", scene.bodies[0].get_centre_of_mass());
      default_shader.set_vec3("viewPos", camera.position);

      scene.bodies[i].draw(default_shader);

      // draw outline
      // second pass - render scaled down object & disable
      // stencil writing
      glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
      glStencilMask(0x00);
      glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
      glDisable(GL_DEPTH_TEST);

      outline_shader.use();
      glm::vec3 com = scene.bodies[i].get_geometric_centroid();
      glm::mat4 t1 = glm::translate(glm::mat4(1.0f), com);
      glm::mat4 s = glm::scale(glm::mat4(1.0f), glm::vec3(1.1f));
      glm::mat4 t0 = glm::translate(glm::mat4(1.0f), -com);
      glm::mat4 outline_matrix = t1 * s * t0 * model_matrix;

      outline_shader.set_mat4("projection", projection);
      outline_shader.set_mat4("view", view);
      outline_shader.set_mat4("model", outline_matrix);

      scene.bodies[i].draw(outline_shader);

      glStencilFunc(GL_ALWAYS, 0, 0xFF);
      glStencilMask(0xFF);
      glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
      glEnable(GL_DEPTH_TEST);

    } else {

      glStencilFunc(GL_ALWAYS, 0, 0xFF);
      glStencilMask(0xFF);

      // light source
      if (i == 0) {
        lighting_shader.use();
        lighting_shader.set_mat4("gWVP", projection * view * model_matrix);
        scene.bodies[i].draw(lighting_shader);

      } else {

        default_shader.use();

        // vert uniforms
        default_shader.set_mat4("projection", projection);
        default_shader.set_mat4("view", view);
        default_shader.set_mat4("model", model_matrix);

        // frag uniforms
        default_shader.set_vec3("objectColor", scene.bodies[i].color);
        default_shader.set_vec3("lightColor", 1.0f, 1.0f, 1.0f);
        default_shader.set_vec3("lightPos", scene.bodies[0].get_centre_of_mass());
        default_shader.set_vec3("viewPos", camera.position);

        // scene.bodies[i].draw(default_shader, &debug);
        scene.bodies[i].draw(default_shader, NULL);
      }
    }

    // probably make a imgui toggle for this
    // draw wireframe collision mesh
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    picking_shader.use();
    // Transform &colliderTrans =
    // scene.bodies[i].hull.getWorldTransform();
    //  glm::vec3 colliderPosition =
    //  colliderTrans.GetPosition();
    //  colliderTrans.SetPosition(colliderPosition *
    //  PHYSICS_SCALE);
    picking_shader.set_mat4("gWVP", projection * view * model_matrix);
    // need to determine how to implement collider draw calls
    // scene.bodies[i].collider.hull.draw(picking_shader);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    // reset to default state after rendering
    glStencilMask(0xFF);
    glStencilFunc(GL_ALWAYS, 0, 0xFF);
    glEnable(GL_DEPTH_TEST);
  }
}

// WHY DON'T WE JUST DECLARE CB FUNCTIONS AS STATIC
//
//
//
//
void Gravitas::on_frame_buf(int width, int height) {

  SCREEN_WIDTH = width;
  SCREEN_HEIGHT = height;
  glViewport(0, 0, width, height);

  // BAD Solution for dynamic window resizing
  // Render();
}
void Gravitas::on_scroll(double x_offset, double y_offset) {
  camera.process_mouse_scroll(static_cast<float>(y_offset));
}
void Gravitas::on_key(int key, int scancode, int action, int mods) {

  static int active_key = -1;

  // handle press & release actions
  if (action == GLFW_PRESS) {

    // nothing currently active, claim key
    if (active_key == -1) {
      if (key == GLFW_KEY_Q || key == GLFW_KEY_W || key == GLFW_KEY_E || key == GLFW_KEY_R || key == GLFW_KEY_1 ||
          key == GLFW_KEY_2 || key == GLFW_KEY_3 || key == GLFW_KEY_4)
        active_key = key;
    }
    // ignore additional presses while another key is active
  } else if (action == GLFW_RELEASE) {

    // If the released key is currently active key, clear it
    if (key == active_key)
      active_key = -1;
  }

  // specific key actions
  switch (active_key) {
  case GLFW_KEY_Q:
    // drag_plane.normal = camera.up_vector;
    drag_plane.normal = glm::vec3(0.0f, 1.0f, 0.0f); // XZ-plane
    break;

  case GLFW_KEY_W:
    drag_plane.normal = camera.right_vector;
    // drag_plane.normal = glm::vec3(1.0f, 0.0f, 0.0f); // YZ-plane
    break;

  case GLFW_KEY_E:
    drag_plane.normal = camera.front_vector;
    // drag_plane.normal = glm::vec3(0.0f, 0.0f, 1.0f); // XY-plane
    break;

  case GLFW_KEY_R:
    is_rotating = true; // rotate flag
                        // disable cursor
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    break;

  case GLFW_KEY_1:
    camera.reset(glm::vec3(0.0f, 0.0f, 20.0f), glm::vec3(0.0f, 0.0f, -1.0f), glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, 0.0f);
    break;

  case GLFW_KEY_2:
    camera.reset(
        glm::vec3(20.0f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f), glm::vec3(0.0f, 1.0f, 0.0f), -180.0f, 0.0f);
    break;

  case GLFW_KEY_3:
    camera.reset(
        glm::vec3(0.0f, 0.0f, -20.0f), glm::vec3(0.0f, 0.0f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f), -270.0f, 0.0f);
    break;

  case GLFW_KEY_4:
    camera.reset(
        glm::vec3(0.0f, 20.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f), glm::vec3(0.0f, 0.0f, -1.0f), -90.0f, -90.0f);
    break;

  default:
    is_rotating = false;
    is_dragging = false;
    selected_object = -1;
    drag_plane.normal = -camera.front_vector;
    break;
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

void Gravitas::on_mouse_move(double x, double y) {

  float curr_x = static_cast<float>(x);
  float curr_y = static_cast<float>(y);

  if (right_mouse_btn.is_down) {

    // process initial mouse input
    if (first_mouse) {

      prev_x = curr_x;
      prev_y = curr_y;
      first_mouse = false;
    }

    float x_offset = curr_x - prev_x;
    float y_offset = prev_y - curr_y; // y-axis goes from bottom to top
    camera.process_mouse_movement(x_offset, y_offset);
  }

  if (left_mouse_btn.is_down) {

    left_mouse_btn.x = curr_x;
    left_mouse_btn.y = curr_y;
    left_mouse_btn.x_offset = curr_x - prev_x;
    left_mouse_btn.y_offset = prev_y - curr_y;
  }

  prev_x = curr_x;
  prev_y = curr_y;
}

void Gravitas::on_mouse_press(int button, int action, int mods) {

  // retrieve cursor coords.
  double cursor_x, cursor_y;
  glfwGetCursorPos(window, &cursor_x, &cursor_y);

  float x = static_cast<float>(cursor_x);
  float y = static_cast<float>(cursor_y);

  // leftMouseButton controls picking/dragging movement
  if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {

    // enable and capture cursor for picking
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_CAPTURED);

    left_mouse_btn.is_down = true;
    left_mouse_btn.x = x;
    left_mouse_btn.y = y;

  }

  // rightMouseButton controls camera/wasd movement
  else if (button == GLFW_MOUSE_BUTTON_RIGHT && action == GLFW_PRESS) {

    // disable cursor
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    right_mouse_btn.is_down = true;
    first_mouse = true;
    right_mouse_btn.x = x;
    right_mouse_btn.y = y;

  }

  // handle release
  else if (action == GLFW_RELEASE) {

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
      left_mouse_btn.is_down = false;

      if (selected_object != -1) {
        scene.bodies[selected_object].is_dragging = false;
      }

      // stop dragging/rotating on release
      is_dragging = false;
      is_rotating = false;

      // reset object selection
      left_mouse_btn.first_mouse = true;
      selected_object = -1;

      if (!right_mouse_btn.is_down) {

        // normal cursor if no buttons are pressed
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
      }
    }

    else if (button == GLFW_MOUSE_BUTTON_RIGHT) {

      right_mouse_btn.is_down = false;
      if (!left_mouse_btn.is_down) {

        // normal cursor if no buttons are pressed
        glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
      }
    }
  }
}

void Gravitas::frame_buf_size_cb(GLFWwindow *w, int a, int b) {
  static_cast<Gravitas *>(glfwGetWindowUserPointer(w))->on_frame_buf(a, b);
}
void Gravitas::scroll_cb(GLFWwindow *w, double x, double y) {
  static_cast<Gravitas *>(glfwGetWindowUserPointer(w))->on_scroll(x, y);
}
void Gravitas::key_cb(GLFWwindow *w, int k, int s, int a, int m) {
  static_cast<Gravitas *>(glfwGetWindowUserPointer(w))->on_key(k, s, a, m);
}
void Gravitas::mouse_btn_cb(GLFWwindow *w, int b, int a, int m) {
  static_cast<Gravitas *>(glfwGetWindowUserPointer(w))->on_mouse_press(b, a, m);
}
void Gravitas::cursor_pos_cb(GLFWwindow *w, double x, double y) {
  static_cast<Gravitas *>(glfwGetWindowUserPointer(w))->on_mouse_move(x, y);
}

int main() {
  Gravitas engine;
  if (!engine.init())
    return -1;
  engine.run();
  return 0;
}
