#include "Gravitas.hpp"
#include "Geometry.hpp"
#include "MassProperties.hpp"
#include "Primitive.hpp"
#include <memory>

static Entity make_entity(std::unique_ptr<Geometry> geom, float density,
                          glm::vec3 pos,
                          glm::quat rot = glm::quat(1.0f, 0.0f, 0.0f, 0.0f)) {
  glm::vec3 com_offset;
  MassProperties props = geom->finalize(density, com_offset);
  return Entity(
      std::make_unique<RigidBody>(props, density, pos + rot * com_offset, rot),
      std::move(geom));
}

static std::unique_ptr<Geometry> single(std::shared_ptr<Primitive> prim) {
  auto geom = std::make_unique<Geometry>();
  geom->add_child(std::move(prim));
  return geom;
}

Gravitas::Gravitas(unsigned int width, unsigned int height)
    : SCREEN_WIDTH(width), SCREEN_HEIGHT(height) {
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

    // imgui shit
    gui.new_frame();
    gui.update();
    gui.render();

    glfwSwapBuffers(window);
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
  window =
      glfwCreateWindow(SCREEN_WIDTH, SCREEN_HEIGHT, "Gravitas", NULL, NULL);
  if (window == nullptr) {
    std::cout << "Failed to create GLFW window" << std::endl;
    glfwTerminate();
    return false;
  }

  glfwMakeContextCurrent(
      window); // make opengl context current immediately after window creation
  glfwSwapInterval(1); // VSYNC enabled; 0 uncaps FPS
  glfwSetWindowUserPointer(window, this);
  glfwSetInputMode(window, GLFW_CURSOR,
                   GLFW_CURSOR_CAPTURED); // tell glfw to capture our mouse

  return true;
}

bool Gravitas::init_glad() {
  // load all opengl function pointers
  // ---------------------------------
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

void Gravitas::init_scene() {
  // Compile Shaders
  default_shader.init("lightObject.vert", "lightObject.frag");
  lighting_shader.init("lightSource.vert", "lightSource.frag");
  picking_shader.init("pickingColor.vert", "pickingColor.frag");
  outline_shader.init("stencilOutline.vert", "stencilOutline.frag");

  // Load Models
  auto suzanne_model = std::make_shared<Model>("suzanne.glb");

  Entity cube = make_entity(single(std::make_shared<Polyhedron>()), 20.0f,
                            {0.0f, 0.0f, 0.0f});
  Entity sphere = make_entity(single(std::make_shared<Sphere>()), 20.0f,
                              {0.0f, 0.0f, 0.0f});
  Entity frustum = make_entity(single(std::make_shared<Frustum>()), 20.0f,
                               {0.0f, 0.0f, 0.0f});
  Entity capsule = make_entity(single(std::make_shared<Capsule>()), 20.0f,
                               {-2.0f, 2.0f, 0.0f});
  Entity suzanne =
      make_entity(single(std::make_shared<Polyhedron>(suzanne_model)), 20.0f,
                  {0.0f, 5.0f, 0.0f});

  Entity dumbbell =
      make_entity(Geometry::make_dumbbell(), 20.0f, {1.0f, 1.0f, 1.0f});

  Entity table = make_entity(Geometry::make_table(), 20.0f, {1.0f, 1.0f, 1.0f});

  sphere.color = color_palette[0];
  frustum.color = color_palette[1];
  suzanne.color = color_palette[2];
  capsule.color = color_palette[3];
  dumbbell.color = color_palette[4];
  table.color = color_palette[5];
  scene.add_entity(std::move(cube));
  scene.add_entity(std::move(sphere));
  scene.add_entity(std::move(frustum));
  scene.add_entity(std::move(capsule));
  scene.add_entity(std::move(suzanne));
  scene.add_entity(std::move(dumbbell));
  scene.add_entity(std::move(table));

  // after the scene entities - render() treats entities[0] as the light source
  scene.add_bounds(50.0f, 10.0f);

  scene.set_debugger(&debug);
  gui.set_physics_engine(&scene);
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
  }
  if (glfwGetKey(window, GLFW_KEY_UP) == GLFW_PRESS)
    camera.process_keyboard(FORWARD, dt);
  if (glfwGetKey(window, GLFW_KEY_DOWN) == GLFW_PRESS)
    camera.process_keyboard(BACKWARD, dt);
  if (glfwGetKey(window, GLFW_KEY_LEFT) == GLFW_PRESS)
    camera.process_keyboard(LEFT, dt);
  if (glfwGetKey(window, GLFW_KEY_RIGHT) == GLFW_PRESS)
    camera.process_keyboard(RIGHT, dt);
  if (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS) {
    bool shift = glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS;
    camera.process_keyboard(shift ? DOWN : UP, dt);
  }
}

void Gravitas::update() {
  // update camera transformations
  projection = glm::perspective(glm::radians(camera.zoom),
                                (float)SCREEN_WIDTH / (float)SCREEN_HEIGHT,
                                0.01f, 500.0f);
  view = camera.get_view_matrix();

  // update picking/dragging
  if (left_mouse_btn.is_down) {

    // convert 2D mouse coords to world space coords
    const Ray &ray =
        screen_to_world_ray(left_mouse_btn.x, left_mouse_btn.y, SCREEN_WIDTH,
                            SCREEN_HEIGHT, projection, view);
    // rotating
    if (is_rotating && selected_object != -1) {

      scene.entities[selected_object].rotate(
          camera.right_vector, camera.up_vector, left_mouse_btn.x_offset,
          left_mouse_btn.y_offset);

      // per-frame offsets -- rotations don't continue when mouse isn't moving
      left_mouse_btn.x_offset = 0.0f;
      left_mouse_btn.y_offset = 0.0f;

    }

    // dragging
    else if (is_dragging && selected_object != -1) {

      // find ray-plane intersection
      RayHit query;
      if (ray_plane_intersect(ray, drag_plane, query)) {

        // update new point of intersection
        glm::vec3 new_position = query.hit_point;
        glm::vec3 displacement = new_position - old_position;

        // update positions + velocities???
        scene.entities[selected_object].drag(displacement);

        old_position = new_position;
      }
    }

    // check initial object selection
    else {

      size_t idx;
      RayHit query;
      if (scene.raycast(ray, idx, query)) {

        selected_object = idx;
        scene.entities[idx].is_dragging = true;

        CLOGI("PICKED RIGID BODY %d", scene.entities[idx].id);

        // record initial object pos
        old_position = query.hit_point;

        // define drag plane
        drag_plane.normal = -camera.front_vector;
        drag_plane.point = query.hit_point;
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

  // default stencil/depth state - outline pass overrides it
  auto reset_gl_state = [] {
    glStencilFunc(GL_ALWAYS, 0, 0xFF);
    glStencilMask(0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_REPLACE);
    glEnable(GL_DEPTH_TEST);
  };

  auto draw_lit = [&](Entity &entity, const glm::mat4 &model_matrix) {
    default_shader.use();

    // vert uniforms - "model" is set per child by Entity::draw
    default_shader.set_mat4("projection", projection);
    default_shader.set_mat4("view", view);

    // frag uniforms
    default_shader.set_vec3("objectColor", entity.color);
    default_shader.set_vec3("lightColor", 1.0f, 1.0f, 1.0f);
    default_shader.set_vec3("lightPos",
                            scene.entities[0].body->get_centre_of_mass());
    default_shader.set_vec3("viewPos", camera.position);

    entity.draw(default_shader, model_matrix, "model", &debug);
  };

  reset_gl_state();
  for (int i = 0; i < scene.entities.size(); ++i) {

    Entity &entity = scene.entities[i];
    if (!entity.is_visible) {
      continue;
    }
    glm::mat4 model_matrix = entity.body->get_physics_matrix();

    if (i == selected_object) {
      // first pass - render object normally & write to stencil buffer
      glStencilFunc(GL_ALWAYS, 1, 0xFF);
      draw_lit(entity, model_matrix);

      // second pass - render scaled up object & disable stencil writing
      glStencilFunc(GL_NOTEQUAL, 1, 0xFF);
      glStencilMask(0x00);
      glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);
      glDisable(GL_DEPTH_TEST);

      // body xfrm is unscaled & its origin is the COM - scaling in model
      // space grows the outline about the COM
      glm::mat4 outline_matrix =
          model_matrix * glm::scale(glm::mat4(1.0f), glm::vec3(1.1f));

      outline_shader.use();
      outline_shader.set_mat4("projection", projection);
      outline_shader.set_mat4("view", view);
      entity.draw(outline_shader, outline_matrix);

      reset_gl_state();

    } else if (i == 0) {
      // light source
      lighting_shader.use();
      entity.draw(lighting_shader, projection * view * model_matrix, "gWVP");

    } else {
      draw_lit(entity, model_matrix);
    }
  }
}

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
      if (key == GLFW_KEY_Q || key == GLFW_KEY_W || key == GLFW_KEY_E ||
          key == GLFW_KEY_R || key == GLFW_KEY_1 || key == GLFW_KEY_2 ||
          key == GLFW_KEY_3 || key == GLFW_KEY_4)
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
    camera.reset(glm::vec3(0.0f, 0.0f, 20.0f), glm::vec3(0.0f, 0.0f, -1.0f),
                 glm::vec3(0.0f, 1.0f, 0.0f), -90.0f, 0.0f);
    break;

  case GLFW_KEY_2:
    camera.reset(glm::vec3(20.0f, 0.0f, 0.0f), glm::vec3(-1.0f, 0.0f, 0.0f),
                 glm::vec3(0.0f, 1.0f, 0.0f), -180.0f, 0.0f);
    break;

  case GLFW_KEY_3:
    camera.reset(glm::vec3(0.0f, 0.0f, -20.0f), glm::vec3(0.0f, 0.0f, 1.0f),
                 glm::vec3(0.0f, 1.0f, 0.0f), -270.0f, 0.0f);
    break;

  case GLFW_KEY_4:
    camera.reset(glm::vec3(0.0f, 20.0f, 0.0f), glm::vec3(0.0f, -1.0f, 0.0f),
                 glm::vec3(0.0f, 0.0f, -1.0f), -90.0f, -90.0f);
    break;

  default:
    is_rotating = false;
    is_dragging = false;
    selected_object = -1;
    drag_plane.normal = -camera.front_vector;
    break;
  }
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

  }

  // handle release
  else if (action == GLFW_RELEASE) {

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
      left_mouse_btn.is_down = false;

      if (selected_object != -1) {
        scene.entities[selected_object].is_dragging = false;
      }

      // stop dragging/rotating on release
      is_dragging = false;
      is_rotating = false;

      // reset object selection
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
