#pragma once

// NOT DONE

#include "ConvexMesh.hpp"
#include <glm/ext/matrix_transform.hpp>
#include <glm/ext/quaternion_trigonometric.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <glm/gtx/scalar_multiplication.hpp>
#include <glm/gtx/string_cast.hpp>

#include <vector>

#include "AABB.hpp"
#include "Debugger.hpp"
#include "Mesh.hpp"
#include "Model.hpp"
#include "Ray.hpp"
#include "Transform.hpp"
// #include "BoundingSphere.hpp"
#include "CollisionGeometry.hpp"
#include "MassProperties.hpp"
#include "QuickHull.hpp"

struct CollisionData {
  CollisionGeometry hull;
  AABB local_aabb;

  // TODO: Compute local aabb once from extrema, update with world space transform for broadphase
  AABB get_world_aabb(const Transform &transform) const {
    return local_aabb.transform_arvo(local_aabb, transform.get_matrix());
  }
  // BoundingSphere get_world_sphere(const Transform &transform) const;
};

typedef enum {
  TETRAHEDRON,
  CUBE,
  SPHERE,
  CONE,
  CYLINDER,
  CAPSULE,
} PrimitiveType;

// ? IS THIS NEEDED?
struct rigid_body_config_t {
  Model &model;
  float mass;
  float density;
  glm::vec3 position;
  glm::vec3 lin_velocity;
  glm::vec3 ang_velocity;
};

class RigidBody {
private:
  glm::vec3 force_net = glm::vec3(0.0f);
  glm::vec3 torque_net = glm::vec3(0.0f);

  // Rendering
  Model render_model;                // Just for drawing
  glm::vec3 model_origin_offset;     // Offset from COM to model origin
  glm::vec3 geometric_origin_offset; // Offset from COM to geometric origin
                                     //
  /*
  glm::vec3 local_com_offset;
  std::array<float, 6U> extrema;

  // linear motion
  glm::vec3 com;

  // angular motion
  // glm::mat3orientation = glm::mat3(1.0);
  glm::mat3 inertia = glm::mat3(1.0);
  glm::mat3 inv_inertia = glm::mat3(1.0);
*/
  float density;
  float baumgarte_factor;
  float restitution;

public:
  Transform transform;
  CollisionData collider;
  MassProperties properties;

  // FIXME: TEMPORARY MAKE PUBLIC
  glm::vec3 position;
  glm::quat orientation;
  glm::vec3 lin_velocity = glm::vec3(0.0f);
  glm::vec3 ang_velocity = glm::vec3(0.0f);
  glm::vec3 lin_momentum = glm::vec3(0.0f);
  glm::vec3 ang_momentum = glm::vec3(0.0f);
  int id;
  bool is_static = false;
  bool is_dragging = false;

  bool raycast(const Ray &world_ray, float &t) const {

    glm::mat4 inv_transform = transform.get_inverse_matrix();
    // Ray local_ray = transform.world_to_local(world_ray);
    // return collision.mesh.ray_intersect(local_ray, t);

    Ray local_ray;
    local_ray.origin = glm::vec3(inv_transform * glm::vec4(world_ray.origin, 1.0f));
    local_ray.direction = glm::vec3(inv_transform * glm::vec4(world_ray.direction, 0.0f));

    // glm::vec3 local_hit_point;
    int hit = collider.hull.raycast(local_ray, t);

    if (hit != 0) {
      // glm::mat4 world_transform = const_cast<Transform
      // &>(transform).get_matrix();
      glm::mat4 world_transform = transform.get_matrix();
      glm::vec3 local_hit_point = local_ray.origin + local_ray.direction * t;
      // hit_point = glm::vec3(world_transform *
      // glm::vec4(local_hit_point, 1.0f));
      //  hit_point = glm::vec3(world_transform *
      //  glm::vec4(local_hit_point, 1.0f));
      return true;
    }

    return false;
  }

  void update_transform() {
    transform.set_position(position);
    transform.set_orientation(orientation);
  }

  glm::mat4 get_render_matrix() const {

    glm::mat4 base = transform.get_matrix(); // Local COM space -> World space
    glm::mat4 offset = glm::translate(glm::mat4(1.0f), -model_origin_offset);
    return base * offset; // Model -> COM -> World
  }

  glm::mat4 get_physics_matrix() const {
    // glm::mat4 T = glm::translate(glm::mat4(1.0f), position);
    // glm::mat4 R = glm::mat4_cast(orientation);
    // return T * R;
    return transform.get_matrix(); // Local COM space -> World space
  }

  const ConvexMesh &get_mesh() const { return collider.hull.get_mesh(); }

  RigidBody(const Model &model, const float &density, const glm::vec3 &position, const bool &is_static = false) {

    this->render_model = model;
    this->position = position;
    this->density = density;
    this->orientation = glm::quat(1, 0, 0, 0);
    this->is_static = is_static;

    // 1. Build collision geometry
    QuickHull qh;
    ConvexMesh mesh = qh.build_convex_mesh(model.get_vertex_data());

    // 2. Compute mass properties
    properties = MassProperties::compute(mesh, density);
    model_origin_offset = properties.centre_of_mass;

    // 3. Shift vertices so COM centered at origin
    for (glm::vec3 &v : mesh.vertices) {
      v -= properties.centre_of_mass;
    }

    // 3.5 UPDATE PLANES TO MATCH NEW VERTEX POSITIONS
    for (auto &face : mesh.faces) {
      // Recalculate plane.point using shifted vertices
      auto verts = mesh.get_face_vertices(face);
      face.plane.point = mesh.vertices[verts[0]];

      // Recalculate plane.distance (assuming normal is still correct)
      face.plane.distance = -glm::dot(face.plane.normal, face.plane.point);
    }
    // 4. Recompute mass properties (COM should be ~(0, 0, 0))
    properties = MassProperties::compute(mesh, density);
    geometric_origin_offset = mesh.compute_geometric_centroid();

    // 5. Build collision geometry
    collider.hull = CollisionGeometry(mesh);
    // FIXME: Reuse extrema + scale computed from quickhull construction
    collider.local_aabb = AABB(mesh.get_extrema());

    // 6. Transform into world space
    update_transform();

    std::cout << "CONVEX MESH VERTICES: " << mesh.vertices.size() << std::endl;
    std::cout << "CONVEX MESH FACES: " << mesh.faces.size() << std::endl;
    std::cout << "CONVEX MESH EDGES: " << mesh.half_edges.size() << std::endl;
  }

  void apply_global_force(glm::vec3 f) { force_net += f; }

  void apply_point_force(glm::vec3 f, glm::vec3 p) {
    force_net += f;
    glm::vec3 r = p - position;
    torque_net += glm::cross(r, f);
  }

  // Integral is just infinite sum of small intervals
  void integrate_forces(double dt) {

    glm::vec3 gravity(0.0f, -9.81f, 0.0f);
    force_net += gravity * properties.mass;

    // Use Euler's rule

    // Integrate linear velocity
    // \Delta v(t) = \frac{1}{m} \int F(t) dt

    lin_velocity += properties.inv_mass * force_net * dt;

    // Integrate angular velocity
    // \Delta \omega (t) = I^{-1} \int \tau (t) dt
    glm::mat3 R = glm::mat3_cast(orientation);
    glm::mat3 world_inv_inertia = R * properties.inv_inertia_tensor * glm::transpose(R);
    ang_velocity += world_inv_inertia * torque_net * dt;

    /*
    // Integrate position
    position += lin_velocity * dt;

    // Integrate orientation
    glm::quat omega_quat(0.0f, ang_velocity);
    glm::quat dt_orientation = 0.5f * (float)dt * omega_quat * orientation;
    orientation += dt_orientation;
    orientation = glm::normalize(orientation);

    // Update transformation matrix
    update_transform();
    */

    // Reset forces
    force_net = glm::vec3(0.0f);
    torque_net = glm::vec3(0.0f);
  }

  /*
  // WARNING: OLD. MAKES NO SENSE
  // apply force & torque
  void integrate_forces(double dt) {
    glm::vec3 gravity(0.0f, -9.81f, 0.0f);

    // Accumulate forces
    force += properties.mass * gravity; // F_g = m*g

    // Integrate momentum
    lin_momentum += force * dt;
    ang_momentum += torque * dt;

    // Derive velocity
    lin_velocity = lin_momentum * properties.inv_mass; // v = p/m

    force = glm::vec3(0.0f);
    torque = glm::vec3(0.0f);

    lin_velocity += gravity * dt;
  }
  */

  void update(float dt) {

    if (is_static || is_dragging) {

      // DO SOMETHING
      return;
    } else {

      // LINEAR POS
      glm::vec3 dt_position = dt * lin_velocity;
      position += dt_position;

      // ANGULAR POS
      glm::quat omega_quat(0.0f, ang_velocity);
      glm::quat dt_orientation = 0.5f * (float)dt * omega_quat * orientation;
      orientation += dt_orientation;
      orientation = glm::normalize(orientation);

      update_transform();
    }
  }

  void drag(glm::vec3 delta) {

    // Drag rigid body
    position += delta;

    // TODO: Compute implied velocity
    // lin_velocity = delta / dt;
    // lin_momentum = properties.mass * lin_velocity;

    // Stop angular motion
    ang_velocity = glm::vec3(0.0f);
    ang_momentum = glm::vec3(0.0f);

    // Update transformation matrix
    update_transform();
  }

  void rotate(float x_offset, float y_offset) {

    x_offset *= 0.01f;
    y_offset *= 0.01f;

    // Movement along y-axis rotates about x-axis and vice versa
    glm::quat rot_x = glm::angleAxis(-y_offset, glm::vec3(1.0f, 0.0f, 0.0f));
    glm::quat rot_y = glm::angleAxis(x_offset, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::quat rotation = rot_y * rot_x;

    // transform.set_rotation(rotation);

    orientation = rotation * orientation;
    orientation = glm::normalize(orientation);

    update_transform();

    /*
    printf("Rotation applied: x_offset=%.3f, y_offset=%.3f\n", x_offset,
           y_offset);
    printf("New orientation: w=%.3f, x=%.3f, y=%.3f, z=%.3f\n", orientation.w,
           orientation.x, orientation.y, orientation.z);
    */
    // SAME ISSUE AS BEFORE. IF MODEL ORIGIN NOT AT CENTRE OF MASS. ROTATION
    // FAILS.
    // world_transform.SetRelRotation(rotation);
    // hull.getWorldTransform().SetRelRotation(rotation);
  }

  // disable dynamics
  void disable() {
    is_dragging = true;
    lin_velocity = glm::vec3(0.0f);
  }

  // Body is reset to provided position
  void reset(glm::vec3 pos) {

    position = pos;
    lin_velocity = glm::vec3(0.0f);
    update_transform();
  }

  // Let collider handle raycasting, not physics object
  /*
  bool ray_intersection(Ray &worldRay, glm::vec3 &hitPoint) {

    glm::mat4 inverseModelMatrix = glm::inverse(world_transform.GetMatrix());
    glm::vec4 rayOrig_local =
        inverseModelMatrix * glm::vec4(worldRay.origin, 1.0f);
    glm::vec4 rayDir_local =
        inverseModelMatrix * glm::vec4(worldRay.direction, 0.0f);

    Ray localRay;
    localRay.origin = glm::vec3(rayOrig_local);
    localRay.direction = glm::vec3(rayDir_local);

    // t in local space
    float t;
    int res = hull.computeRayIntersection(localRay, t);

    glm::vec3 hitLocal = localRay.origin + localRay.direction * t;
    glm::vec4 hitWorld4 =
        hull.getWorldTransform().GetMatrix() * glm::vec4(hitLocal, 1.0f);
    hitPoint = glm::vec3(hitWorld4);

    return res;
  }
  */

  void draw(Shader &shader, Debugger *debug = nullptr) {

    if (debug) {
      debug->draw_mesh(collider.hull.get_mesh(), transform.get_matrix(), glm::vec3(0.0f, 1.0f, 0.0f));
    }
    render_model.draw(shader);
  }

  glm::mat3 get_world_inverse_inertia() const {
    glm::mat3 R = glm::mat3_cast(orientation);
    return R * properties.inv_inertia_tensor * glm::transpose(R);
  }

  // Returns world space position
  glm::vec3 get_centre_of_mass() const { return position; }

  glm::vec3 get_local_centre_of_mass() const { return properties.centre_of_mass; }

  glm::vec3 get_local_geometric_centroid() const { return geometric_origin_offset; }
  // Returns geometric_centroid in world space
  glm::vec3 get_geometric_centroid() const {

    // Get offset translation matrix
    // glm::mat4 offset =
    //    glm::translate(glm::mat4(1.0f), -geometric_origin_offset);

    return glm::vec3(transform.get_matrix() * glm::vec4(geometric_origin_offset, 1.0f));
  }
};
