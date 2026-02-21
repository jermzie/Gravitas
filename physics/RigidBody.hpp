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
  // BoundingSphere local_sphere;

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
} primitive_type;

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
  // Geometry & collisions

  // Transform (world space)
  glm::vec3 position;
  glm::quat orientation;

  // Physics state
  MassProperties properties;
  glm::vec3 lin_velocity = glm::vec3(0.0f);
  glm::vec3 lin_momentum = glm::vec3(0.0f);
  glm::vec3 ang_velocity = glm::vec3(0.0f);
  glm::vec3 ang_momentum = glm::vec3(0.0f);

  glm::vec3 force;
  glm::vec3 torque;

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
  // mass properties
  float mass;
  float inv_mass;
  float density;
  float friction;

public:
  Transform transform;
  CollisionData collider;

  int id;
  bool is_static = false;
  bool is_dragging = false;

  void apply_force(glm::vec3 f, glm::vec3 point);

  void integrate(float dt);

  /*
  AABB get_world_aabb() const {
    AABB local_box = collider.hull.compute_aabb();
    // need to transfor to world?
    return local_box;
  }
  */

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

  ConvexMesh get_mesh() const { return collider.hull.get_mesh(); }

  RigidBody(const Model &model, const float &density, const glm::vec3 &position) {

    this->render_model = model;
    this->position = position;
    this->density = density;
    this->orientation = glm::quat(1, 0, 0, 0);

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

  /*
  // body created by singular model -- often imported models
  RigidBody(Model& model, float rho, glm::vec3 position, glm::vec3 velocity,
            glm::vec3 L) {

    // basic physics properties
    rigidbody_model = model;
    density = rho;
    lin_velocity = velocity;
    ang_momentum = L;

    // compute convex hull & half-edge mesh of model
    QuickHull qh;
    ConvexMesh mesh = qh.build_mesh(render_model.get_vertex_data());
    glm::vec3 geometric_centre;

    extrema = qh.get_extrema_vertices();

    glm::vec3 model_origin_offset = hull.recentre_to_origin();

    mass_props = MassProperties::compute(mesh, density);

    collision.mesh = mesh;
    collision.local_aabb;
    collision.local_sphere;

    // compute aabb
    box.init(extrema);

    // local com & inertia
    hull.computeMassProperties(density, mass, local_com_offset, inertia);
    inv_inertia = glm::inverse(inertia);
    com = position + local_com_offset;

    // transformations
    world_transform.SetAbsPosition(position);
    hull.getWorldTransform().SetAbsPosition(position);
    hull.worldCentroid += position;

    // model
    hull.getHullModel(rigidbody_model, world_transform);
  }
  */

  /*
  // body created by collection of convex meshes -- manually defined
  RigidBody(std::vector<ConvexHull>models, float rho, glm::vec3 position,
  glm::vec3 velocity, glm::vec3 L) {

          // basic physics properties
          rigidbody_model = models;
          density = rho;
          lin_velocity = velocity;
          ang_momentum = L;

  }
  */

  // apply force & torque
  void integrateForces(double dt) {

    glm::vec3 gravity(0.0f, -1.0f, 0.0f);
    lin_velocity += gravity * dt;
  }

  void update(float dt) {

    if (is_static) {

      // DO SOMETHING
    } else {

      // LINEAR MOTION
      glm::vec3 dt_position = dt * lin_velocity;
      position += dt_position;

      // ANGULAR MOTION
      glm::mat3 R = glm::mat3_cast(orientation);
      glm::mat3 world_inertia = R * properties.inv_inertia_tensor * glm::transpose(R);
      ang_velocity = world_inertia * ang_momentum;

      // Update orientation
      glm::quat omega_quat(0.0f, ang_velocity.x, ang_velocity.y, ang_velocity.z);
      glm::quat dt_orientation = 0.5f * dt * omega_quat * orientation;
      orientation += dt_orientation;
      orientation = glm::normalize(orientation);

      // TRANSFORMATION MATRIX
      // transform.set_displacement(dt_position);
      // transform.set_rotation(dt_orientation);
      //
      update_transform();
    }
  }

  void drag(glm::vec3 delta) {

    // dragVelocity = 100.0f * displacement;
    //  update model transformations

    // Update transform
    // transform.set_displacement(delta);

    // Update COM position
    position += delta;
    update_transform();
    // track xyz offSets and deltaTime to accumulate velocity while dragging???
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

  // Body is static and unable to move
  void disable() {

    is_static = true;
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

  void draw(Shader &shader) { render_model.draw(shader); }

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
