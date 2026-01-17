#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/orthonormalize.hpp>
#include <glm/gtx/scalar_multiplication.hpp>
#include <glm/gtx/string_cast.hpp>

#include <vector>

#include "../inc/Mesh.hpp"
#include "../inc/Model.hpp"
#include "../inc/Ray.hpp"
#include "../inc/WorldTransform.hpp"
#include "AABB.hpp"
#include "BoundingSphere.hpp"
#include "ConvexHull.hpp"
#include "QuickHull.hpp"

class RigidBody {
private:
  std::vector<ConvexHull> body; // collection of convex hulls

  WorldTransform world_transform;
  Model rigidbody_model;
  glm::vec3 local_com_offset;
  std::array<float, 6U> extrema;

  // linear motion
  glm::vec3 com;
  glm::vec3 lin_velocity;
  glm::vec3 lin_momentum;

  // angular motion
  glm::mat3 orientation = glm::mat3(1.0);
  glm::vec3 ang_velocity;
  glm::vec3 ang_momentum;
  glm::mat3 inertia = glm::mat3(1.0);
  glm::mat3 inv_inertia = glm::mat3(1.0);

  // mass properties
  float mass;
  float inv_mass;
  float density;
  float friction;

public:
  int id;
  bool is_static = false;
  bool is_dragging = false;

  BoundingSphere sphere;
  AABB box;
  ConvexHull hull;

  // body created by singular model -- often imported models
  RigidBody(Model model, float rho, glm::vec3 position, glm::vec3 velocity,
            glm::vec3 L) {

    // basic physics properties
    rigidbody_model = model;
    density = rho;
    lin_velocity = velocity;
    ang_momentum = L;

    // compute convex hull & half-edge mesh of model
    QuickHull qh;
    hull = qh.getConvexHull(rigidbody_model.GetVertexData());
    extrema = qh.getExtremaVertices();

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
  void update(double delta_time) {

    if (is_static) {

      // DO SOMETHING

    } else {

      // linear motion
      // lin_velocity += gravity * deltaTime;
      com += lin_velocity * delta_time;

      // angular motion
      glm::mat3 world_inertia =
          orientation * inv_inertia * glm::transpose(orientation);
      ang_velocity = world_inertia * ang_momentum;
      orientation += glm::matrixCross3(ang_velocity) * orientation * delta_time;
      orientation = glm::orthonormalize(orientation);

      // update translation matrix
      world_transform.SetRelPosition(lin_velocity * delta_time);
      hull.getWorldTransform().SetRelPosition(lin_velocity * delta_time);
      hull.updateCentroid(lin_velocity * delta_time);

      // SAME ISSUE AS BEFORE. IF MODEL ORIGIN NOT AT CENTRE OF MASS. ROTATION
      // FAILS.
      // world_transform.SetAbsRotation(glm::mat4(orientation));
      // hull.getWorldTransform().SetAbsRotation(glm::mat4(orientation));

      // update rotation matrix
      world_transform.SetRotationAbtPoint(glm::mat4(orientation),
                                          local_com_offset);
      hull.getWorldTransform().SetRotationAbtPoint(glm::mat4(orientation),
                                                   local_com_offset);

      // applyForces();
    }
  }

  void drag(glm::vec3 displacement) {

    // dragVelocity = 100.0f * displacement;
    //  update model transformations
    world_transform.SetRelPosition(displacement);
    hull.getWorldTransform().SetRelPosition(displacement);

    // update COM positiosn
    hull.updateCentroid(displacement);
    com += displacement;

    // track xyz offSets and deltaTime to accumulate velocity while dragging???
  }

  void rotate(float xOffset, float yOffset) {

    xOffset *= 0.01f;
    yOffset *= 0.01f;

    glm::mat4 rotX =
        glm::rotate(glm::mat4(1.0f), yOffset, glm::vec3(1.0f, 0.0f, 0.0f));
    glm::mat4 rotY =
        glm::rotate(glm::mat4(1.0f), xOffset, glm::vec3(0.0f, 1.0f, 0.0f));
    glm::mat4 rotation = rotY * rotX;

    world_transform.SetRotationAbtPoint(rotation, local_com_offset);
    hull.getWorldTransform().SetRotationAbtPoint(rotation, local_com_offset);

    // SAME ISSUE AS BEFORE. IF MODEL ORIGIN NOT AT CENTRE OF MASS. ROTATION
    // FAILS.
    // world_transform.SetRelRotation(rotation);
    // hull.getWorldTransform().SetRelRotation(rotation);
  }

  void disable() {

    is_static = true;

    lin_velocity = glm::vec3(0.0f);
  }

  void reset(glm::vec3 position) {

    world_transform.SetAbsPosition(position);

    // Shull.updateCentroid(position);
    hull.getWorldTransform().SetAbsPosition(position);

    com = position;
    lin_velocity = glm::vec3(0.0f);
  }

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

  void draw(Shader &shader) { rigidbody_model.Draw(shader); }

  WorldTransform &getWorldTransform() { return world_transform; }

  glm::vec3 get_centre_of_mass() { return com; }

  void printVec(glm::vec3 v) {
    std::cout << "vec3(" << v.x << " " << v.y << " " << v.z << ")\n";
  }

  void printDebug() {

    std::string i = glm::to_string(inertia);
    std::cout << "inertia: " << i << std::endl;
    std::cout << "com: ";
    printVec(com);
    std::cout << "omega: ";
    printVec(ang_velocity);
  }
};

#endif
