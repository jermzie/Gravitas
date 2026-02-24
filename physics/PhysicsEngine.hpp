#pragma once

#include <glm/glm.hpp>

#include <algorithm>

#include "AABB.hpp"
#include "Broadphase.hpp"
#include "CollisionGeometry.hpp"
#include "Config.hpp"
#include "Debugger.hpp"
#include "Model.hpp"
#include "Plane.hpp"
#include "RigidBody.hpp"
// DOESNT WORK
// #include "BoundingSphere.hpp"
#include "Narrowphase.hpp"

class PhysicsEngine {
private:
  Debugger *debug = nullptr;
  Narrowphase sat;
  Broadphase bvh;

public:
  void set_debugger(Debugger *debug) { this->debug = debug; }

  std::vector<RigidBody> bodies;
  std::vector<std::pair<int, int>> potential_collisions;

  void step(float dt) {
    glm::vec3 n = glm::vec3(0.0f, 1.0f, 0.0f);
    glm::vec3 p = glm::vec3(0.0f);
    Plane ground(n, p);
    if (debug) {
      debug->draw_plane(ground, 50, glm::vec3(1, 1, 1));
    }

    // std::cerr << "Entered collision function\n";
    //  1. integrate forces
    for (auto &b : bodies) {
      if (b.is_static) {

      } else {
        b.integrate_forces(dt);
        b.update(dt);
      }
    }

    // 2.0. check if colliding with ground
    for (auto &body : bodies) {
      if (sat.poly_plane_collision(body, ground)) {

        Narrowphase::ContactManifold cp = sat.create_plane_contact(body, ground);
        if (cp.num_points == 0)
          continue;

        // Position correction: high Baumgarte so penetration resolves in 1-2 frames.
        // A low factor lets gravity win -- body sinks faster than correction applies.
        const float baumgarte_factor = 0.8f;
        const float slop = 0.005f;
        float correction = std::max(cp.max_penetration - slop, 0.0f) * baumgarte_factor;
        body.position += correction * ground.normal;
        body.update_transform(); // sync matrix used by rendering & collision

        // Velocity correction -- THE main sink fix.
        // Gravity adds downward velocity every frame. If we don't remove the
        // normal component here, the body re-penetrates next frame and Baumgarte
        // can never keep up, causing the slow sink.
        float v_normal = glm::dot(body.lin_velocity, ground.normal);
        if (v_normal < 0.0f) {
          const float restitution = 0.3f;
          // Treat near-zero approach speeds as resting contact: zero the normal
          // velocity instead of reflecting it, preventing micro-bounce jitter.
          const float rest_threshold = 0.05f; // tune alongside gravity magnitude
          float e = (std::abs(v_normal) < rest_threshold) ? 0.0f : restitution;
          body.lin_velocity -= (1.0f + e) * v_normal * ground.normal;
        }
      }
    }

    // 2. broadphase (BVH) -- find potential colliding pairs
    potential_collisions = bvh.update(bodies, NULL);

    for (auto pair : potential_collisions) {
      if (sat.poly_poly_collision(bodies[pair.first], bodies[pair.second], debug)) {

        CLOGI("HIT OBJECT");
      }
    }
    /*
    for (auto pair : potential_collisions) {

      if (narrowphase.SATPolyPoly(bodies[pair.first].hull,
                                  bodies[pair.second].hull)) {
        printf("HIT OBJECT\n");
      }
    }
    */

    // 3. narrowphase (SAT) -- build contact list
    /*
        for (auto &b1 : bodies) {

          for (auto &b2 : bodies) {

            if (b1.id == b2.id) {
              continue;
            }

            else if (sat.poly_poly_collision(b1, b2, debug)) {
              CLOGI("HIT OBJECT");
            }
          }
        }
        */

    // 4. collision solver (iterative)

    for (size_t i = 0; i < bodies.size(); i++) {

      glm::vec3 position = bodies[i].get_centre_of_mass();

      // check if body above/below floor/ceil; if not just translate y-axis
      // if (position.y <= -20.0f) {

      // bodies[i].reset(glm::vec3(position.x, 20.0f, position.z));
      //}

      // bodies[i].integrateForces(dt);
      bodies[i].update(dt);
    }
  }

  bool raycast(const Ray &ray, int &id, float &t) { return bvh.raycast(bodies, ray, id, t); }

  void add_rigid_body(RigidBody &&body) {
    body.id = (int)bodies.size();
    bodies.push_back(std::move(body));
    bvh.insert(body.collider.local_aabb, body.id);
    CLOGI("ADDED BODY %d", body.id);
  }

  void remove_rigid_body(RigidBody body) {

    // bodies.erase();s
  }

  void remove_all_bodies() { bodies.clear(); }
};
