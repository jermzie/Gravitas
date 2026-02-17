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

    // std::cerr << "Entered collision function\n";
    //  1. integrate forces
    /*for (auto& b : bodies) {
            if (b.isStatic) {

            }
            else {
                    b.integrateForces(dt);
                    b.update(dt);
            }
    }*/

    // 2. broadphase (BVH) -- find potential colliding pairs
    potential_collisions = bvh.update(bodies, debug);
    CLOGI("NUM COLLISIONS %ld", potential_collisions.size());

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
