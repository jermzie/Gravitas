#pragma once

#include <glm/glm.hpp>

#include <algorithm>

#include "../inc/Model.hpp"
#include "../inc/Plane.hpp"
#include "CollisionGeometry.hpp"
#include "RigidBody.hpp"

// DOESNT WORK
// #include "Broadphase.hpp"
// DOESNT WORK
// #include "AABB.hpp"
// DOESNT WORK
// #include "BoundingSphere.hpp"
#include "Narrowphase.hpp"

class PhysicsEngine {
private:
  Narrowphase sat;
  // BVH broadphase;

public:
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
    // DOESNT WORK
    // potential_collisions = broadphase.query_pairs();

    // DOESNT WORK
    /*
    for (auto pair : potential_collisions) {

      if (narrowphase.SATPolyPoly(bodies[pair.first].hull,
                                  bodies[pair.second].hull)) {
        printf("HIT OBJECT\n");
      }
    }
    */

    // 3. narrowphase (SAT) -- build contact list

    for (auto &b1 : bodies) {

      for (auto &b2 : bodies) {

        if (b1.id == b2.id) {
          continue;
        }

        else if (sat.poly_poly_collision(b1, b2)) {
          std::cout << "HIT OBJECT" << std::endl;
        }
      }
    }

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

  void add_rigid_body(RigidBody &&body) {

    body.id = (int)bodies.size() + 1;
    bodies.push_back(std::move(body));
    // broadphase.insert();
  }

  void remove_rigid_body(RigidBody body) {

    // bodies.erase();s
  }

  void remove_all_bodies() { bodies.clear(); }
};
