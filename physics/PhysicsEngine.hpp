#ifndef PHYSICSENGINE_HPP
#define PHYSICSENGINE_HPP

#include <glm/glm.hpp>

#include <algorithm>

#include "../inc/Model.hpp"
#include "../inc/Plane.hpp"
#include "RigidBody.hpp"
#include "ConvexHull.hpp"
#include "BoundingSphere.hpp"
#include "SAT.hpp"



class PhysicsEngine {
private:

	SAT narrowphase;

public:

	std::vector<RigidBody> bodies;
	std::vector<BoundingSphere>colliders;

	void step(double dt) {

		// 1. integrate forces
		/*for (auto& b : bodies) {
			if (b.isStatic) {

			}
			else {
				b.integrateForces(dt);
				b.update(dt);
			}
		}*/

		// 2. broadphase (BVH) -- find potential colliding pairs

		// 3. narrowphase (SAT) -- build contact list

		
		if (narrowphase.SATPolyPoly(bodies[0].hull, bodies[1].hull)){

			std::cout << "HIT OBJECT" << std::endl;
		}

		/*
		for (auto& b1 : bodies) {

			for (auto& b2 : bodies) {

				if (b1.id == b2.id) {
					continue;
				}

				
				else if(narrowphase.optimizedSAT(b1.hull, b2.hull)) {
					std::cout << "HIT BETWEEN OBJECT: " << b1.id << " & " << b2.id << std::endl;
				}
				
			}
		}
		*/
		

		// 4. collision solver (iterative)



		
		for (size_t i = 0; i < bodies.size(); i++) {

			
			glm::vec3 position = bodies[i].getCentreOfMass();

			// check if body above/below floor/ceil; if not just translate y-axis
			if (position.y <= -20.0f) {

				bodies[i].reset(glm::vec3(position.x, 20.0f, position.z));
			}

			//bodies[i].integrateForces(dt);
			bodies[i].update(dt);

		}
		



	}

	void addRigidBody(RigidBody&& body) {

		body.id = (int)bodies.size() + 1;
		bodies.push_back(std::move(body));
	}

	void removeRigidBody(RigidBody body) {


		
	}

	void removeAllBodies() {
		bodies.clear();
	}
};

#endif
