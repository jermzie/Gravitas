#ifndef PHYSICSENGINE_HPP
#define PHYSICSENGINE_HPP

#include <glm/glm.hpp>

#include <algorithm>

#include "../inc/Model.hpp"
#include "../inc/Plane.hpp"
#include "RigidBody.hpp"
#include "ConvexHull.hpp"
#include "BoundingSphere.hpp"
//#include "SAT.hpp"



class PhysicsEngine {
private:

	//SAT collisionDetection;

public:

	std::vector<RigidBody> bodies;
	std::vector<BoundingSphere>colliders;

	void step(double dt) {

		for (size_t i = 0; i < bodies.size(); i++) {

			glm::vec3 position = bodies[i].getCentreOfMass();

			// check if body above/below floor/ceil; if not just translate y-axis 
			if (position.y <= -10.0f) {

				bodies[i].reset(glm::vec3(position.x, 10.0f, position.z));
			}

			bodies[i].update(dt);

			/*for (size_t j = 0; j < bodies.size(); j++) {

				if (i == j) continue;

				std::cout << collisionDetection.optimizedSAT(bodies[i].hull, bodies[j].hull) << std::endl;
				
			}*/
		}
	}

	void addRigidBody(RigidBody&& body) {
		bodies.push_back(std::move(body));

	}

	void removeRigidBody(RigidBody body) {
		
	}

	void removeAllBodies() {
		bodies.clear();
	}
};

#endif
