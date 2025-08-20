#ifndef PHYSICSENGINE_HPP
#define PHYSICSENGINE_HPP

#include <glm/glm.hpp>

#include "../inc/Model.hpp"
#include "../inc/Plane.hpp"
#include "RigidBody.hpp"
//#include "ConvexHull.hpp"
#include "BoundingSphere.hpp"
#include "SAT.hpp"



class PhysicsEngine {
private:

	double timeStep;
	SAT collisionDetection;

public:

	std::vector<RigidBody> rigidBodies;
	std::vector<BoundingSphere>colliders;

	bool collides = false;

	void step(double timeStep) {

		for (auto& body : rigidBodies) {


			glm::vec3 position = body.getCentreOfMass();

			//std::cout << "Object " << " Position: " << position.x << " " << position.y << " " << position.z << "\n";


			// check if body above/below floor/ceil; if not just translate y-axis 
			if (position.y <= -10.0f) {

				body.reset(glm::vec3(position.x, 10.0f, position.z));
			}

			body.update(timeStep);

			for (auto& body2 : rigidBodies) {
				collides = collisionDetection.optimizedSAT(body.hull, body2.hull);
			}
		}

		std::cout << collides << std::endl;

		
	}

	void addRigidBody(RigidBody&& body) {
		rigidBodies.push_back(std::move(body));

	}

	void removeRigidBody(RigidBody body) {

	}

	void removeAllBodies() {
		rigidBodies.clear();
	}
};

#endif
