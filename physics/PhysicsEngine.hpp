#ifndef PHYSICSENGINE_HPP
#define PHYSICSENGINE_HPP

#include <glm/glm.hpp>

#include "../inc/Model.hpp"
#include "RigidBody.hpp"
//#include "ConvexHull.hpp"
#include "BoundingSphere.hpp"



class PhysicsEngine {
private:
	glm::vec3 gravity = glm::vec3(0.0, -0.001f, 0.0);
	double timeStep;

public:

	std::vector<RigidBody> rigidBodies;
	std::vector<BoundingSphere>colliders;


	void step(double timeStep) {

		glm::vec3 gravityAccel(gravity * timeStep);

		for (auto& body : rigidBodies) {

			//body.accelerateLinearly(gravityAccel);
			body.update(timeStep);
		}
	}

	void addRigidBody(RigidBody body) {
		rigidBodies.push_back(body);

	}

	void removeRigidBody(RigidBody body) {

	}

	void removeAllBodies() {
		rigidBodies.clear();
	}
};

#endif
