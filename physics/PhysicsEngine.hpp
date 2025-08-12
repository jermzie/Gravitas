#ifndef PHYSICSENGINE_HPP
#define PHYSICSENGINE_HPP

#include <glm/glm.hpp>

#include "../inc/Model.hpp"
#include "../inc/Plane.hpp"
#include "RigidBody.hpp"
//#include "ConvexHull.hpp"
#include "BoundingSphere.hpp"



class PhysicsEngine {
private:
	const glm::vec3 gravity = glm::vec3(0.0, -9.81, 0.0);

	Plane floor = Plane(glm::vec3(0.0, 1.0, 0.0), glm::vec3(0.0, -10.0, 0.0));
	Plane ceil = Plane(glm::vec3(0.0, -1.0, 0.0), glm::vec3(0.0, 10.0, 0.0));

	double timeStep;

public:

	std::vector<RigidBody> rigidBodies;
	std::vector<BoundingSphere>colliders;


	void step(double timeStep) {

		for (auto& body : rigidBodies) {


			glm::vec3 position = body.getCentreOfMass();

			//std::cout << "Object " << " Position: " << position.x << " " << position.y << " " << position.z << "\n";


			// check if body above/below floor/ceil; if not just translate y-axis 
			if (position.y <= -50.0) {

				body.setCentreOfMass(glm::vec3(position.x, 50.0f, position.z));
				WorldTransform& bodyTrans = body.getWorldTransform();
				bodyTrans.SetAbsolutePos(glm::vec3(position.x, 50.0f, position.z));

				WorldTransform& colliderTrans = body.collider.getWorldTransform();
				colliderTrans.SetAbsolutePos(glm::vec3(position.x, 50.0f, position.z));

				body.setLinearVelocity(glm::vec3(0.0f));
			}


			body.accelerateLinearly(gravity * static_cast<float>(timeStep));
			body.update(timeStep);
		}
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
