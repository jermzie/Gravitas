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

	double timeStep;

public:

	std::vector<RigidBody> rigidBodies;
	std::vector<BoundingSphere>colliders;


	void step(double timeStep) {

		for (auto& body : rigidBodies) {


			glm::vec3 position = body.getCentreOfMass();

			//std::cout << "Object " << " Position: " << position.x << " " << position.y << " " << position.z << "\n";


			// check if body above/below floor/ceil; if not just translate y-axis 
			if (position.y <= -10.0) {

				//body.setCentreOfMass(glm::vec3(position.x, 50.0f, position.z));
				WorldTransform& bodyTrans = body.getWorldTransform();
				bodyTrans.SetAbsolutePos(glm::vec3(position.x, 10.0f, position.z));

				WorldTransform& hullTrans = body.hull.getWorldTransform();
				hullTrans.SetAbsolutePos(glm::vec3(position.x, 10.0f, position.z));

				body.reset(glm::vec3(position.x, 10.0f, position.z));
			}

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
