#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/scalar_multiplication.hpp>
#include <glm/gtx/matrix_cross_product.hpp>

#include <vector>

#include "../inc/Model.hpp"
#include "../inc/Mesh.hpp"
#include "../inc/WorldTransform.hpp"
#include "../inc/Ray.hpp"
#include "ConvexHull.hpp"
#include "QuickHull.hpp"
#include "BoundingSphere.hpp"

class RigidBody {
private:
	std::vector<Vertex>mesh;		// Polyhedron triangle mesh

	WorldTransform worldTrans;
	Model rigidBodyModel;

	glm::vec3 centreOfMass;
	glm::vec3 linearVelocity;

	glm::mat4 orientation = glm::mat4(1.0);
	glm::vec3 angularVelocity;


	glm::vec3 acceleration = glm::vec3(0.0, -0.005, 0.0);



	double mass;
	double inverseMass;
	double friction;


	bool isDragging = false;


	glm::mat3 inertiaTensor;

public:


	BoundingSphere collider;
	ConvexHull hull;

	RigidBody(Model model, double mass, glm::vec3 position, glm::vec3 velocity, glm::vec3 omega){

		// physics properties
		this->rigidBodyModel = model;
		this->mass = mass;
		this->centreOfMass = position;
		this->linearVelocity = velocity;
		this->angularVelocity = omega;

		// set inital transformation matrices
		worldTrans.SetPosition(position);

		QuickHull qh;
		hull = qh.getConvexHull(model.GetVertexData());
		hull.computeConvexHull(model, worldTrans);

		// sphere centroid position
		collider.computeBoundingSphere(model, worldTrans);

	}


	void accelerateLinearly(const glm::vec3 deltaVelocity) {

		linearVelocity += deltaVelocity;
	}

	void update(double deltaTime) {

		if (isDragging) {


		}
		else {

			centreOfMass += linearVelocity * deltaTime;

			//orientation += glm::matrixCross4(angularVelocity) * orientation * deltaTime;


			worldTrans.SetPosition(centreOfMass);
			//worldMat4.SetRotate(orientation);



			// move collider centroid position alongside rigid body position
			//collider.UpdateCentroid(worldTrans);

		}
	}

	void Drag(glm::vec3 displacement) {

		// move rigidbody to newPos, following ray 

		centreOfMass += displacement;

		// track xyz offSets and deltaTime to accumulate velocity while dragging???
	}

	void Disable() {


	}


	WorldTransform& getWorldTransform() { 
		return worldTrans; 
	}

	void setCentreOfMass(glm::vec3 position) {

		centreOfMass = position;

	}

	glm::vec3 getCentreOfMass() {
		return centreOfMass;
	}

	void Draw(Shader& shader) {
		rigidBodyModel.Draw(shader);
	}

	bool collided(Ray& worldRay, glm::vec3& hitPoint) {

		glm::mat4 inverseModelMatrix = glm::inverse(worldTrans.GetMatrix());
		glm::vec4 rayOrig_local = inverseModelMatrix * glm::vec4(worldRay.origin, 1.0f);
		glm::vec4 rayDir_local = inverseModelMatrix * glm::vec4(worldRay.direction, 0.0f);

		Ray localRay;
		localRay.origin = glm::vec3(rayOrig_local);
		localRay.direction = glm::vec3(rayDir_local);
		
		// t in local space
		float t;
		int res = hull.computeRayIntersection(localRay, t);

		glm::vec3 hitLocal = localRay.origin + localRay.direction * t;
		glm::vec4 hitWorld4 = hull.getWorldTransform().GetMatrix() * glm::vec4(hitLocal, 1.0f);
		hitPoint = glm::vec3(hitWorld4);

		return res;
	}

	// BoundingSphere getBounds() {
	// 	//return collider;
	// }

};

#endif
