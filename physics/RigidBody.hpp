#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/scalar_multiplication.hpp>
#include <glm/gtx/matrix_cross_product.hpp>
#include <glm/gtx/string_cast.hpp>
#include <glm/gtx/orthonormalize.hpp>

#include <vector>

#include "../inc/Model.hpp"
#include "../inc/Mesh.hpp"
#include "../inc/WorldTransform.hpp"
#include "../inc/Ray.hpp"
#include "ConvexHull.hpp"
#include "QuickHull.hpp"
#include "BoundingSphere.hpp"
#include "AABB.hpp"

class RigidBody {
private:
	std::vector<ConvexHull>body;		// collection of convex hulls 

	WorldTransform bodyTrans;
	Model bodyModel;
	glm::vec3 localCOMOffset;

	// linear motion
	glm::vec3 centreOfMass;
	glm::vec3 linearVelocity;
	glm::vec3 linearMomentum;


	// angular motion
	glm::mat3 orientation = glm::mat3(1.0);
	glm::vec3 angularVelocity;
	glm::vec3 angularMomentum;
	glm::mat3 inertia = glm::mat3(1.0);
	glm::mat3 invInertia = glm::mat3(1.0);

	// mass properties
	float mass;
	float density;
	float invMass;
	float friction;

public:

	int id;
	bool isStatic = false;

	BoundingSphere sphere;
	AABB node;
	ConvexHull hull;

	// body created by singular model -- often imported models
	RigidBody(Model model, float rho, glm::vec3 position, glm::vec3 velocity, glm::vec3 L){

		// basic physics properties
		bodyModel = model;
		density = rho;
		linearVelocity = velocity;
		angularMomentum = L;


		// compute convex hull & half-edge mesh of model
		QuickHull qh;
		hull = qh.getConvexHull(bodyModel.GetVertexData());
		auto extrema = qh.getExtremaVertices();

		// local com & inertia
		hull.computeMassProperties(density, mass, localCOMOffset, inertia);
		invInertia = glm::inverse(inertia);
		centreOfMass = position + localCOMOffset;
		
		// transformations
		bodyTrans.SetAbsPosition(position);
		hull.getWorldTransform().SetAbsPosition(position);
		hull.worldCentroid += position;

		// model
		hull.getHullModel(bodyModel, bodyTrans);
	}

	/*
	// body created by collection of convex meshes -- manually defined
	RigidBody(std::vector<ConvexHull>models, float rho, glm::vec3 position, glm::vec3 velocity, glm::vec3 L) {

		// basic physics properties
		bodyModel = models;
		density = rho;
		linearVelocity = velocity;
		angularMomentum = L;

	}
	*/


	// apply force & torque 
	void integrateForces(double dt) {

	}

	void update(double deltaTime) {

		if (isStatic) {

			// DO SOMETHING

		}
		else {

			// linear motion
			//linearVelocity += gravity * deltaTime;
			centreOfMass += linearVelocity * deltaTime;

			// angular motion
			// omega;
			glm::mat3 worldInertia = orientation * invInertia * glm::transpose(orientation);
			angularVelocity = worldInertia * angularMomentum;
			orientation += glm::matrixCross3(angularVelocity) * orientation * deltaTime;

			orientation = glm::orthonormalize(orientation);

			// update translation matrix
			bodyTrans.SetRelPosition(linearVelocity * deltaTime);
			hull.getWorldTransform().SetRelPosition(linearVelocity * deltaTime);
			hull.updateCentroid(linearVelocity * deltaTime);

			
			// SAME ISSUE AS BEFORE. IF MODEL ORIGIN NOT AT CENTRE OF MASS. ROTATION FAILS.
			//bodyTrans.SetAbsRotation(glm::mat4(orientation));
			//hull.getWorldTransform().SetAbsRotation(glm::mat4(orientation));

			// update rotation matrix
			bodyTrans.SetRotationAbtPoint(glm::mat4(orientation), localCOMOffset);
			hull.getWorldTransform().SetRotationAbtPoint(glm::mat4(orientation), localCOMOffset);
	

			//applyForces();


		}
	}

	void drag(glm::vec3 displacement) {

		// update model transformations
		bodyTrans.SetRelPosition(displacement);
		hull.getWorldTransform().SetRelPosition(displacement);


		// update COM positiosn
		hull.updateCentroid(displacement);
		centreOfMass += displacement;

		// track xyz offSets and deltaTime to accumulate velocity while dragging???
	}

	void rotate(float xOffset, float yOffset) {

		xOffset *= 0.01f;
		yOffset *= 0.01f;

		glm::mat4 rotX = glm::rotate(glm::mat4(1.0f), yOffset, glm::vec3(1.0f, 0.0f, 0.0f));
		glm::mat4 rotY = glm::rotate(glm::mat4(1.0f), xOffset, glm::vec3(0.0f, 1.0f, 0.0f));
		glm::mat4 rotation = rotY * rotX;


		bodyTrans.SetRotationAbtPoint(rotation, localCOMOffset);
		hull.getWorldTransform().SetRotationAbtPoint(rotation, localCOMOffset);

		// SAME ISSUE AS BEFORE. IF MODEL ORIGIN NOT AT CENTRE OF MASS. ROTATION FAILS.
		//bodyTrans.SetRelRotation(rotation);
		//hull.getWorldTransform().SetRelRotation(rotation);
	}

	void disable() {

		isStatic = true;

		linearVelocity = glm::vec3(0.0f);

	}

	void reset(glm::vec3 position) {

		bodyTrans.SetAbsPosition(position);

		//Shull.updateCentroid(position);
		hull.getWorldTransform().SetAbsPosition(position);


		centreOfMass = position;
		linearVelocity = glm::vec3(0.0f);

		
	}

	bool collided(Ray& worldRay, glm::vec3& hitPoint) {

		glm::mat4 inverseModelMatrix = glm::inverse(bodyTrans.GetMatrix());
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

	void draw(Shader& shader) {
		bodyModel.Draw(shader);
	}

	WorldTransform& getWorldTransform() {
		return bodyTrans;
	}

	glm::vec3 getCentreOfMass() {
		return centreOfMass;
	}

	void printVec(glm::vec3 v) {
		std::cout << "vec3(" << v.x << " " << v.y << " " << v.z << ")\n";
	}

	void printDebug() {
		

		std::string i = glm::to_string(inertia);
		std::cout << "inertia: " << i << std::endl;
		std::cout << "com: ";  printVec(centreOfMass);
		std::cout << "omega: ";  printVec(angularVelocity);

	}
};

#endif
