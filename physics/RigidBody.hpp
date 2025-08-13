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

	glm::mat3 orientation = glm::mat3(1.0);
	glm::vec3 angularVelocity;


	glm::vec3 acceleration = glm::vec3(0.0, -0.005, 0.0);



	float mass;
	float density;
	float inverseMass;
	float friction;


	bool isDragging = false;


	glm::mat3 inertiaTensor = glm::mat3(1.0);

public:


	BoundingSphere sphere;
	ConvexHull hull;

	RigidBody(Model model, float rho, glm::vec3 position, glm::vec3 velocity, glm::vec3 omega){

		// basic physics properties
		rigidBodyModel = model;
		density = rho;
		linearVelocity = velocity;
		angularVelocity = omega;

		QuickHull qh;
		hull = qh.getConvexHull(model.GetVertexData());

		// sphere centroid position
		// sphere.computeBoundingSphere(model, worldTrans);

		// com
		hull.computeMassProperties(density, mass, centreOfMass, inertiaTensor);

		// set inital transformation matrices
		worldTrans.SetAbsolutePos(centreOfMass);
		worldTrans.SetPosition(position);

		hull.computeConvexHull(model, worldTrans);
	}

	void applyForces() {

	}

	void update(double deltaTime) {

		if (isDragging) {

			std::cout << "Hello?\n";


		}
		else {

			centreOfMass += linearVelocity * deltaTime;

			//orientation += glm::matrixCross4(angularVelocity) * orientation * deltaTime;


			worldTrans.SetPosition(linearVelocity * deltaTime);
			hull.getWorldTransform().SetPosition(linearVelocity * deltaTime);

			applyForces();
			//worldMat4.SetRotate(orientation);



			// move collider centroid position alongside rigid body position
			//collider.UpdateCentroid(worldTrans);

		}
	}

	void drag(glm::vec3 displacement) {

		// update model transformations
		worldTrans.SetPosition(displacement);
		WorldTransform& hullTrans = hull.getWorldTransform();
		hullTrans.SetPosition(displacement);


		// update COM positiosn
		hull.updateCentroid(displacement);
		centreOfMass += displacement;

		// track xyz offSets and deltaTime to accumulate velocity while dragging???
	}

	void disable() {

		isDragging = true;

		linearVelocity = glm::vec3(0.0f);

	}

	void reset() {


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

	void draw(Shader& shader) {
		rigidBodyModel.Draw(shader);
	}

	WorldTransform& getWorldTransform() {
		return worldTrans;
	}

	glm::vec3 getCentreOfMass() {
		return centreOfMass;
	}z

};

#endif
