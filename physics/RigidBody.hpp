#ifndef RIGIDBODY_HPP
#define RIGIDBODY_HPP

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/scalar_multiplication.hpp>
#include <glm/gtx/matrix_cross_product.hpp>

#include <vector>

#include "../quickhull/QuickHull.hpp"
#include "../inc/Model.hpp"
#include "../inc/Mesh.hpp"
#include "../inc/WorldTransform.hpp"
#include "../inc/Ray.hpp"
#include "ConvexHull.hpp"
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


	bool fixed = false;


	glm::mat3 inertiaTensor;

public:


	BoundingSphere collider;
	//quickhull::QuickHull<float>qh;
	ConvexHull hull;

	RigidBody(Model model, double mass, glm::vec3 position, glm::vec3 velocity, glm::vec3 omega){

		// physics properties
		this->rigidBodyModel = model;
		this->mass = mass;
		this->centreOfMass = position;
		this->linearVelocity = velocity;
		this->angularVelocity = omega;

		// set inital transformation matrices
		worldTrans.SetPosition(centreOfMass);


		// sphere centroid position
		collider.computeBoundingSphere(model, worldTrans);
	
		hull.computeConvexHull(model, worldTrans);

		/*
		std::vector<glm::vec3>pointCloud = model.GetVertexData();
		auto hull = qh.getConvexHull(&pointCloud[0].x, pointCloud.size(), true, false);

		std::string objectName = model.fileName;
		size_t dotPos = objectName.find_last_of(".");
		std::string name = objectName.substr(0, dotPos);
		std::string ext = objectName.substr(dotPos);
		std::string convexhullName = name + "_quickhull" + ext;

		hull.writeWaveformOBJ(convexhullName);
		*/
	}


	void accelerateLinearly(const glm::vec3 deltaVelocity) {

		linearVelocity = linearVelocity + deltaVelocity;
	}

	void update(double deltaTime) {


		

		/*
		centreOfMass += linearVelocity * deltaTime;

		orientation += glm::matrixCross4(angularVelocity) * orientation * deltaTime;
	

		worldMat4.SetPosition(centreOfMass);
		worldMat4.SetRotate(orientation);
		*/


		// move collider centroid position alongside rigid body position
		//collider.UpdateCentroid(worldTrans);

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

	bool collided(Ray& r, float &t) {

		return collider.computeRayIntersection(r, t);
	}

	// BoundingSphere getBounds() {
	// 	//return collider;
	// }

};

#endif
