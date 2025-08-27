#ifndef BOUNDINGSPHERE_HPP
#define BOUNDINGSPHERE_HPP

#include <glm/glm.hpp>

#include "../inc/WorldTransform.hpp"
#include "../inc/Model.hpp"
#include "../inc/Camera.hpp"
#include "../inc/Ray.hpp"

const float PI = 3.14159265358979323846f;

class BoundingSphere {

private:

	Model sphere;
	WorldTransform worldTrans;

	glm::vec3 localCentroid;
	glm::vec3 worldCentroid;
	float radius;
	float scale;		// scale factor for sphere model



	glm::vec3 computeCentroid(const Model& model) {

		unsigned int numVertices = 0;
		glm::vec3 centroid(0.0f, 0.0f, 0.0f);

		for (auto& mesh : model.meshes) {
			for (auto& vertex : mesh.vertices) {

				centroid += vertex.Position;
				numVertices++;

			}
		}

		return centroid /= static_cast<float>(numVertices);
	}

public:


	BoundingSphere() {

		sphere.LoadModel("ball.obj");
	}

	// Init
	void computeBoundingSphere(Model model, WorldTransform objectTrans) {

		// find bounding sphere
		float maxDistSq = 0.0f;
		localCentroid = computeCentroid(model);

		for (auto& mesh : model.meshes) {
			for (auto& vertex : mesh.vertices) {

				float distSq = glm::dot((vertex.Position - localCentroid), (vertex.Position - localCentroid));

				maxDistSq = std::max(distSq, maxDistSq);

			}
		}

		radius = std::sqrt(maxDistSq);

		glm::vec4 objectWorldTrans = objectTrans.GetMatrix() * glm::vec4(localCentroid, 1.0f);
		worldCentroid = glm::vec3(objectWorldTrans);
		worldTrans.SetRelPosition(worldCentroid);


		// scale sphere model to correct dims. of bounding sphere 
		glm::vec3 modelOrigin = computeCentroid(sphere);

		// compute original radius R0
		float R0 = 0.0f;
		for (auto& mesh : sphere.meshes) {
			for (auto& vertex : mesh.vertices) {

				R0 = std::max(R0, glm::length(vertex.Position - modelOrigin));
			}
		}

		scale = radius / R0;
		worldTrans.SetScale(scale);
	}

	// Update
	

	bool computeRayIntersection(Ray& r, float &t) {

		// Equation of sphere w/ radius r, at some point P = (x, y, z), centered at C = (C_x, C_y, C_z):
		// In vector form:
		// (C - P) * (C - P) = r^2

		// ray originates from camera position
		// oc = (C - P)
		glm::vec3 oc = worldCentroid - r.origin;

		// a = d * d = |d|^2 (vector dotted w/ itself is vector length squared)
		float a = glm::dot(r.direction, r.direction);

		float b = glm::dot(r.direction, oc);

		// c =  |(C - P)|^2 - r^2
		float c = glm::length(oc) * glm::length(oc) - radius * radius;

		float discriminant = b * b - a * c;


		// miss 
		if (discriminant < 0) {
			return false;
		}

		float tMin = 0.0f;
		float tMax = 100000.0f;

		float sqrtd = std::sqrt(discriminant);


		// three cases -- two roots (discrim. > 0), one root (discrim. = 0), no solution (discrim. < 0)
		t = (b - sqrtd) / a;
		
		if (t <= tMin || t >= tMax)
		{
			t = (b + sqrtd) / a;
			if (t <= tMin || t >= tMax)
			{
				return false;
			}
		}


		// hit

		glm::vec3 hitPoint = r.origin + r.direction * t;
		r.t = t;
		//std::cout << "HITPOINT: " << hitPoint.x << " " << hitPoint.y << " " << hitPoint.z << "\n";
		return true;
	}

	void updateCentroid(glm::vec3 displacement) {


		// cannot do objectTrans.GetPosition() bc that returns the object COM, which is diff from the worldCentroid position
		// however, the transform matrix is the same, so we just apply the same translation vector to the worldCentroid

		/*
		PROBLEM:
		worldCentroid is not updating when object is moving


		*/

		//worldTrans = objectTrans;
		//worldTrans.SetScale(scale);

		// Apply the object's transformation to the centroid
		//glm::vec4 worldCentroidTrans = objectTrans.GetMatrix() * glm::vec4(localCentroid, 1.0f);
		worldCentroid += displacement;
		//worldTrans.SetPosition(worldCentroid);
	}


	// OUTDATED -- NEEDS UPDATES

	glm::vec3 GetCentroid() {
		return worldCentroid;
	}

	glm::vec3 GetScale() {
		
		return glm::vec3(scale, scale, scale);
	}

	WorldTransform& getWorldTransform() {
		return worldTrans;
	}

	void Draw(Shader& shader) {
		sphere.Draw(shader);
	}
};

#endif
