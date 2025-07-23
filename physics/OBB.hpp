#ifndef OBB_HPP
#define OBB_HPP

#include <glm/glm.hpp>

#include "../inc/Ray.hpp"
#include "../inc/Model.hpp"

class OBB {
private:

	glm::vec3 minAABB;
	glm::vec3 maxAABB;
	glm::vec3 centroid;

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

	// use Jacobi Eigenvalue algorithm -- core GLM doesn't support Eigen decomposition
	void computeEigenvectors(glm::mat4 mat) {

	}



public:

	OBB() = default;
	OBB() {

	}
	bool ComputeOBB(Model model) {

		// 1. find the centre of the OBB
		centroid = computeCentroid(model);


		// 2. find the covariance
		glm::mat3 cov(0.0f);


		// 3. eigen decomposition

		glm::mat3 evec(0.0f);



		// 4. project vertices along eigenvectors
		glm::vec3 minProj


		// 5. determine extents



	}

	bool RayOBBIntersection(const Ray& r, float &t) {

	}

	bool OBBCollisionDetection();
};

#endif
