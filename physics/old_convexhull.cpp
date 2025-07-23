#include "ConvexHull.h"

#pragma once

#include "../inc/mesh.h"
#include "../inc/model.h"

#include <vector>
#include <glm/glm.hpp>

/*

WRITE THIS SHIT FROM SCRATCH FIRST (DRAFT)
YOUR OWN IMPLEMENTATION

OPTIMIZE LATER

Convex Hull w/ QuickHull Algo

High-level Steps:
1. Find Initial Tetrahedron
2. Partition Points (assign points to each face if 'outside')
3a. Select face w/ nonempty 'outside' set
3b. Pick point farthest from selected face
3c. Determine set of all faces visible from point (dot-product & dfs)
3d. Compute horizon (boundary loop of edges visible from point)
3e. Delete set of all visible faces
3f. Form new triangular faces, connecting point to horizon edges
3g. Reassign Points (assign points from deleted faces to new faces)
4.	Repeat step 3. until all 'outside' sets are empty

*/

class ConvexHull {
private:

	Model* model;
	//std::vector<Face>faces;
	std::array<size_t, 6>extremeValues;
	std::vector<glm::vec3>vertexData;

	std::array<size_t, 6> getExtrema() {

		std::array<size_t, 6> extremaIndices{};
		std::array<float, 6> extrema{ vertexData[0].x, vertexData[0].x , vertexData[0].y, vertexData[0].y, vertexData[0].z, vertexData[0].z };

		const size_t vCount = vertexData.size();
		for (size_t i = 1; < i < vCount; ++i) {

			const glm::vec3& pos = vertexData[i];

			// x-axis min/max
			if (pos.x < extrema[0]) {
				extrema[0] = pos.x;
				extremaIndices[0] = i;

			}
			else if (pos.x > extrema[1]) {
				extrema[1] = pos.x;
				extremaIndices[1] = i;
			}

			// y-axis min/max
			if (pos.y < extrema[2]) {
				extrema[2] = pos.y;
				extremaIndices[2] = i;
			}
			else if (pos.y > extrema[3]) {
				extrema[3] = pos.y;
				extremaIndices[3] = i;
			}

			// z-axis min/max
			if (pos.z < extrema[4]) {
				extrema[4] = pos.z;
				extremaIndices[4] = i;
			}
			else if (pos.z > extrema[5]) {
				extrema[5] = pos.z;
				extremaIndices[5] = i;
			}
		}

		return extremaIndices;
	}

	float GetScale(const std::array<size_t, 6>& extrema) {

		float scale = 0.0f;
		for (size_t i = 0; i < 6; ++i) {

			const float* val = (const float*)(&vertexData[extrema[i]]);

		}
	}


	void SetupInitialTetraHedron() {
		const size_t vCount = vertexData.size();


		if (vCount <= 4) {

		}

		// 1. find the two extrema points furthest apart
		for (size_t i = 0; i < 6; ++i) {

		}

		// 2. find the point furthest from the line formed between chosen extrema points -- forms triangle
		for (size_t i = 0; i < vCount; ++i) {

		}

		// 3. find the point furthest from the triangle plane -- forms tetrahedron



		// 4. create half-edge mesh & compute planes defined by each triangle face
	}


public:

	ConvexHull() = default;
	ConvexHull(const Model& model) : model(model) {

	}

	void BuildConvexHull(const Model& model) {

	}



};