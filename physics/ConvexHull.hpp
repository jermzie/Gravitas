#ifndef CONVEXHULL_HPP
#define CONVEXHULL_HPP

#include <glm/glm.hpp>

#include <cmath>
#include <vector>
#include <algorithm>
#include <array>
#include <deque>

#include "../inc/Mesh.hpp"
#include "../inc/Model.hpp"
#include "../inc/Plane.hpp"
#include "HalfEdgeMesh.hpp"
/*
* 
Convex Hull w/ QuickHull Algo

High-level Steps:
1. Find Initial Tetrahedron																	-- setupInitialTetrahedron(); getExtremeValues();
2. Partition Points (assign points to each face if 'outside')								-- partitionPoints(); addPointToFace();
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


	float epsilon;
	float epsilonSquared;
	float scale;
	bool isPlanar;

	HalfEdgeMesh mesh;

	const std::vector<Vertex>vertices;
	const std::vector<glm::vec3>vertexData;			// Vertex data is unchanged


	std::array<size_t, 6>extremaIndices;
	std::vector < std::unique_ptr<std::vector<size_t>>> conflictListsPool;		// Assigned points pool

	std::vector<size_t>visibleFaces;				// All faces visibles from given point
	std::vector<size_t>horizonEdges;				// Loop of connected edges


	struct FaceData {
		size_t faceIndex;
		size_t enteredFromHalfEdge;					// Mark as horizon edge if face is not visible
	};

	std::vector<FaceData> possibleVisibleFaces;
	std::deque<size_t>faceStack;						// Face stack


	void setupInitialTetrahedron();
	std::array<size_t, 6> getExtrema();
	float getScale();
	bool addPointToFace(HalfEdgeMesh::Face& face, size_t pointIdx);


	void buildMesh(const std::vector<glm::vec3>& pointCloud);								// Constructs 
	void createConvexHalfEdgeMesh();														// Updates mesh; creates ConvexHull obj. that getConvexHull() returns
	void getConvexHull();							

	bool connectHorizonEdges();

	inline std::unique_ptr<std::vector<size_t>> getConflictList();							// Shifts ownership of "outside" set 
	inline void reclaimConflictList(std::unique_ptr < std::vector<size_t>>& ptr);			// When a face is disabled, we reclaim its assigned points for future usage


public:


	ConvexHull() = default;

};

#endif
