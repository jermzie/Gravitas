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

class ConvexHull
{
private:
	//
	std::unique_ptr<std::vector<glm::vec3>> optimizedVBO;
	std::vector<Vertex> vertices;
	std::vector<int> indices;

	float epsilon, epsilonSquared, scale;
	bool isPlanar;

	HalfEdgeMesh mesh;

	const std::vector<glm::vec3> vertexData;

	std::array<size_t, 6> extremaIndices;
	std::vector<std::unique_ptr<std::vector<size_t>>> conflictListsPool; // Assigned points pool

	std::vector<size_t> visibleFaces; // All faces visibles from given point
	std::vector<size_t> horizonEdges; // Loop of connected edges

	std::vector<size_t> newFaces;
	std::vector<size_t> newHalfEdges;
	std::vector<std::unique_ptr<std::vector<size_t>>> disabledFaceConflictLists;

	struct FaceData
	{
		size_t faceIndex;
		size_t enteredFromHalfEdge; // Mark as horizon edge if face is not visible
	};

	std::vector<FaceData> possibleVisibleFaces;
	std::deque<size_t> faceStack;

	void buildMesh(const std::vector<glm::vec3> &pointCloud, float defaultEps);
	void createConvexHalfEdgeMesh();

	void setupInitialTetrahedron();

	void getConvexHull(const std::vector<glm::vec3> &pointCloud, bool CCW, bool useOriginalIndices);

	std::array<size_t, 6> getExtrema();

	float getScale();

	bool connectHorizonEdges(std::vector<size_t> &horizonEdges);

	bool addPointToFace(HalfEdgeMesh::Face &face, size_t pointIdx);

	inline std::unique_ptr<std::vector<size_t>> getConflictList();

	inline void reclaimConflictList(std::unique_ptr<std::vector<size_t>> &ptr);

public:
	ConvexHull() = default;

	//
	std::vector<Vertex> &getVertices();
	std::vector<int> &getIndices();
	void writeOBJ(const std::string &fileName, const std::string &objectName = "quickhull") const;
	// void getConvexHullAsMesh();
};

#endif