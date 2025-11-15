#ifndef QUICKHULL_HPP
#define QUICKHULL_HPP

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#ifndef PROJECT_SOURCE_DIR
#error "PROJECT_SOURCE_DIR is not defined"
#endif


#include <iostream>
#include <string>
#include <regex>
#include <memory>
#include <cassert>
#include <cmath>
#include <vector>
#include <algorithm>
#include <array>
#include <deque>
#include <unordered_map>
#include <set>
#include <filesystem>

#include "../inc/Shader.hpp"
#include "../inc/Mesh.hpp"
#include "../inc/Model.hpp"
#include "../inc/Plane.hpp"
#include "../inc/Ray.hpp"
#include "../inc/WorldTransform.hpp"
#include "MeshBuilder.hpp"
#include "HalfEdgeMesh.hpp"
#include "ConvexHull.hpp"
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

class QuickHull
{
private:

	float epsilon, epsilonSquared, scale;
	bool isPlanar;

	MeshBuilder mesh;

	std::vector<glm::vec3> vertexData;
	std::array<size_t, 6> extremaIndices;
	std::vector<glm::vec3> tempPlanarVertices;
	std::vector<std::unique_ptr<std::vector<size_t>>> conflictListsPool;

	// Temporary variables used during iteration process
	std::vector<size_t> newFaces;
	std::vector<size_t> newHalfEdges;
	std::vector<std::unique_ptr<std::vector<size_t>>> disabledFaceConflictLists;
	std::vector<size_t> visibleFaces;
	std::vector<size_t> horizonEdges;

	struct FaceData {
		size_t faceIdx;
		size_t enteredFromHalfEdge; // Mark as horizon edge if face is not visible
		
		FaceData() = default;
		FaceData(size_t face, size_t he) : faceIdx(face), enteredFromHalfEdge(he) {}
	};

	std::vector<FaceData> possibleVisibleFaces;
	std::deque<size_t> faceStack;

	void buildMesh(const std::vector<glm::vec3> &pointCloud, float defaultEps = 0.0001f);

	void setupInitialTetrahedron();

	void createConvexHalfEdgeMesh();

	bool connectHorizonEdges(std::vector<size_t>& horizonEdges);

	std::array<size_t, 6> getExtrema();

	float getScale();

	bool addPointToFace(MeshBuilder::Face &face, size_t pointIdx);

	inline std::unique_ptr<std::vector<size_t>> getConflictList();

	inline void reclaimConflictList(std::unique_ptr<std::vector<size_t>> &ptr);

	void debugState() const;

public:

	QuickHull() = default;

	ConvexHull getConvexHull(const std::vector<glm::vec3>& pointCloud, bool CCW = true, bool useOriginalIndices = false, float epsilon = 0.0001f);

	HalfEdgeMesh getHalfEdgeMesh(const std::vector<glm::vec3>& pointCloud, bool CCW = true, float epsilon = 0.0001f);

	std::array<float, 6> getExtremaVertices();


};

#endif
