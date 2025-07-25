#ifndef CONVEXHULL_HPP
#define CONVEXHULL_HPP

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#ifndef PROJECT_SOURCE_DIR
#error "PROJECT_SOURCE_DIR is not defined"
#endif

#include <string>
#include <memory>
#include <cassert>
#include <cmath>
#include <vector>
#include <algorithm>
#include <array>
#include <deque>
#include <unordered_map>

#include "../inc/Shader.hpp"
#include "../inc/Mesh.hpp"
#include "../inc/Model.hpp"
#include "../inc/Plane.hpp"
#include "../inc/Ray.hpp"
#include "../inc/WorldTransform.hpp"
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
	// rendering stuffs
	std::unique_ptr<std::vector<glm::vec3>> optimizedVBO;
	std::vector<glm::vec3> vertices;
	std::vector<int> indices;

	Model convexhullModel;
	std::string modelName;

	WorldTransform worldTrans;
	glm::vec3 localCentroid;
	glm::vec3 worldCentroid;

	// algo stuffs
	float epsilon, epsilonSquared, scale;
	bool isPlanar;

	HalfEdgeMesh mesh;

	std::vector<glm::vec3> vertexData;

	std::array<size_t, 6> extremaIndices;
	std::vector<std::unique_ptr<std::vector<size_t>>> conflictListsPool; // Assigned points pool

	std::vector<size_t> visibleFaces; // All faces visibles from given point
	std::vector<size_t> horizonEdges; // Loop of connected edges

	std::vector<size_t> newFaces;
	std::vector<size_t> newHalfEdges;
	std::vector<std::unique_ptr<std::vector<size_t>>> disabledFaceConflictLists;

	std::vector<glm::vec3> tempPlanarVertices;
	struct FaceData
	{
		size_t faceIndex;
		size_t enteredFromHalfEdge; // Mark as horizon edge if face is not visible
		
		FaceData() = default;
		FaceData(size_t face, size_t he) : faceIndex(face), enteredFromHalfEdge(he) {}
	};

	std::vector<FaceData> possibleVisibleFaces;
	std::deque<size_t> faceStack;

	void buildMesh(const std::vector<glm::vec3> &pointCloud, float defaultEps = 0.0001f);
	void createConvexHalfEdgeMesh();

	void setupInitialTetrahedron();

	void getConvexHull(const std::vector<glm::vec3> &pointCloud, bool CCW = true, bool useOriginalIndices = false);

	std::array<size_t, 6> getExtrema();

	float getScale();

	bool connectHorizonEdges(std::vector<size_t> &horizonEdges);

	bool addPointToFace(HalfEdgeMesh::Face &face, size_t pointIdx);

	inline std::unique_ptr<std::vector<size_t>> getConflictList();

	inline void reclaimConflictList(std::unique_ptr<std::vector<size_t>> &ptr);

	glm::vec3 computeCentroid();	

public:
	ConvexHull() = default;

	ConvexHull(string fileName, const std::vector<glm::vec3> &vertexData);

	void computeConvexHull(string fileName, const std::vector<glm::vec3> &vertexData, WorldTransform worldTrans);

	bool computeRayIntersection(const Ray &r, float &t);

	void updateCentroid(WorldTransform objectTrans);

	std::vector<glm::vec3> &getVertices();
	
	std::vector<int> &getIndices();

	WorldTransform &getWorldTransform();
        
	const std::vector<glm::vec3> &getVertices() const;
	
	const std::vector<int> &getIndices() const;
	
	void writeOBJ(const std::string &fileName, const std::string &objectName = "quickhull") const;
	
	// void getConvexHullAsMesh();

	void Draw(Shader &shader);
};

#endif
