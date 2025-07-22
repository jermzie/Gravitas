#include "ConvexHull.h"
#include "HalfEdgeMesh.hpp"
#pragma once

#include "../inc/mesh.h"
#include "../inc/model.h"
#include "../inc/ray.h"

#include <vector>
#include <memory>
#include <cassert>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

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

void ConvexHull::getConvexHull() {}

void ConvexHull::buildMesh(const std::vector<glm::vec3>& pointCloud, float defaultEps) {

	if (pointCloud.size() == 0) {

		mesh = HalfEdgeMesh();
		return;
	}
	vertexData = pointCloud;

	// Find extreme values and use them to compute scale of point cloud
	extremaIndices = getExtrema();
	scale = getScale();

	// Epsilon determines tolerance of *fat planes*
	epsilon = defaultEps * scale;
	epsilonSquared = epsilon * epsilon;


	// Planar case -- when all points lie on a 2D subspace of R^3
	isPlanar = false;

	createConvexHalfEdgeMesh();			// Iteratively update mesh until ...

	if (isPlanar) {

	}

}

/*DOING*/
// Forms inital hull from extreme values
void ConvexHull::setupInitialTetrahedron() {

	const size_t vertexCount = vertexData.size();


	// Degenerate case -- just return degenerate tetrahedron
	if (vertexCount <= 4) {
		size_t v[4] = { 0, std::min((size_t)1,vertexCount-1), std::min((size_t)2,vertexCount-1), std::min((size_t)3,vertexCount-1) };

		const glm::vec3 normal = getTriangleNormal(vertexData[v[0]], vertexData[v[1]], vertexData[v[2]]);
		const Plane trianglePlane(normal, vertexData[v[0]]);

		// normal should point outwards away from tetrahedrons
		if(trianglePlane.isPointAbovePlane(vertexData[v[3]])){
			std::swap(v[0].v[1]);
		}

		return mesh.setup(v[0], v[1], v[2], v[3]);
	}


	// 1. Form a line between two furthest extrema vertices
	float maxDist = epsilonSquared;
	std::pair<size_t, size_t> selectedPoints;

	for (int i = 0; i < 6; i++) {
		for (int j = i + 1; j < 6; j++) {

			const float dist = glm::distance2(vertexData[extremaIndices[i]], vertexData[extremaIndices[j]);	// get squared distance between extreme values

			if (dist > maxDist){
				maxDist = dist;
				selectedPoints = { extremaIndices[i], extremaIndices[j] };
			}
		}
	}

	// Degenerate case -- point cloud seems to consist of a single point
	if (maxDist == epsilonSquared) {

		return mesh.setup(0, std::min((size_t)1, vertexCount - 1), std::min((size_t)2, vertexCount - 1), std::min((size_t)3, vertexCount - 1));
	}
	assert(selectedPoints.first != selectedPoints.second);


	// 2. Find point furthest from the line (forms a triangle face)
	maxDist = epsilonSquared;
	size_t maxIdx = std::numeric_limits<size_t>::max();

	const Ray r(vertexData[selectedPoints.first], (vertexData[selectedPoints.second] - vertexData[selectedPoints.first]));
	
	for (int i = 0; i < vertexCount; i++) {

		const float distToRay = getSquaredDistanceToRay(vertexData[i], r);
		
		if (distToRay > maxDist) {
			maxDist = distToRay;
			maxIdx = i;
		}
	}

	// Degenerate case -- point cloud seems to belong to 1D subspace of R^3 
	if (maxDist == epsilonSquared) {
		

		// Pick any distinct point and return a thin triangle
		auto it = std::find_if(m_vertexData.begin(), m_vertexData.end(), [&](const vec3& ve) {
			return ve != m_vertexData[selectedPoints.first] && ve != m_vertexData[selectedPoints.second];
			});

		const size_t thirdPoint = (it == m_vertexData.end()) ? selectedPoints.first : std::distance(m_vertexData.begin(), it);
		
		
		it = std::find_if(m_vertexData.begin(), m_vertexData.end(), [&](const vec3& ve) {
			return ve != m_vertexData[selectedPoints.first] && ve != m_vertexData[selectedPoints.second] && ve != m_vertexData[thirdPoint];
			});

		const size_t fourthPoint = (it == m_vertexData.end()) ? selectedPoints.first : std::distance(m_vertexData.begin(), it);
		return m_mesh.setup(selectedPoints.first, selectedPoints.second, thirdPoint, fourthPoint);


		return mesh.setup(selectedPoints.first, selectedPoints.second, thirdPoint, fourthPoint);
	}
	assert(maxIdx != selectedPoints.first && maxIdx != selectedPoints.second);

	// Forms base triangle for tetrahedron
	std::array<glm::vec3, 3>baseTriangle { 
		vertexData[selectedPoints.first], 
		vertexData[selectedPoints.second], 
		vertexData[maxIdx] 
	};

	std::array<size_t, 3>baseTriangleIndices{
		selectedPoints.first,
		selectedPoints.second,
		maxIdx
	};


	// 3. Find point furthest from the plane (forms a tetrahedron)
	maxDist = epsilonSquared;
	maxIdx = 0;

	const glm::vec3 normal = getTriangleNormal(baseTriangle[0], baseTriangle[1], baseTriangle[2]);
	const Plane p(normal, baseTriangle[0]);
	for (int i = 0; i < vertexCount; i++) {

		const float distToPlane  = std::abs(getSignedDistanceToPlane(vertexData[i], p));
		if (distToPlane > maxDist) {

			maxDist = distToPlane;
			maxIdx = i;
		}
	}

	// Degenerate case -- point cloud seems to lie on 2D subspace of R^3
	if (maxDist == epsilonSquared) {
		
		planar = true;
		const glm::vec3 n = getTriangleNormal(baseTriangle[1], baseTriangle[2], baseTriangle[0]);
	}


	// Enforce CCW winding -- OpenGL considers all CCW polygons to be front-facing by default
	const Plane trianglePlane(normal, baseTriangle[0]);
	if (trianglePlane.isPointAbovePlane(vertexData[maxIdx]) {

		std::swap(baseTriangleIndices[0], baseTriangleIndices[1]);
	}


	// Create iniial tetrahedron half edge mesh
	mesh.setup(baseTriangleIndices[0], baseTriangleIndices[1], baseTriangleIndices[2], maxIdx);

	// Compute planes defined by each triangle face
	for (auto& face : mesh.faces) {

		auto vFace = mesh.getFaceVertices(face);
		const glm::vec3& va = vertexData[vFace[0]];
		const glm::vec3& vb = vertexData[vFace[1]];
		const glm::vec3& vc = vertexData[vFace[2]];


		const glm::vec3 vNormal = getTriangleNormal(va, vb, vc);
		const Plane P(vNormal, va);

		face.P = P;
		
	}

	// 4. Assign points to each face if "outside" (vertices inside tetrahedron are ignored)
	for (int i = 0; i < vertexCount; i++) {
		for (auto& f : mesh.faces) {

			if (addPointToFace(f, i)) {
				break;
			}
		}
	}
}

// *DOING*
void ConvexHull::createConvexHalfEdgeMesh() {


	visibleFaces.clear();
	horizonEdges.clear();
	possibleVisibleFaces.clear();


	// Compute initial hull 
	setupInitialTetrahedron();
	assert(mesh.faces.size() == 4);

	// Initialize face stack
	faceStack.clear();
	for (int i = 0; i < 4; i++) {

		auto& f = mesh.faces[i];

		// Ensure face has a populated conflict list  before pushing onto stack
		if (f.pointsOnPositiveSide && f.pointsOnPositiveSide->size() > 0) {

			faceStack.push_back(i);
			f.inFaceStack = 1;			/*WTF is this a uint_8*/ 
		}
	}



	// Iterate over face stack until no "outside" points exist
	size_t it = 0;
	while (!faceStack.empty()) {
		it++;
		if (it == std:numeric_limits<size_t>::max()) {


			it = 0;
		}

		// Pop top face off stack
		const size_t faceIdx = faceStack.front();
		faceStack.pop_front();
		auto& face = mesh.faces[faceIdx];
		face.inFaceStack = 0;

		assert(!face.pointsOnPositiveSide || face.pointsOnPositiveSide->size() > 0);

		// Ignore faces with empty conflict lists or disabled faces
		if (!face.pointsOnPositiveSide || face.isDisabled) {

			continue;
		}


		// Choose furthest point as new vertex in convex hull
		const glm::vec3 point = vertexData[face.furthestPoint];
		const size_t pointIdx = face.furthestPoint;


		
		horizonEdges.clear();
		possibleVisibleFaces.clear();
		visibleFaces.clear();


		// Find all faces visible to point
		// Build a list of horizon edges
		possibleVisibleFaces.emplace_back(faceIdx, std::numeric_limits<size_t>::max());
		while (possibleVisibleFaces.size()) {

			const auto faceData = possibleVisibleFaces.back();
			possibleVisibleFaces.pop_back();


			// 
			auto& faceTest = mesh.faces[faceData.faceIndex];
			assert(!faceTest.isDisabled());


			// Check if face is already visible
			if (faceTest.visibilityCheckedOnIteration == it) {

				if (faceTest.isVisibleFaceOnCurrentIteration) {

					continue;
				}
			}

			// Test visibility
			else {

				const Plane& P = faceTest.P;
				faceTest.visibilityCheckedOnIteration = it;

				const float dist = getSignedDistanceToPlane(point, P);

				// Ensure point is outside plane
				if (dist > 0) {

					faceTest.isVisibleFaceOnCurrentIteration = 1;
					faceTest.horizonEdgesOnCurrentIteration = 0;

					//
					for (auto he : mesh.getFaceHalfEdges(faceTest)) {

						if (mesh.halfEdges[he].twin != faceData.enteredFromHalfEdge) {


							possibleVisibleFaces.emplace_back(mesh.halfEdges[mesh.halfEdges[he].twin].face, he);
						}
					}

					continue;

				}

				assert(faceData.faceIndex != faceIdx);

			}


			// Face is not visible, thus half edge is part of the horizon edge
			faceTest.isVisibleFaceOnCurrentIteration = 0;
			horizonEdges.push_back(faceData.enteredFromHalfEdge);

			const auto halfEdges = mesh.getFaceHalfEdges(mesh.faces[mesh.halfEdges[faceData.enteredFromHalfEdge].face]);
			
		
		}
		

		const size_t horizonEdgesCount = horizonEdges.size();


		// Attempt to form loop between horizon edges
		if (!connectHorizonEdges()) {

			std::cerr << "Failed to solve horizon edge." << std::endl;

			// Point is invalid and we don't add to convex hull
			auto it = std::find(face.pointsOnPositiveSide->begin(),
								face.pointsOnPositiveSide->end(),
								pointIdx);

			// Erase point from future iterations
			face.pointsOnPositiveSide->erase(it);
			if (face.pointsOnPositiveSide->size() == 0) {

				reclaimConflictList(face.pointsOnPositiveSide);
			}

			continue;
		}

		// Disable all visible faces & half-edges, we resuse their 
		for (auto faceIdx : visibleFaces) {

			auto& disabledFace = mesh.faces[faceIdx];
			auto halfEdges = mesh.getFaceHalfEdges(disabledFace);

			for (size_t j = 0; j < 3; j++) {

				if () {
					if () {

					}
					else {

					}
				}
			}
		}


		// Build new faces with horizon edges
		for (size_t i = 0; i < horizonEdgesCount; i++) {


			const size_t AB = horizonEdges[i];

			auto horizonEdgeVertices = mesh.getHalfEdgeVertices(mesh.halfEdges[AB]);
			size_t A, B, C;

			A = horizonEdgeVertices[0];
			B = horizonEdgeVertices[1];
			C = horizonEdgeVertices[2];

			const size_t newFaceIdx = mesh.addFace();


			mesh.halfEdges[AB].next = BC;
			mesh.halfEdges[BC].next = CA;
			mesh.halfEdges[CA].next = AB;

			mesh.halfEdges[BC].face = newFaceIdx;
			mesh.halfEdges[CA].face = newFaceIdx;
			mesh.halfEdges[AB].face = newFaceIdx;

			mesh.halfEdges[CA].vert = A;
			mesh.halfEdges[BC].vert = C;


		}

	}

	

}


/*DONE*/
// Returns indices to extreme values
std::array<size_t, 6> ConvexHull::getExtrema() {

	std::array<size_t, 6> outIndices{ 0, 0, 0, 0, 0, 0 };
	extremeValues{ vertexData[0].x, vertexData[0].x, vertexData[0].y, vertexData[0].y, vertexData[0].z, vertexData[0].z };

	for (int i = 0; i < vertexData.size(); i++) {

		const glm::vec3& pos = vertexData[i];

		// X-axis
		if (pos.x < extremeValues[0]) {
			extremeValues[0] = pos.x;
			outIndices[0] = i;
		}

		else if (pos.x > extremeValues[1]) {
			extremeValues[1] = pos.x;
			outIndices[1] = i;
		}

		// Y-axis
		if (pos.y < extremeValues[2]) {
			extremeValues[2] = pos.y;
			outIndices[2] = i;
		}

		else if (pos.y > extremeValues[3]) {
			extremeValues[3] = pos.y;
			outIndices[3] = i;
		}

		// Z-axis
		if (pos.z < extremeValues[4]) {
			extremeValues[4] = pos.z;
			outIndices[4] = i;
		}

		else if (pos.z > extremeValues[5]) {
			extremeValues[5] = pos.z;
			outIndices[5] = i;
		}
	}

	return outIndices;
}

/*DONE*/
// Returns scale for computing epsilon
float ConvexHull::getScale() {

	float s = 0.0f;
	for (int i = 0; i < 6; i++) {

		const float* v = (const float*)(&vertexData[extremaIndices[i]]);
		v += i / 2;

		s = std::max(s, std::abs(*v));
	}

	return s;
}


/*TODO*/
// Check if horizon edges form connected loop
bool ConvexHull::connectHorizonEdges() {


	

}

/*DONE*/
bool ConvexHull::addPointToFace(HalfEdgeMesh::Face& face, size_t pointIdx) {
	
	// Negative dist means point is inside the hull
	const float distToPlane = getSignedDistanceToPlane(vertexData[pointIdx], face.P);

	// Check if point lies above plane and above epsilon tolerance -- |dist| is greater than epsilon * ||normal||
	if (distToPlane > 0 && distToPlane * distToPlane > epsilonSquared * face.P.normLengthSq) {

		if (!face.pointsOnPositiveSide) {

			face.pointsOnPositiveSide = std::move(getConflictList());
		}

		face.pointsOnPositiveSide->push_back(pointIdx);

		if (distToPlane > face.furthestPointDist) {

			face.furthestPointDist = distToPlane;
			face.furthestPoint = pointIdx;
		}

		return true;

	}

	return false;
}

/*DONE*/
std::unique_ptr<std::vector<size_t>> ConvexHull::getConflictList() {

	// Ensure pool isn't empty
	if (conflictListsPool.size() == 0) {
		return std::unique_ptr<T>(new T());
	}

	auto it = conflictListsPool.end() - 1;


	std::unique_ptr<std::vector<size_t>> conflictListPtr = std::move(*it);
	conflictListsPool.erase(it);

	conflictListPtr->clear();
	return conflictListPtr;
	
}

/*DONE*/
void ConvexHull::reclaimConflictList(std::unique_ptr<std::vector<size_t>>& ptr) {

	const size_t ize = ptr->size();

	// 
	if ((size + 1) * 128 < ptr->capacity()) {

		ptr.reset(nullptr);
		return;
	}

	conflictListsPool.push_back(std::move(ptr));
}
