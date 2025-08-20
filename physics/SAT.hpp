#ifndef SAT_HPP
#define SAT_HPP

#include "ConvexHull.hpp"
#include "HalfEdgeMesh.hpp"

// SAT is purely a test to check if two polygons/hedra are intersecting
// 

// possible separation axes include: 
// All face normals of poly A
// All face normals of poly B
// Cross products of all edge combos between A and B (3D only)


class SAT {
public:

	struct Interval {
		glm::vec3 min;
		glm::vec3 max;
	};

	struct FaceCollision {

		float separation;
		size_t faceIdx;

		FaceCollision() = default;
		FaceCollision(float distance, size_t face) : separation(distance), faceIdx(face) {}

	};

	struct EdgeCollision {

		float separation;
		std::pair<size_t, size_t> edgeIdx;

		EdgeCollision() = default;
		EdgeCollision(float distance, std::pair<size_t, size_t> edges) : separation(distance), edgeIdx(edges) {}
	};

	// O(n^3)
	// Iterate over all edges of A and B to test possible axes
	// Project vertices of A and B onto axes
	// Compare interval overlap

	/*
	bool bruteForceSAT(const ConvexHull& polyA, const ConvexHull& polyB) {

		auto verticesA = polyA.getVertices();
		auto verticesB = polyB.getVertices();

		size_t edgeCountA = polyA.mesh.halfEdges.size();
		size_t edgeCountB = polyB.mesh.halfEdges.size();

		std::vector<bool> visitedA(edgeCountA, false);
		std::vector<bool> visitedB(edgeCountB, false);

		for (size_t i = 0; i < edgeCountA; i++) {

			auto& halfEdgeA = polyA.mesh.halfEdges[i];
			visitedA[i] = true;

			// skip twin
			if (visitedA[halfEdgeA.twin]) {
				continue;
			}

			for (size_t j = 0; j < edgeCountB; j++) {

				auto& halfEdgeB = polyB.mesh.halfEdges[j];
				visitedB[j] = true;

				// skip twin
				if (visitedB[halfEdgeB.twin]) {
					continue;
				}

				// calculate separating axis 
				auto va = polyA.mesh.getHalfEdgeVertices(halfEdgeA);
				auto vb = polyB.mesh.getHalfEdgeVertices(halfEdgeB);

				glm::vec3 edgeA = verticesA[va[1]] - verticesA[va[0]];
				glm::vec3 edgeB = verticesB[vb[1]] - verticesB[vb[0]];
				glm::vec3 axis = glm::cross(edgeA, edgeB);

				// project both hulls onto axis -- O(n)
				Interval intA = projectToAxis(axis, polyA);
				Interval intB = projectToAxis(axis, polyB);


				// check separation and find penetration depth
				float separation = compareIntervals(intA, intB);

			}
		}

		return true;
	}

	// brute force helper functions
	float compareIntervals(Interval intervalA, Interval intervalB) {


	}

	Interval projectToAxis(const glm::vec3& axis, const ConvexHull& poly) {

		Interval extrema;

		for (auto& vertex : poly.getVertices()) {

			// kinda?
			glm::projection = glm::dot(axis, vertex);

			extrema.max = std::max();
			extrema.min = std::min();

		}

		return extrema;

	}
	*/

	// O(n^2)
	// Optimized w/ Gauss Maps
	bool optimizedSAT(const ConvexHull& polyA, const ConvexHull& polyB) {

		// Check all face normals of A -- O(n^2)
		FaceCollision fa = queryFaceNormals(polyA, polyB);
		if (fa.separation > 0.0f) {
			return false;
		}

		// Check all face normals of B -- O(n^2)
		FaceCollision fb = queryFaceNormals(polyB, polyA);
		if (fb.separation > 0.0f) {
			return false;
		}

		// Check all edge combos between -- O(n^2)
		EdgeCollision eab = queryEdgeCombos(polyA, polyB);
		if (eab.separation > 0.0f) {
			return false;
		}

		return true;
	}
	
	FaceCollision queryFaceNormals(const ConvexHull& poly, const ConvexHull& other) {

		float maxDist = -std::numeric_limits<float>::max();
		size_t maxIdx;

		size_t faceCount = poly.mesh.faces.size();

		for (size_t i = 0; i < faceCount; i++) {

			auto face = poly.mesh.faces[i];
			auto plane = face.P;

			// search for most extreme point opposite of normal direction
			// if distance is negative, it's a penetration
			glm::vec3 vertex = findSupportPoint(-plane.normal, other);
			float distance = getSignedDistanceToPlane(vertex, plane);

			if (distance > maxDist) {
				maxDist = distance;
				maxIdx = i;
			}
		}

		return FaceCollision(maxDist, maxIdx);
	}

	EdgeCollision queryEdgeCombos(const ConvexHull& polyA, const ConvexHull& polyB) {

		float maxDist = -std::numeric_limits<float>::max();
		std::pair<size_t, size_t> maxIdx;

		size_t edgeCountA = polyA.mesh.halfEdges.size();
		size_t edgeCountB = polyB.mesh.halfEdges.size();

		std::vector<bool> visitedA(edgeCountA, false);
		std::vector<bool> visitedB(edgeCountB, false);

		for (size_t i = 0; i < edgeCountA; i++) {

			auto& edgeA = polyA.mesh.halfEdges[i];
			visitedA[i] = true;

			// skip twin
			if (visitedA[edgeA.twin]) {
				continue;
			}

			for (size_t j = 0; j < edgeCountB; j++) {

				auto& edgeB = polyB.mesh.halfEdges[j];
				visitedB[j] = true;

				// skip twin
				if (visitedB[edgeB.twin]) {
					continue;
				}

				// check if edges form possible separating axis
				if (buildMinkowskiFace(edgeA, edgeB, polyA, polyB)) {

					// find separation distance
					float distance = findSeparationDist(edgeA, edgeB, polyA, polyB);

					if (distance > maxDist) {
						maxDist = distance;
						maxIdx.first = i;
						maxIdx.second = j;
					}

				}
			}
		}

		return EdgeCollision(maxDist, maxIdx);
	}

	bool buildMinkowskiFace(const HalfEdgeMesh::HalfEdge& halfEdgeA, const HalfEdgeMesh::HalfEdge& halfEdgeB, const ConvexHull& polyA, const ConvexHull& polyB) {

		// get two face normals associated with edge
		// just care abt sign, no need to normalize
		glm::vec3 a = polyA.mesh.faces[halfEdgeA.face].P.normal;
		glm::vec3 b = polyA.mesh.faces[polyA.mesh.halfEdges[halfEdgeA.twin].face].P.normal;
		glm::vec3 c = polyB.mesh.faces[halfEdgeB.face].P.normal;
		glm::vec3 d = polyB.mesh.faces[polyB.mesh.halfEdges[halfEdgeB.twin].face].P.normal;

		// negate c and d for Minkowski difference
		return isMinkowskiFace(a, b, -c, -d);

	}

	bool isMinkowskiFace(const glm::vec3& a, const glm::vec3& b, const glm::vec3& c, const glm::vec3& d) {

		// recall vertices are simply normals of adjacent faces for both edges
		// test if arcs AB and CD intersect on unit sphere

		// planes through arcs AB and CD respectively
		glm::vec3 BxA = glm::cross(b, a);
		glm::vec3 DxC = glm::cross(d, c);

		// find signed distance to plane
		float CBA = glm::dot(c, BxA);
		float DBA = glm::dot(d, BxA);
		float ADC = glm::dot(a, DxC);
		float BDC = glm::dot(b, DxC);

		// overlap test -- if vertices separated by plane, sign is negative
		// hemisphere test (edge case) -- overlap test fails if arcs on different hemispheres
		return CBA * DBA < 0 && ADC * BDC < 0 && CBA * BDC > 0;
	}

	float findSeparationDist(const HalfEdgeMesh::HalfEdge& halfEdgeA, const HalfEdgeMesh::HalfEdge& halfEdgeB, const ConvexHull& polyA, const ConvexHull& polyB) {

		auto verticesA = polyA.getVertices();
		auto verticesB = polyB.getVertices();

		auto va = polyA.mesh.getHalfEdgeVertices(halfEdgeA);
		auto vb = polyB.mesh.getHalfEdgeVertices(halfEdgeB);

		glm::vec3 edgeA = verticesA[va[1]] - verticesA[va[0]];
		glm::vec3 pointA = verticesA[va[1]];
		glm::vec3 edgeB = verticesB[vb[1]] - verticesB[vb[0]];
		glm::vec3 pointB = verticesB[vb[1]];

		// skip parallel edges within eps?
		if (glm::dot(edgeA, edgeB) == 0.0f) return -std::numeric_limits<float>::max();

		// normalized cross product
		glm::vec3 normal = glm::cross(edgeA, edgeB);
		normal = glm::normalize(normal);

		if (glm::dot(normal, pointA - polyA.getCentroid()) < 0.0f) {
			normal = -normal;
		}

		return glm::dot(normal, pointB - pointA);

	}

	glm::vec3 findSupportPoint(const glm::vec3& direction, const ConvexHull& poly) {

		float maxProj = -std::numeric_limits<float>::max();
		glm::vec3 maxVert;
		for (auto& vertex : poly.getVertices()) {

			float projection = glm::dot(vertex, direction);

			if (projection > maxProj) {

				maxProj = projection;
				maxVert = vertex;
			}
		}

		return maxVert;
	}
};

#endif
