#ifndef SAT_HPP
#define SAT_HPP


// SEMI WORKS BUT BAD TIME COMPLEXITY


#include "ConvexHull.hpp"
#include "HalfEdgeMesh.hpp"
#include "../inc/Debugger.hpp"

// SAT is purely a test to check if two polygons/hedra are intersecting
// 

// possible separation axes include: 
// All face normals of poly A
// All face normals of poly B
// Cross products of all edge combos between A and B (3D only)


class SAT {
public:


	struct FaceColInfo {

		float separation = 0.0f;
		size_t faceIdx;

		FaceColInfo() = default;
		FaceColInfo(float distance, size_t face) : separation(distance), faceIdx(face) {}

	};

	struct EdgeColInfo {

		float separation = 0.0f;
		std::pair<size_t, size_t> edgeIdx;

		EdgeColInfo() = default;
		EdgeColInfo(float distance, std::pair<size_t, size_t> edges) : separation(distance), edgeIdx(edges) {}
	};

	struct ContactPoint {

		glm::vec3 point;	// world-space contact point
		glm::vec3 normal;   // contact normal from A -> B
		float penetration;	// penetration depth

	};

	struct ContactManifold {

		float penetration;
		glm::vec3 normal;
		std::vector<ContactPoint> point;
	};

	// O(n^2)
	// Gauss Map Optimization
	bool SATPolyPoly(const ConvexHull& polyA, const ConvexHull& polyB) {

		/*std::cout << "polyA Centroid ";
		printVec(polyA.getCentroid());
		std::cout << "polyB Centroid ";
		printVec(polyB.getCentroid());*/


		// All computations in local space of hull B -- multiply A by inverse transformations of B
		glm::mat4 modelB = polyB.getWorldTransform().GetMatrix();
		//glm::mat4 modelA = polyA.getWorldTransform().GetMatrix();
		glm::mat4 modelA = glm::inverse(modelB) * polyA.getWorldTransform().GetMatrix();


		// Check all face normals of A -- O(n^2)
		FaceColInfo fa = queryFaceNormals(polyA, polyB, modelA);
		std::cout << "polyA normals: " << fa.separation << std::endl;
		if (fa.separation > 0.0f) {
			return false;
		}

		// Check all face normals of B -- O(n^2)
		FaceColInfo fb = queryFaceNormals(polyB, polyA, glm::inverse(modelA));
		std::cout << "polyB normals: " << fb.separation << std::endl;
		if (fb.separation > 0.0f) {
			return false;
		}

		// Check all edge combos between -- O(n^2)
		EdgeColInfo eab = queryEdgeCombos(polyA, polyB);
		std::cout << "edge combos: " << eab.separation << std::endl;

		if (eab.separation > 0.0f) {
			return false;
		}

		return true;
	}

	FaceColInfo queryFaceNormals(const ConvexHull& polyA, const ConvexHull& polyB, glm::mat4 trans) {

		float maxDist = -std::numeric_limits<float>::max();
		size_t maxIdx;

		const size_t faceCount = polyA.mesh.faces.size();
		for (size_t i = 0; i < faceCount; i++) {

			const HalfEdgeMesh::Face face = polyA.mesh.faces[i];
			Plane plane = face.P;

			// Transform normal into hull B's local space
			glm::mat3 normalMat = getNormalMatrix(trans);
			glm::vec3 axis = normalMat * -plane.normal;

			// Find furthest vertex in direction opposite to normal
			glm::vec3 vertex = findSupportPoint(axis, polyB);

			// Compute separation distance between vertex and face plane
			plane.normal = normalMat * plane.normal;
			plane.point = glm::vec3(trans * glm::vec4(plane.point, 1.0f));
			plane.distance = -glm::dot(plane.normal, plane.point);

			float dist = getSignedDistanceToPlane(vertex, plane);

			if (dist > maxDist) {
				maxDist = dist;
				maxIdx = i;
			}
		}

		return FaceColInfo(maxDist, maxIdx);
	}

	EdgeColInfo queryEdgeCombos(const ConvexHull& polyA, const ConvexHull& polyB) {

		glm::mat4 modelB = polyB.getWorldTransform().GetMatrix();
		glm::mat4 modelA = polyA.getWorldTransform().GetMatrix();
		glm::mat4 trans = glm::inverse(modelB) * modelA;

		// transform world centroid pos. (getCentroid()) into B's local space
		glm::vec3 centroidA = glm::vec3(trans * glm::vec4(polyA.getCentroid(), 1.0f));
		glm::vec3 centroidB = polyB.getLocalCentroid();


		size_t edgeCountA = polyA.mesh.halfEdges.size();
		size_t edgeCountB = polyB.mesh.halfEdges.size();


		float maxDist = -std::numeric_limits<float>::max();
		std::pair<size_t, size_t> maxIdx;

		glm::mat3 normMatA = getNormalMatrix(trans);


		std::vector<bool> visitedA(edgeCountA, false);
		for (size_t i = 0; i < edgeCountA; i++) {

			// skip twin half-edge
			if (visitedA[i]) {
				continue;
			}

			const auto halfEdgeA = polyA.mesh.halfEdges[i];
			visitedA[i] = true;
			visitedA[halfEdgeA.twin] = true;

			// get indices to halfedge vertices
			const auto heIdxA = polyA.mesh.getHalfEdgeVertices(halfEdgeA);
			const auto vertsA = polyA.getVertices();

			// transform A's vertices into B's local space
			glm::vec3 p1 = glm::vec3(trans * glm::vec4(vertsA[heIdxA[0]], 1.0f));
			glm::vec3 q1 = glm::vec3(trans * glm::vec4(vertsA[heIdxA[1]], 1.0f));
			glm::vec3 edgeA = q1 - p1;


			// get face normals for edge A in B's local space
			glm::vec3 aLocal = polyA.mesh.faces[halfEdgeA.face].P.normal;
			glm::vec3 bLocal = polyA.mesh.faces[polyA.mesh.halfEdges[halfEdgeA.twin].face].P.normal;
			glm::vec3 aNorm = glm::normalize(normMatA * aLocal);
			glm::vec3 bNorm = glm::normalize(normMatA * bLocal);


			std::vector<bool> visitedB(edgeCountB, false);
			for (size_t j = 0; j < edgeCountB; j++) {

				// skip twin half-edge
				if (visitedB[j]) {
					continue;
				}

				const auto halfEdgeB = polyB.mesh.halfEdges[j];
				visitedB[j] = true;
				visitedB[halfEdgeB.twin] = true;

				// get indices to edge vertices
				const auto heIdxB = polyB.mesh.getHalfEdgeVertices(halfEdgeB);
				const auto vertsB = polyB.getVertices();

				glm::vec3 p2 = vertsB[heIdxB[0]];
				glm::vec3 q2 = vertsB[heIdxB[1]];
				glm::vec3 edgeB = q2 - p2;


				// get face normals for edge B in B's local space
				glm::vec3 cNorm = glm::normalize(polyB.mesh.faces[halfEdgeB.face].P.normal);
				glm::vec3 dNorm = glm::normalize(polyB.mesh.faces[polyB.mesh.halfEdges[halfEdgeB.twin].face].P.normal);


				// check if edges form Minkowski face
				// negate hull B's normals to account for Minkowski difference
				if (isMinkowskiFace(aNorm, bNorm, -cNorm, -dNorm)) {



					// skip near-parallel edges
					glm::vec3 cross = glm::cross(edgeA, edgeB);
					float crossLength = glm::length(cross);

					if (crossLength > 0.005f * glm::sqrt(glm::length2(edgeA) * glm::length2(edgeB))) {

						// compute separating axis
						glm::vec3 axis = cross / crossLength;

						// ensure axis points from A to B
						if (glm::dot(axis, centroidB - centroidA) < 0.0f) {
							axis = -axis;
						}

						/*
						// Compute distance between edges along the axis
						float separation = glm::dot(axis, p2 - p1);

						if (separation > maxDist) {
							maxDist = separation;
							maxIdx = { i, j };
						}
						*/

						// Project all vertices of both hulls onto the axis
						float minA = std::numeric_limits<float>::max();
						float maxA = -std::numeric_limits<float>::max();

						// Get all vertices of A in B's local space
						const auto vertsA_all = polyA.getVertices();
						for (size_t k = 0; k < vertsA_all.size(); k++) {
							glm::vec3 vertA_local = glm::vec3(trans * glm::vec4(vertsA_all[k], 1.0f));
							float projection = glm::dot(vertA_local, axis);
							minA = std::min(minA, projection);
							maxA = std::max(maxA, projection);
						}

						float minB = std::numeric_limits<float>::max();
						float maxB = -std::numeric_limits<float>::max();

						// Get all vertices of B (already in B's local space)
						const auto vertsB_all = polyB.getVertices();
						for (size_t k = 0; k < vertsB_all.size(); k++) {
							float projection = glm::dot(vertsB_all[k], axis);
							minB = std::min(minB, projection);
							maxB = std::max(maxB, projection);
						}

						// Compute separation (positive = separated, negative = overlapping)
						// Since axis points from A to B: separation = minB - maxA
						float separation = minB - maxA;

						// Update if this is the smallest (least negative or smallest positive) separation found
						if (separation > maxDist) {
							maxDist = separation;
							maxIdx = { i, j };
						}
					}
				}
			}
		}

		return EdgeColInfo(maxDist, maxIdx);
	}

	// Normal transformations use different matrix from model matrix; 
	static inline glm::mat3 getNormalMatrix(glm::mat4 model) {
		return glm::transpose(glm::inverse(glm::mat3(model)));
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
		return CBA * DBA < 0.0f && ADC * BDC < 0.0f && CBA * BDC > 0.0f;
	}

	glm::vec3 findSupportPoint(glm::vec3 axis, const ConvexHull& poly) {

		auto localVerts = poly.getVertices();

		float maxProj = -std::numeric_limits<float>::max();
		glm::vec3 maxVert;
		for (size_t i = 0; i < localVerts.size(); i++) {

			//glm::vec3 worldVert = glm::vec3(model * glm::vec4(localVerts[i], 1.0f));
			float projection = glm::dot(localVerts[i], axis);

			if (projection > maxProj) {

				maxProj = projection;
				maxVert = localVerts[i];
			}
		}

		return maxVert;
	}

	void printVec(glm::vec3 v) {
		std::cout << "vec3(" << v.x << " " << v.y << " " << v.z << ")\n";
	}
};

#endif
