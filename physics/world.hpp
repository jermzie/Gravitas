

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

	struct Interval {
		glm::vec3 min;
		glm::vec3 max;

		/*bool overlaps(const Interval& other) const {
			return !(max < other.min || other.max < min);
		}*/

		float getPenetration(const Interval& other) const {

		}
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

	// O(n^3)
	// Brute Force Approach
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
	// Gauss Map Optimization
	bool SATPolyPoly(const ConvexHull& polyA, const ConvexHull& polyB) {

		std::cout << "polyA Centroid ";
		printVec(polyA.getCentroid());
		std::cout << "polyB Centroid ";
		printVec(polyB.getCentroid());


		// Check all face normals of A -- O(n^2)
		FaceColInfo fa = queryFaceNormals(polyA, polyB);
		std::cout << "polyA normals: " << fa.separation << std::endl;
		if (fa.separation > 0.0f) {
			return false;
		}

		// Check all face normals of B -- O(n^2)
		FaceColInfo fb = queryFaceNormals(polyB, polyA);
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

	FaceColInfo queryFaceNormals(const ConvexHull& polyA, const ConvexHull& polyB) {

		// All computations in local space of hull B -- multiply A by inverse transformations of B
		glm::mat4 modelB = polyB.getWorldTransform().GetMatrix();
		glm::mat4 modelA = polyA.getWorldTransform().GetMatrix();
		//glm::mat4 modelA = glm::inverse(modelB) * polyA.getWorldTransform().GetMatrix();


		float maxDist = -std::numeric_limits<float>::max();
		size_t maxIdx;

		const size_t faceCount = polyA.mesh.faces.size();
		for (size_t i = 0; i < faceCount; i++) {

			const HalfEdgeMesh::Face face = polyA.mesh.faces[i];
			Plane plane = face.P;

			// Transform normal into [] space
			glm::mat3 normalMat = getNormalMatrix(modelA);
			glm::vec3 direction = normalMat * -plane.normal;

			// Find furthest vertex in direction opposite to normal
			glm::vec3 vertex = findSupportPoint(direction, polyB);

			// Compute separation distance between vertex and face plane
			plane.normal = normalMat * plane.normal;
			plane.point = glm::vec3(modelA * glm::vec4(plane.point, 1.0f));
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


		// All computations in local space of hull B -- multiply A by inverse transformations of B
		glm::mat4 modelB = polyB.getWorldTransform().GetMatrix();
		glm::mat4 modelA = polyA.getWorldTransform().GetMatrix();
		//glm::mat4 modelA = glm::inverse(modelB) * polyA.getWorldTransform().GetMatrix();
		//glm::vec3 centroidA = glm::vec3(modelA * glm::vec4(polyA.getCentroid(), 1.0f));

		size_t edgeCountA = polyA.mesh.halfEdges.size();
		size_t edgeCountB = polyB.mesh.halfEdges.size();


		float maxDist = -std::numeric_limits<float>::max();
		std::pair<size_t, size_t> maxIdx;

		std::vector<bool> visitedA(edgeCountA, false);
		for (size_t i = 0; i < edgeCountA; i++) {

			const auto halfEdgeA = polyA.mesh.halfEdges[i];
			visitedA[i] = true;

			// skip twin half-edge
			if (visitedA[halfEdgeA.twin]) {
				continue;
			}

			// get indices to halfedge vertices
			const auto heIdxA = polyA.mesh.getHalfEdgeVertices(halfEdgeA);
			const auto vertsA = polyA.getVertices();

			glm::vec3 p1 = glm::vec3(modelA * glm::vec4(vertsA[heIdxA[0]], 1.0f));
			glm::vec3 q1 = glm::vec3(modelA * glm::vec4(vertsA[heIdxA[1]], 1.0f));

			glm::vec3 edgeA = q1 - p1;


			// compute normal matrix
			glm::mat3 normA = getNormalMatrix(modelA);

			// get two face normals associated with edge
			glm::vec3 aLocal = polyA.mesh.faces[halfEdgeA.face].P.normal;
			glm::vec3 bLocal = polyA.mesh.faces[polyA.mesh.halfEdges[halfEdgeA.twin].face].P.normal;

			// transform to [] space
			glm::vec3 aTrans = normA * aLocal;
			glm::vec3 bTrans = normA * bLocal;

			std::vector<bool> visitedB(edgeCountB, false);
			for (size_t j = 0; j < edgeCountB; j++) {

				const auto halfEdgeB = polyB.mesh.halfEdges[j];
				visitedB[j] = true;

				// skip twin half-edge
				if (visitedB[halfEdgeB.twin]) {
					continue;
				}

				// get indices to edge vertices
				const auto heIdxB = polyB.mesh.getHalfEdgeVertices(halfEdgeB);
				const auto vertsB = polyB.getVertices();

				glm::vec3 p2 = glm::vec3(modelB * glm::vec4(vertsB[heIdxB[0]], 1.0f));
				glm::vec3 q2 = glm::vec3(modelB * glm::vec4(vertsB[heIdxB[1]], 1.0f));

				//glm::vec3 p2 = vertsB[heIdxB[0]];
				//glm::vec3 q2 = vertsB[heIdxB[1]];

				glm::vec3 edgeB = q2 - p2;


				// compute normal matrix
				glm::mat3 normB = getNormalMatrix(modelB);

				// hull B normals remain untransformed
				glm::vec3 cLocal = polyB.mesh.faces[halfEdgeB.face].P.normal;
				glm::vec3 dLocal = polyB.mesh.faces[polyB.mesh.halfEdges[halfEdgeB.twin].face].P.normal;

				// transform to [] space
				glm::vec3 cTrans = normB * cLocal;
				glm::vec3 dTrans = normB * dLocal;

				// check separation distances only if edges form a minkowski face
				//if (buildMinkowskiFace(edgeA, edgeB, polyA, polyB)) {
				if (isMinkowskiFace(aTrans, bTrans, -cTrans, -dTrans)) {

					// find separation distance
					float distance = findSeparationDist(p1, edgeA, p2, edgeB, polyA.getCentroid());
					//float distance = findSeparationDist(p1, edgeA, p2, edgeB, centroidA);

					if (distance > maxDist) {
						maxDist = distance;
						maxIdx.first = i;
						maxIdx.second = j;
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

	/*static inline std::array<glm::vec3, 2> transformHalfEdgeVertices(const ConvexHull& poly, const glm::mat4& model, std::array<size_t, 2> heIndices) {

		std::vector<glm::vec3> localVertices = poly.getVertices();
		std::array<glm::vec3, 2> worldVertices;

		for (size_t i = 0; i < 2; i++) {
			worldVertices[i] = glm::vec3(model * glm::vec4(localVertices[heIndices[i]], 1.0f));
		}

		return worldVertices;
	}*/


	static inline bool isParallel(const glm::vec3& a, const glm::vec3& b, const float& eps = 0.005f) {

		glm::vec3 cross = glm::cross(a, b);
		float L = glm::length(cross);
		float denom = glm::sqrt(glm::dot(a, a) * glm::dot(b, b));
		if (denom <= 1e-12f) return true; // degenerate edge -> treat as parallel/ignore
		return L < eps * denom;
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

	float findSeparationDist(const glm::vec3& pointA, const glm::vec3& edgeA, const glm::vec3& pointB, const glm::vec3& edgeB, const glm::vec3& centroidA) {

		// skip near parallel edges
		//glm::vec3 n = glm::cross(edgeA, edgeB);
		if (isParallel(edgeA, edgeB)) {
			return -std::numeric_limits<float>::max();
		}

		//const float eps = 1e-6f;
		//float l = glm::length(n);
		//if (l < eps * glm::sqrt(glm::length2(edgeA) * glm::length2(edgeB))) {
		//	return -std::numeric_limits<float>::max();
		//}

		//// separating axis orthogonal to both edges
		//glm::vec3 normal = n / l;

		// ensure normal points from A -> B

		glm::vec3 normal = glm::normalize(glm::cross(edgeA, edgeB));
		if (glm::dot(normal, pointA - centroidA) < 0.0f) {
			normal = -normal;
		}

		return glm::dot(normal, pointB - pointA);
	}

	/*
	glm::vec3 findSupportPoint(glm::vec3 direction, const ConvexHull& c) {

		auto vLocal = c.getVertices();

		float maxProj = -std::numeric_limits<float>::max();
		glm::vec3 maxVert;

		for (size_t i = 0; i < vLocal.size(); i++) {

			//glm::vec3 worldVert = glm::vec3(model * glm::vec4(localVerts[i], 1.0f));
			float projection = glm::dot(vLocal[i], direction);

			if (projection > maxProj) {

				maxProj = projection;
				maxVert = vLocal[i];
			}
		}

		return maxVert;
	}
	*/
	void printVec(glm::vec3 v) {
		std::cout << "vec3(" << v.x << " " << v.y << " " << v.z << ")\n";
	}

	glm::vec3 findSupportPoint(glm::vec3 direction, const ConvexHull& poly) {

		//std::vector<glm::vec3> vertices;
		//transformVertices(poly, modelMat4, vertices);

		glm::mat4 model = poly.getWorldTransform().GetMatrix();
		auto localVerts = poly.getVertices();

		float maxProj = -std::numeric_limits<float>::max();
		glm::vec3 maxVert;
		for (size_t i = 0; i < localVerts.size(); i++) {

			glm::vec3 worldVert = glm::vec3(model * glm::vec4(localVerts[i], 1.0f));
			float projection = glm::dot(worldVert, direction);

			if (projection > maxProj) {

				maxProj = projection;
				maxVert = worldVert;
			}
		}

		return maxVert;
	}
};

