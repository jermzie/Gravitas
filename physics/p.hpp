#include "ConvexHull.hpp"
#include "HalfEdgeMesh.hpp"

class NarrowPhase {

	struct FaceColInfo {

		float separation = 0.0f;
		size_t faceIdx;

		FaceCollision() = default;
		FaceCollision(float distance, size_t face) : separation(distance), faceIdx(face) {}

	};

	struct EdgeColInfo {

		float separation = 0.0f;
		std::pair<size_t, size_t> edgeIdx;

		EdgeCollision() = default;
		EdgeCollision(float distance, std::pair<size_t, size_t> edges) : separation(distance), edgeIdx(edges) {}
	};

	bool SATPolyPoly(const ConvexHull& polyA, const ConvexHull& polyB){

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

		// Check all edge combos between A & B -- O(n^2)
		EdgeColInfo eab = queryEdgeCombos(polyA, polyB);
		std::cout << "edge combos: " << eab.separation << std::endl;
		if (eab.separation > 0.0f) {
			return false;
		}

		return true;

	}

	findSupportPoint(const glm::vec3& direction, const ConvexHull& c) {

	}

	FaceColInfo queryFaceNormals(const ConvexHull& c1, const ConvexHull& c2) {

		float maxDist = -std::numeric_limits<float>::max();
		size_t maxIdx;

		size_t faceCount = c1.mesh.faces.size();
		for (size_t i = 0; i < faceCount; i++) {

			auto face = c1.mesh.faces[i];
			auto plane = face.P;

			// 
			glm::vec3 vertex = findSupportPoint(direction, c2);


			float dist = glm::dot(worldNormal, vertex) + worldDistance;

			if (dist > maxDist) {
				maxDist = dist;
				maxIdx = i;
			}
		}

		return FaceColInfo(maxDist, maxIdx);
	}

	EdgeColInfo queryEdgeCombos(const ConvexHull& c1, const ConvexHull& c2) {

		float maxDist = -std::numeric_limits<float>::max();
		std::pair<size_t, size_t> maxIdx;

		size_t edgeCount1 = c1.mesh.halfEdges.size();
		size_t edgeCount2 = c2.mesh.halfEdges.size();

		std::vector<bool> visited1(edgeCount1, false);
		for (size_t i = 0; i < edgeCount1; i++) {

			const auto& halfEdge1 = c1.mesh.halfEdges[i];
			visited1[i] = true;

			// skip twin half-edge
			if (visited1[halfEdge1.twin]) {
				continue;
			}


			// get indices to edge vertices
			const auto heIdx1 = c1.mesh.getHalfEdgeVertices(halfEdge1);
			const auto verts1 = c1.getVertices();

			glm::vec3 p1 = glm::vec3(modelA * glm::vec4(vertices1[heIndices1[0]], 1.0f));
			glm::vec3 q1 = glm::vec3(modelA * glm::vec4(vertices1[heIndices1[1]], 1.0f));
			glm::vec3 worldEdge1 = q1 - p1;



			// compute normal matrix
			glm::mat3 normA = getNormalMatrix(modelA);

			// get two face normals associated with edge
			glm::vec3 aLocal = polyA.mesh.faces[halfEdgeA.face].P.normal;
			glm::vec3 bLocal = polyA.mesh.faces[polyA.mesh.halfEdges[halfEdgeA.twin].face].P.normal;

			// transform to B's local space
			glm::vec3 aTrans = normA * aLocal;
			glm::vec3 bTrans = normA * bLocal;

			std::vector<bool> visitedB(edgeCountB, false);
			for (size_t j = 0; j < edgeCountB; j++) {

				auto& halfEdgeB = polyB.mesh.halfEdges[j];
				visitedB[j] = true;

				// skip twin half-edge
				if (visitedB[halfEdgeB.twin]) {
					continue;
				}

				// get indices to edge vertices
				const auto heIndicesB = polyB.mesh.getHalfEdgeVertices(halfEdgeB);
				const auto verticesB = polyB.getVertices();

				glm::vec3 p2 = verticesB[heIndicesB[0]];
				glm::vec3 q2 = verticesB[heIndicesB[1]];
				glm::vec3 worldEdgeB = q2 - p2;

				// hull B normals remain untransformed
				glm::vec3 cLocal = polyB.mesh.faces[halfEdgeB.face].P.normal;
				glm::vec3 dLocal = polyB.mesh.faces[polyB.mesh.halfEdges[halfEdgeB.twin].face].P.normal;


				// only check distances if edges form a minkowski face
				//if (buildMinkowskiFace(edgeA, edgeB, polyA, polyB)) {
				if (isMinkowskiFace(aTrans, bTrans, -cLocal, -dLocal)) {

					// find separation distance
					float distance = findSeparationDist(p1, worldEdgeA, p2, worldEdgeB, polyA.getCentroid());
					//float distance = findSeparationDist(p1, worldEdgeA, p2, worldEdgeB, centroidA);

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
	}
};