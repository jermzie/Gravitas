#ifndef HALFEDGEMESH_HPP
#define HALFEDGEMESH_HPP

#include <vector>
#include <unordered_map>

#include "MeshBuilder.hpp"

class HalfEdgeMesh {
public:

	struct HalfEdge
	{
		size_t vert;
		size_t twin;
		size_t face;
		size_t next;
	};

	struct Face
	{
		size_t he; // Index of one of its half edges
		Plane P; // Should recompute normals ???
	};

	std::vector<glm::vec3> vertices;
	std::vector<Face> faces;
	std::vector<HalfEdge> halfEdges;

	HalfEdgeMesh() = default;

	HalfEdgeMesh(const MeshBuilder& buildMesh, const std::vector<glm::vec3>& pointCloud) {

		std::unordered_map<size_t, size_t> faceMap;
		std::unordered_map<size_t, size_t> halfEdgeMap;
		std::unordered_map<size_t, size_t> vertexMap;

		size_t i = 0;
		for (const auto& f : buildMesh.faces) {

			if (!f.isDisabled()) {

				faces.push_back({ static_cast<size_t>(f.he),  static_cast<Plane>(f.P) });
				//faces.push_back({ static_cast<size_t>(f.he) });
				faceMap[i] = faces.size() - 1;

				const auto heIndices = buildMesh.getFaceHalfEdges(f);
				for (const auto he : heIndices) {

					const size_t vertex2index = buildMesh.halfEdges[he].vert;

					if (vertexMap.count(vertex2index) == 0) {
						vertices.push_back(pointCloud[vertex2index]);
						vertexMap[vertex2index] = vertices.size() - 1;
					}
				}
			}
			i++;
		}

		i = 0;
		for (const auto& he : buildMesh.halfEdges) {
			if (!he.isDisabled()) {

				halfEdges.push_back({ static_cast<size_t>(he.vert), static_cast<size_t>(he.twin), static_cast<size_t>(he.face), static_cast<size_t>(he.next) });
				halfEdgeMap[i] = halfEdges.size() - 1;
			}
			i++;
		}

		for (auto& f : faces)
		{
			assert(halfEdgeMap.count(f.he) == 1);
			f.he = halfEdgeMap[f.he];
		}

		for (auto& he : halfEdges)
		{
			he.face = faceMap[he.face];
			he.twin = halfEdgeMap[he.twin];
			he.next = halfEdgeMap[he.next];
			he.vert = vertexMap[he.vert];
		}

		//debugState();
	}

	std::array<size_t, 3> getFaceVertices(const Face& f) const {

		std::array<size_t, 3> v;

		const HalfEdge* he = &halfEdges[f.he];
		v[0] = he->vert;

		he = &halfEdges[he->next];
		v[1] = he->vert;

		he = &halfEdges[he->next];
		v[2] = he->vert;

		return v;

	}

	std::array<size_t, 3> getFaceHalfEdges(const Face& f) const {

		return { f.he, halfEdges[f.he].next, halfEdges[halfEdges[f.he].next].next };
	}

	std::array<size_t, 2> getHalfEdgeVertices(const HalfEdge& he) const {

		return { halfEdges[he.twin].vert, he.vert };

	}


	void debugState() const
	{
		std::cout << "=============================" << std::endl;
		std::cout << "=== HalfEdgeMesh Debug Info ===" << std::endl;
		std::cout << "vertexData.size(): " << vertices.size() << std::endl;
		std::cout << "mesh.faces.size(): " << faces.size() << std::endl;
		std::cout << "mesh.halfEdges.size() : " << halfEdges.size() << std::endl;
		std::cout << "============================" << std::endl;
	}
};
#endif
