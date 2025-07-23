#ifndef HALFEDGEMESH_HPP
#define HALFEDGEMESH_HPP

#include <glm/glm.hpp>

#include <vector>
#include <limits>
#include <array>
#include <cassert>
#include <memory>
#include <cinttypes>

#include "../inc/Plane.hpp"

// Container for half-edge representation of polygon mesh
// Makes iterating over edges & faces easier
class HalfEdgeMesh {
public:

	struct HalfEdge {

		size_t vert;	// end vertex the half-edge points to
		size_t twin;	// oppositely oriented half-edge on adjacent face
		size_t face;	// index of incident face
		size_t next;	// points to next edge on incident face

		void disable() {
			vert = std::numeric_limits<size_t>::max();
		}

		bool isDisabled() const {
			return vert == std::numeric_limits<size_t>::max();
		}
	};

	struct Face {
		size_t he;		// index of any one of its three half-edges
		Plane P;

		size_t furthestPoint;
		float furthestPointDist;


		size_t visibilityCheckedOnIteration;


		/* WTF IS THIS SHIT*/

		// Use uint8_t instead of boolean
		std::uint8_t isVisibleFaceOnCurrentIteration : 1;
		std::uint8_t inFaceStack : 1;
		std::uint8_t horizonEdgesOnCurrentIteration : 3; // Bit for each half edge assigned to this face, each being 0 or 1 depending on whether the edge belongs to horizon edge
		std::unique_ptr<std::vector<size_t>> pointsOnPositiveSide;

		Face() : he(std::numeric_limits<size_t>::max()),
			furthestPoint(0),
			furthestPointDist(0),
			visibilityCheckedOnIteration(0),
			isVisibleFaceOnCurrentIteration(0),
			inFaceStack(0),
			horizonEdgesOnCurrentIteration(0)
		{

		}

		void disable() {
			he = std::numeric_limits<size_t>::max();
		}

		bool isDisabled() const {
			return he == std::numeric_limits<size_t>::max();
		}
	};


	// Mesh data
	std::vector<Face> faces;
	std::vector<HalfEdge> halfEdges;


	// When mesh is modified and faces + half edges are deleted, we simply mark as disabled for potential future usage
	std::vector<size_t> disabledFaces, disabledHalfEdges;

	size_t addFace() {
		if (disabledFaces.size()) {
			size_t idx = disabledFaces.back();

			Face& f = faces[idx];
			assert(f.isDisabled());
			assert(!f.pointsOnPositiveSide);

			f.furthestPointDist = 0.0f;


			disabledFaces.pop_back();
			return idx;

		}

		faces.emplace_back();
		return faces.size() - 1;
	}

	size_t addHalfEdge() {
		if (disabledHalfEdges.size()) {
			size_t idx = disabledHalfEdges.back();
			disabledHalfEdges.pop_back();
			return idx;
		}

		halfEdges.emplace_back();
		return halfEdges.size() - 1;
	}

	// Return pointer to all vertices in "outside" set
	std::unique_ptr<std::vector<size_t>> disableFace(size_t faceIdx) {

		Face& f = faces[faceIdx];
		f.disable();
		disabledFaces.push_back(faceIdx);
		return std::move(f.pointsOnPositiveSide);
	}

	void disableHalfEdge(size_t heIdx) {

		HalfEdge& he = halfEdges[heIdx];
		he.disable();
		disabledHalfEdges.push_back(heIdx);
	}

	HalfEdgeMesh() = default;

	// Build inital tetrahedron from vertex indices
	size_t setup(size_t a, size_t b, size_t c, size_t d) {

		faces.clear();
		halfEdges.clear();
		disabledFaces.clear();
		disabledHalfEdges.clear();

		// tetrahedron has 4 faces & 6 edges
		faces.reserve(4);
		halfEdges.reserve(12);

		// create initial faces
		// --------------------
		Face ABC;
		ABC.he = 0;
		faces.push_back(std::move(ABC));

		Face ACD;
		ACD.he = 3;
		faces.push_back(std::move(ACD));

		Face BAD;
		BAD.he = 6;
		faces.push_back(std::move(BAD));

		Face CBD;
		CBD.he = 9;
		faces.push_back(std::move(CBD));

		// create initial halfedges
		// ------------------------
		// face ABC
		HalfEdge AB;
		AB.vert = b;
		AB.twin = 6;
		AB.face = 0;
		AB.next = 1;
		halfEdges.push_back(AB); // halfEdges[0]

		HalfEdge BC;
		BC.vert = c;
		BC.twin = 9;
		BC.face = 0;
		BC.next = 2;
		halfEdges.push_back(BC); // halfEdges[1]

		HalfEdge CA;
		CA.vert = a;
		CA.twin = 3;
		CA.face = 0;
		CA.next = 0;
		halfEdges.push_back(CA); // halfEdges[2]


		// face ACD
		HalfEdge AC;
		AC.vert = c;
		AC.twin = 2;
		AC.face = 1;
		AC.next = 4;
		halfEdges.push_back(AC); // halfEdges[3]

		HalfEdge CD;
		CD.vert = d;
		CD.twin = 11;
		CD.face = 1;
		CD.next = 5;
		halfEdges.push_back(CD); // halfEdges[4]

		HalfEdge DA;
		DA.vert = a;
		DA.twin = 7;
		DA.face = 1;
		DA.next = 3;
		halfEdges.push_back(DA); // halfEdges[5]


		// face BAD
		HalfEdge BA;
		BA.vert = a;
		BA.twin = 0;
		BA.face = 2;
		BA.next = 7;
		halfEdges.push_back(BA); // halfEdges[6]

		HalfEdge AD;
		AD.vert = d;
		AD.twin = 5;
		AD.face = 2;
		AD.next = 8;
		halfEdges.push_back(AD); // halfEdges[7]

		HalfEdge DB;
		DB.vert = b;
		DB.twin = 10;
		DB.face = 2;
		DB.next = 6;
		halfEdges.push_back(DB); // halfEdges[8]


		// face CBD
		HalfEdge CB;
		CB.vert = b;
		CB.twin = 1;
		CB.face = 3;
		CB.next = 10;
		halfEdges.push_back(CB); // halfEdges[9]

		HalfEdge BD;
		BD.vert = d;
		BD.twin = 8;
		BD.face = 3;
		BD.next = 11;
		halfEdges.push_back(BD); // halfEdges[10]

		HalfEdge DC;
		DC.vert = c;
		DC.twin = 4;
		DC.face = 3;
		DC.next = 9;
		halfEdges.push_back(DC); // halfEdges[11]
		
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

		return { he.vert, halfEdges[he.twin].vert };

	}
};
