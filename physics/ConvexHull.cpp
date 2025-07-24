#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#include <vector>
#include <memory>
#include <cassert>

#include "../inc/Mesh.hpp"
#include "../inc/Model.hpp"
#include "../inc/Ray.hpp"
#include "ConvexHull.hpp"
#include "HalfEdgeMesh.hpp"
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

/* DOING */
void ConvexHull::getConvexHull(const std::vector<glm::vec3> &pointCloud, bool CCW, bool useOriginalIndices)
{

	std::vector<bool> faceProcessed(mesh.faces.size(), false);
	std::vector<size_t> faceStack;
	std::unordered_map<size_t, size_t> mapVertexIndex;

	for (size_t i = 0; i < mesh.faces.size(); i++)
	{

		if (!mesh.faces[i].isDisabled())
		{

			faceStack.push_back(i);
			break;
		}
	}
	if (faceStack.size() == 0)
	{
		return;
	}

	const size_t isCCW = CCW ? 1 : 0;
	const size_t faceCount;
	indices.reserve(faceCount * 3);

	while (faceStack.size())
	{

		auto it = faceStack.end() - 1;
		size_t topIdx = *it;
		assert(!mesh.faces[topIdx].isDisabled());
		faceStack.erase(it);

		if (faceProcessed[topIdx])
		{

			continue;
		}
		else
		{

			faceProcessed[topIdx] = true;
			auto he = mesh.getFaceHalfEdges(mesh.faces[topIdx]);

			// Push neighboring faces onto stack
			size_t adjacentFaces[] = {mesh.halfEdges[mesh.halfEdges[he[0]].twin].face, mesh.halfEdges[mesh.halfEdges[he[1]].twin].face, mesh.halfEdges[mesh.halfEdges[he[2]].twin].face};
			for (auto f : adjacentFaces)
			{
				if (!faceProcessed[f] && !mesh.faces[f].isDisabled())
				{
					faceStack.push_back(f);
				}
			}

			// Process face vertices
			auto vertices = mesh.getFaceVertices(mesh.faces[topIdx]);
			if (!useOriginalIndices)
			{
				for (auto &v : vertices)
				{
					auto it = mapVertexIndex.find(v);
					if (it == mapVertexIndex.end())
					{

						// WRONG LOGIC???
						// vertices stored as std::vector<Vertex> vertices ???s
						optimizedVBO->push_back(pointCloud[v]);
						mapVertexIndex[v] = optimizedVBO->size() - 1;
						v = optimizedVBO->size() - 1;
					}
					else
					{
						v = it->second;
					}
				}
			}
			indices.push_back(vertices[0]);
			indices.push_back(vertices[1 + iCCW]);
			indices.push_back(vertices[2 - iCCW]);
		}
	}
}

void ConvexHull::buildMesh(const std::vector<glm::vec3> &pointCloud, float defaultEps)
{

	if (pointCloud.size() == 0)
	{

		mesh = HalfEdgeMesh();
		return;
	}
	vertexData = pointCloud;

	// Find extreme values and use them to compute scale of point cloud
	extremaIndices = getExtrema();
	scale = getScale();

	// Epsilon determines tolerance of "fat planes"
	epsilon = defaultEps * scale;
	epsilonSquared = epsilon * epsilon;

	// Planar case -- when all points lie on a 2D subspace of R^3
	isPlanar = false;

	createConvexHalfEdgeMesh(); // Iteratively update mesh until ...
	if (isPlanar)
	{
	}
}

/*DOING*/
// Forms inital hull from extreme values
void ConvexHull::setupInitialTetrahedron()
{

	const size_t vertexCount = vertexData.size();

	// Degenerate case -- just return degenerate tetrahedron
	if (vertexCount <= 4)
	{
		size_t v[4] = {0, std::min((size_t)1, vertexCount - 1), std::min((size_t)2, vertexCount - 1), std::min((size_t)3, vertexCount - 1)};

		const glm::vec3 normal = getTriangleNormal(vertexData[v[0]], vertexData[v[1]], vertexData[v[2]]);
		const Plane trianglePlane(normal, vertexData[v[0]]);

		// normal should point outwards away from tetrahedrons
		if (trianglePlane.isPointAbovePlane(vertexData[v[3]]))
		{
			std::swap(v[0].v[1]);
		}

		return mesh.setup(v[0], v[1], v[2], v[3]);
	}

	// 1. Form a line between two furthest extrema vertices
	float maxDist = epsilonSquared;
	std::pair<size_t, size_t> selectedPoints;

	for (int i = 0; i < 6; i++)
	{
		for (int j = i + 1; j < 6; j++)
		{

			const float dist = glm::distance2(vertexData[extremaIndices[i]], vertexData[extremaIndices[j]);	// get squared distance between extreme values

			if (dist > maxDist)
			{
				maxDist = dist;
				selectedPoints = {extremaIndices[i], extremaIndices[j]};
			}
		}
	}

	// Degenerate case -- point cloud seems to consist of a single point
	if (maxDist == epsilonSquared)
	{

		return mesh.setup(0, std::min((size_t)1, vertexCount - 1), std::min((size_t)2, vertexCount - 1), std::min((size_t)3, vertexCount - 1));
	}
	assert(selectedPoints.first != selectedPoints.second);

	// 2. Find point furthest from the line (forms a triangle face)
	maxDist = epsilonSquared;
	size_t maxIdx = std::numeric_limits<size_t>::max();

	const Ray r(vertexData[selectedPoints.first], (vertexData[selectedPoints.second] - vertexData[selectedPoints.first]));

	for (int i = 0; i < vertexCount; i++)
	{

		const float distToRay = getSquaredDistanceToRay(vertexData[i], r);

		if (distToRay > maxDist)
		{
			maxDist = distToRay;
			maxIdx = i;
		}
	}

	// Degenerate case -- point cloud seems to belong to 1D subspace of R^3
	if (maxDist == epsilonSquared)
	{

		// Pick any distinct point and return a thin triangle
		auto it = std::find_if(m_vertexData.begin(), m_vertexData.end(), [&](const vec3 &ve)
							   { return ve != m_vertexData[selectedPoints.first] && ve != m_vertexData[selectedPoints.second]; });

		const size_t thirdPoint = (it == m_vertexData.end()) ? selectedPoints.first : std::distance(m_vertexData.begin(), it);

		it = std::find_if(m_vertexData.begin(), m_vertexData.end(), [&](const vec3 &ve)
						  { return ve != m_vertexData[selectedPoints.first] && ve != m_vertexData[selectedPoints.second] && ve != m_vertexData[thirdPoint]; });

		const size_t fourthPoint = (it == m_vertexData.end()) ? selectedPoints.first : std::distance(m_vertexData.begin(), it);
		return m_mesh.setup(selectedPoints.first, selectedPoints.second, thirdPoint, fourthPoint);

		return mesh.setup(selectedPoints.first, selectedPoints.second, thirdPoint, fourthPoint);
	}
	assert(maxIdx != selectedPoints.first && maxIdx != selectedPoints.second);

	// Forms base triangle for tetrahedron
	std::array<glm::vec3, 3> baseTriangle{
		vertexData[selectedPoints.first],
		vertexData[selectedPoints.second],
		vertexData[maxIdx]};

	std::array<size_t, 3> baseTriangleIndices{
		selectedPoints.first,
		selectedPoints.second,
		maxIdx};

	// 3. Find point furthest from the plane (forms a tetrahedron)
	maxDist = epsilonSquared;
	maxIdx = 0;

	const glm::vec3 normal = getTriangleNormal(baseTriangle[0], baseTriangle[1], baseTriangle[2]);
	const Plane p(normal, baseTriangle[0]);
	for (int i = 0; i < vertexCount; i++)
	{

		const float distToPlane = std::abs(getSignedDistanceToPlane(vertexData[i], p));
		if (distToPlane > maxDist)
		{

			maxDist = distToPlane;
			maxIdx = i;
		}
	}

	// Degenerate case -- point cloud seems to lie on 2D subspace of R^3
	if (maxDist == epsilonSquared)
	{

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
		const glm::vec3 &va = vertexData[vFace[0]];
		const glm::vec3 &vb = vertexData[vFace[1]];
		const glm::vec3 &vc = vertexData[vFace[2]];

		const glm::vec3 vNormal = getTriangleNormal(va, vb, vc);
		const Plane P(vNormal, va);

		face.P = P;
		
	}

	// 4. Assign points to each face if "outside" (vertices inside tetrahedron are ignored)
	for (int i = 0; i < vertexCount; i++) {
		for (auto &f : mesh.faces)
		{

			if (addPointToFace(f, i))
			{
				break;
			}
		}
	}
}

// *DOING*
void ConvexHull::createConvexHalfEdgeMesh()
{

	visibleFaces.clear();
	horizonEdges.clear();
	possibleVisibleFaces.clear();

	// Compute initial hull
	setupInitialTetrahedron();
	assert(mesh.faces.size() == 4);

	// Initialize face stack
	faceStack.clear();
	for (int i = 0; i < 4; i++)
	{

		auto &f = mesh.faces[i];

		// Ensure face has a populated conflict list  before pushing onto stack
		if (f.pointsOnPositiveSide && f.pointsOnPositiveSide->size() > 0)
		{

			faceStack.push_back(i);
			f.inFaceStack = 1; /*WTF is this a uint_8*/
		}
	}

	// Iterate over face stack until no "outside" points exist
	size_t it = 0;
	while (!faceStack.empty())
	{
		it++;
		if (it == std : numeric_limits<size_t>::max())
		{

			it = 0;
		}

		// Pop top face off stack
		const size_t topIdx = faceStack.front();
		faceStack.pop_front();
		auto &topFace = mesh.faces[topIdx];
		topFace.inFaceStack = 0;

		assert(!topFace.pointsOnPositiveSide || topFace.pointsOnPositiveSide->size() > 0);

		// Ignore faces with empty conflict lists or disabled faces
		if (!topFace.pointsOnPositiveSide || topFace.isDisabled)
		{

			continue;
		}

		// Choose furthest point (eye point) as new vertex in convex hull
		const size_t eyePointIdx = topFace.furthestPoint;
		const glm::vec3 &eyePoint = vertexData[topFace.furthestPoint];

		horizonEdges.clear();
		possibleVisibleFaces.clear();
		visibleFaces.clear();

		// Find all faces visible to eye point
		// Build a list of horizon edges
		possibleVisibleFaces.emplace_back(topFace, std::numeric_limits<size_t>::max());
		while (possibleVisibleFaces.size())
		{

			const auto faceData = possibleVisibleFaces.back();
			possibleVisibleFaces.pop_back();

			auto &testFace = mesh.faces[faceData.faceIndex];
			assert(!testFace.isDisabled());

			// Check if testFace is already visible
			if (testFace.visibilityCheckedOnIteration == it)
			{

				if (testFace.isVisibleFaceOnCurrentIteration)
				{

					continue;
				}
			}

			// Test visibility
			else
			{

				const Plane &P = testFace.P;
				testFace.visibilityCheckedOnIteration = it;
				const float dist = getSignedDistanceToPlane(eyePoint, P);

				// Ensure point is outside plane
				if (dist > 0)
				{

					testFace.isVisibleFaceOnCurrentIteration = 1;
					testFace.horizonEdgesOnCurrentIteration = 0;

					// add half edges if face is visible
					for (auto he : mesh.getFaceHalfEdges(testFace))
					{

						if (mesh.halfEdges[he].twin != faceData.enteredFromHalfEdge)
						{

							possibleVisibleFaces.emplace_back(mesh.halfEdges[mesh.halfEdges[he].twin].face, he);
						}
					}

					continue;
				}

				assert(faceData.faceIndex != topIdx);
			}

			// Face is not visible, thus half edge is part of the horizon edge
			testFace.isVisibleFaceOnCurrentIteration = 0;
			horizonEdges.push_back(faceData.enteredFromHalfEdge);

			// Get all half edges of non-visible face
			const auto halfEdges = mesh.getFaceHalfEdges(mesh.faces[mesh.halfEdges[faceData.enteredFromHalfEdge].face]);

			// Determine index of horizon half edge
			const std::int8_t heIdx =
				(halfEdges[0] == faceData.enteredFromHalfEdge)	 ? 0
				: (halfEdges[1] == faceData.enteredFromHalfEdge) ? 1
																 : 2;

			// Bitmask for horizon half edge, discard other half edges of non-visible face
			mesh.faces[mesh.halfEdges[faceData.enteredFromHalfEdge].face].horizonEdgesOnCurrentIteration |= (1 << heIdx);
		}

		const size_t horizonEdgesCount = horizonEdges.size();

		// Attempt to form loop between horizon edges
		if (!connectHorizonEdges(horizonEdges))
		{

			std::cerr << "Failed to solve horizon edge." << std::endl;

			// Eye point is invalid and we don't add to convex hull
			auto it = std::find(topFace.pointsOnPositiveSide->begin(),
								topFace.pointsOnPositiveSide->end(),
								eyePointIdx);

			// Erase eye point from future iterations
			topFace.pointsOnPositiveSide->erase(it);
			if (topFace.pointsOnPositiveSide->size() == 0)
			{

				reclaimConflictList(topFace.pointsOnPositiveSide);
			}

			continue;
		}

		// Disable all visible faces & half-edges, we resuse their
		for (auto faceIdx : visibleFaces)
		{

			auto &disabledFace = mesh.faces[faceIdx];
			auto halfEdges = mesh.getFaceHalfEdges(disabledFace);

			for (size_t j = 0; j < 3; j++)
			{

				// disable all half edges except horizon half edge
				if ((disabledFace.horizonEdgesOnCurrentIteration & (1 << j)) == 0)
				{
					if ()
					{
					}
					else
					{

						mesh.disableHalfEdge(halfEdges[j]);
					}
				}
			}

			// Dsiable face but ..
		}

		// Build new faces with horizon edges
		for (size_t i = 0; i < horizonEdgesCount; i++)
		{

			// Existing half edge
			const size_t AB = horizonEdges[i];

			// Triangle vertices
			auto horizonEdgeVertices = mesh.getHalfEdgeVertices(mesh.halfEdges[AB]);
			size_t A, B, C;

			A = horizonEdgeVertices[0];
			B = horizonEdgeVertices[1];
			C = eyePointIdx;

			const size_t newFaceIdx = mesh.addFace();
			newFaces.push_back(newFaceIdx);

			// New half edges
			const size_t CA = newHalfEdges[2 * i + 0];
			const size_t BC = newHalfEdges[2 * i + 1];

			mesh.halfEdges[CA].vert = A;
			mesh.halfEdges[BC].vert = C;

			mesh.halfEdges[CA].twin = newHalfEdges[i > 0 ? i * 2 - 1 : 2 * horizonEdgesCount - 1];
			mesh.halfEdges[BC].twin = newHalfEdges[((i + 1) * 2) % (horizonEdgesCount * 2)];

			mesh.halfEdges[BC].face = newFaceIdx;
			mesh.halfEdges[CA].face = newFaceIdx;
			mesh.halfEdges[AB].face = newFaceIdx;

			mesh.halfEdges[AB].next = BC;
			mesh.halfEdges[BC].next = CA;
			mesh.halfEdges[CA].next = AB;

			// New face
			auto &newFace = mesh.faces[newFaceIdx];

			const glm::vec3 planeNormal = getTriangleNormal(vertexData[A], vertexData[B], eyePoint);
			newFace.P = Plane(planeNormal, eyePoint);
			newFace.he = AB;
		}

		// Assign points
		for (auto &conflictList : disabledFaceConflictLists)
		{

			assert(conflictList);
			for (const auto &point : *(conflictList))
			{

				// Don't assign eyePoint (part of convex hull now)
				if (point == eyePointIdx)
				{
					continue;
				}

				//
				for (size_t j = 0; j < horizonEdgesCount; j++)
				{
					if (addPointToFace(mesh.faces[newFaces[j]], point))
					{

						// skip to next point if successfully added to conflictList
						break;
					}
				}
			}
			//
			reclaimConflictList(conflictList);
		}

		// Push new faces onto faceStack
		for (const auto newFaceIdx : newFaces)
		{

			auto &newFace = mesh.faces[newFaceIdx];
			if (newFace.pointsOnPositiveSide)
			{

				assert(newFace.pointsOnPositiveSide->size() > 0);
				if (!newFace.inFaceStack)
				{

					faceStack.push_back(newFaceIdx);
					newFace.inFaceStack = 1;
				}
			}
		}
	}

	conflictListsPool.clear();
}

/*DONE*/
// Check if horizon edges form connected loop
bool ConvexHull::connectHorizonEdges(std::vector<size_t> &horizonEdges)
{

	const size_t horizonEdgeCount = horizonEdges.size();
	for (size_t i = 0; i < horizonEdgeCount - 1; i++)
	{

		// inital end vertex
		const size_t endVertex = mesh.halfEdges[horizonEdges[i]].vert;

		bool foundNext = false;
		for (size_t j = i + 1; i < horizonEdgeCount; j++)
		{

			// end vertex for twin edge (pointing in opposite direction to current edge)
			const size_t beginVertex = mesh.halfEdges[mesh.halfEdges[horizonEdges[j]].twin].vert;

			// edges are connected
			if (beginVertex == endVertex)
			{

				// reorder s.t. horizonEdges are in sequence
				std::swap(horizonEdges[i + 1], horizonEdges[j]);
				foundNext = true;
				break;
			}
		}
		if (!foundNext)
		{
			return false;
		}
	}

	// final vertex must match initial vertex
	assert(mesh.halfEdges[horizonEdges[horizonEdgeCount - 1]].vert == mesh.halfEdges[mesh.halfEdges[horizonEdges[0]].twin].vert);
	return true;
	s
}

/*DONE*/
// Returns indices to extreme values
std::array<size_t, 6> ConvexHull::getExtrema()
{

	std::array<size_t, 6> outIndices{0, 0, 0, 0, 0, 0};
	extremeValues{vertexData[0].x, vertexData[0].x, vertexData[0].y, vertexData[0].y, vertexData[0].z, vertexData[0].z};

	for (int i = 0; i < vertexData.size(); i++)
	{

		const glm::vec3 &pos = vertexData[i];

		// X-axis
		if (pos.x < extremeValues[0])
		{
			extremeValues[0] = pos.x;
			outIndices[0] = i;
		}

		else if (pos.x > extremeValues[1])
		{
			extremeValues[1] = pos.x;
			outIndices[1] = i;
		}

		// Y-axis
		if (pos.y < extremeValues[2])
		{
			extremeValues[2] = pos.y;
			outIndices[2] = i;
		}

		else if (pos.y > extremeValues[3])
		{
			extremeValues[3] = pos.y;
			outIndices[3] = i;
		}

		// Z-axis
		if (pos.z < extremeValues[4])
		{
			extremeValues[4] = pos.z;
			outIndices[4] = i;
		}

		else if (pos.z > extremeValues[5])
		{
			extremeValues[5] = pos.z;
			outIndices[5] = i;
		}
	}

	return outIndices;
}

/*DONE*/
// Returns scale for computing epsilon
float ConvexHull::getScale()
{

	float s = 0.0f;
	for (int i = 0; i < 6; i++)
	{

		const float *v = (const float *)(&vertexData[extremaIndices[i]]);
		v += i / 2;

		s = std::max(s, std::abs(*v));
	}

	return s;
}

/*DONE*/
// Adds point if above face plane and above epsilon tolerance
bool ConvexHull::addPointToFace(HalfEdgeMesh::Face &face, size_t pointIdx)
{

	// Negative dist means point is inside the hull
	const float distToPlane = getSignedDistanceToPlane(vertexData[pointIdx], face.P);

	// lies above plane and above epsilon tolerance -- |dist| is greater than epsilon * ||normal||
	if (distToPlane > 0 && distToPlane * distToPlane > epsilonSquared * face.P.normLengthSq)
	{

		if (!face.pointsOnPositiveSide)
		{

			// Reuse old conflict list if face has none
			face.pointsOnPositiveSide = std::move(getConflictList());
		}

		// Push new point to conflict list
		face.pointsOnPositiveSide->push_back(pointIdx);

		if (distToPlane > face.furthestPointDist)
		{

			face.furthestPointDist = distToPlane;
			face.furthestPoint = pointIdx;
		}

		return true;
	}

	return false;
}

/*DONE*/
// Reuse discarded conflict list memory
std::unique_ptr<std::vector<size_t>> ConvexHull::getConflictList()
{

	// Allocate new memory if pool is empty
	if (conflictListsPool.size() == 0)
	{
		return std::unique_ptr<std::vector<size_t>>(new std::vector<size_t>());
	}

	auto it = conflictListsPool.end() - 1;

	std::unique_ptr<std::vector<size_t>> conflictListPtr = std::move(*it);
	conflictListsPool.erase(it);

	conflictListPtr->clear();
	return conflictListPtr;
}

/*DONE*/
// Recycle conflict list memory to pool when face is removed
void ConvexHull::reclaimConflictList(std::unique_ptr<std::vector<size_t>> &ptr)
{

	const size_t ize = ptr->size();

	//
	if ((size + 1) * 128 < ptr->capacity())
	{

		ptr.reset(nullptr);
		return;
	}

	conflictListsPool.push_back(std::move(ptr));
}

/*DOING*/
void ConvexHull::writeOBJ(const std::string &fileName, const std::string &objectName) const
{

	std::ofstream objFile;
	objFile.open(fileName);
	objFile << "o " << objectName << "\n";
	for (const auto &v : getConvexHullVertices())
	{

		// WRONG LOGIC
		// vertices stored as std::vector<Vertex>vertices ???
		objFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
	}
	const auto &indBuf = getConvexHullIndices();
	size_t triangleCount = indBuf.size() / 3;
	for (size_t i = 0; i < triangleCount; i++)
	{
		objFile << "f " << indBuf[i * 3] + 1 << " " << indBuf[i * 3 + 1] + 1 << " " << indBuf[i * 3 + 2] + 1 << "\n";
	}
	objFile.close();
}

/* DONE */
std::vector<Vertex> ConvexHull::getVertices()
{

	return vertices;
}

/* DONE */
std::vector<int> ConvexHull::getIndices()
{

	return indices;
}
