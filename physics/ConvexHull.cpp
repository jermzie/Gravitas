#include "ConvexHull.hpp"
#include <set>
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

ConvexHull::ConvexHull(const Model &objectModel, WorldTransform objectTrans){

	// Get verticess
	vertexData = objectModel.GetVertexData();
	
	// Build half-edge mesh structure
	buildMesh(vertexData);

	// Build convex hull vertices & indices
	getConvexHull(vertexData);

	// Compute convex hull centroid for rendering & transformations
	localCentroid = computeCentroid();
	glm::vec4 objectWorldTrans = objectTrans.GetMatrix() * glm::vec4(localCentroid, 1.0f);
	worldCentroid = glm::vec3(objectWorldTrans);

	std::string objectName = objectModel.fileName;
	size_t dotPos = objectName.find_last_of(".");
	std::string name = objectName.substr(0, dotPos);
	std::string ext = objectName.substr(dotPos);
	convexhullName = name + "_convexhull" + ext;

	writeOBJ(convexhullName, name);
	//convexhullModel.LoadModel(modelName);
}

void ConvexHull::computeConvexHull(const Model &objectModel, WorldTransform objectTrans){

	std::cout << "============================" << std::endl;
	std::cout << objectModel.fileName << std::endl;
	// Get vertices
	std::vector<glm::vec3> modelVertices = objectModel.GetVertexData();
	std::set<glm::vec3, Vec3Compare> uniqueVert(modelVertices.begin(), modelVertices.end());
	vertexData.assign(uniqueVert.begin(), uniqueVert.end());

	std::cout << "Unique Model Vertices \n";
	debugPrintUniqueVertices(vertexData);

	// Build convex hull as half-edge mesh structure
	buildMesh(vertexData, 1e-8f);

	// Build convex hull vertices & indices
	getConvexHull(vertexData);

	std::cout << objectModel.fileName << std::endl;
	debugPrintState();

	std::cout << "Convex Hull Vertices \n";
	debugPrintUniqueVertices(vertices);

	// Compute convex hull centroid for rendering & transformations
	localCentroid = computeCentroid();
	glm::vec4 objectWorldTrans = objectTrans.GetMatrix() * glm::vec4(localCentroid, 1.0f);
	worldCentroid = glm::vec3(objectWorldTrans);

	std::string objectName = objectModel.fileName;
	size_t dotPos = objectName.find_last_of(".");
	std::string name = objectName.substr(0, dotPos);
	std::string ext = objectName.substr(dotPos);
	convexhullName = name + "_convexhull" + ext;

	writeOBJ(convexhullName, name);
	//convexhullModel.LoadModel(modelName);
}


glm::vec3 ConvexHull::computeCentroid() {

	unsigned int num = 0;
	glm::vec3 centroid(0.0f, 0.0f, 0.0f);

	for(auto& v : vertices){
		
		centroid += v;
		num++;
	}

	return centroid /= static_cast<float>(num);
}

/* TODO */
bool ConvexHull::computeRayIntersection(const Ray &r, float &t){
	
	/*
	 GENERAL IDEA
	 go through all faces (non-diabled) and test ray-plane intersections lol
	*/
	for(auto& f : mesh.faces) {
		
		if(!f.isDisabled()){

	        	// Calculate ray-plane intersection
        		float denom = glm::dot(f.P.normal, r.direction);
	
		        // Check if ray is not parallel to plane -- avoid zero division
				if (std::abs(denom) < 1e-6) {
					return false;
				}


	        	t = glm::dot(f.P.normal, f.P.point - r.origin) / denom;

			//return t >= 0;

		
			// min max intersect distances
			return (t >= 0.1f && t < 50.0f);
		}
	}

	return false;
}

void ConvexHull::updateCentroid(glm::vec3 displacement){

	/*
	worldTrans = objectTrans;

	// Apply the object's transformation to the centroid
	glm::vec4 worldCentroidTrans = objectTrans.GetMatrix() * glm::vec4(localCentroid, 1.0f);
	worldCentroid = glm::vec3(worldCentroidTrans);
	*/

	worldCentroid += displacement;
}

/* DEBBUG */
void ConvexHull::getConvexHull(const std::vector<glm::vec3> &pointCloud, bool CCW, bool useOriginalIndices)
{
	
	if(!useOriginalIndices) {
		optimizedVBO.reset(new std::vector<glm::vec3>());
	}
	
	std::vector<bool> faceProcessed(mesh.faces.size(), false);
	std::vector<size_t> faceStack;
	std::unordered_map<size_t, size_t> mapVertex2Index;

	indices.clear();

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
	const size_t faceCount = mesh.faces.size() - mesh.disabledFaces.size();
	indices.reserve(faceCount * 3);

	while (faceStack.size())
	{

		auto itf = faceStack.end() - 1;
		size_t topIdx = *itf;
		assert(!mesh.faces[topIdx].isDisabled());
		faceStack.erase(itf);

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
			auto faceVertices = mesh.getFaceVertices(mesh.faces[topIdx]);
			if (!useOriginalIndices)
			{
				for (auto &v : faceVertices)
				{
					auto itv = mapVertex2Index.find(v);
					if (itv == mapVertex2Index.end())
					{

						optimizedVBO->push_back(pointCloud[v]);
						mapVertex2Index[v] = optimizedVBO->size() - 1;
						v = optimizedVBO->size() - 1;

					}
					else
					{
						v = itv->second;
					}
				}
			}

			if (faceVertices[0] != faceVertices[1 + isCCW] &&
				faceVertices[0] != faceVertices[2 - isCCW] &&
				faceVertices[1 + isCCW] != faceVertices[2 - isCCW]) {

				indices.push_back(faceVertices[0]);
				indices.push_back(faceVertices[1 + isCCW]);
				indices.push_back(faceVertices[2 - isCCW]);
			}

			//indices.push_back(faceVertices[0]);
			//indices.push_back(faceVertices[1 + isCCW]);
			//indices.push_back(faceVertices[2 - isCCW]);
		}
	}

	if(!useOriginalIndices){
		vertices = std::vector<glm::vec3>(*optimizedVBO);
	}
}


/* DONE */
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

	// Epsilon and scale determines tolerance of "fat planes"
	epsilon = defaultEps * scale;
	epsilonSquared = epsilon * epsilon;

	// Planar case -- when all points lie on a 2D subspace of R^3
	isPlanar = false;

	createConvexHalfEdgeMesh(); // Iteratively update mesh until ...

	// Points seem to lie on a 2D subspace of R^3
	if (isPlanar)
	{
		const size_t newPointIdx = tempPlanarVertices.size() - 1;
		for(auto& he : mesh.halfEdges){
			if(he.vert == newPointIdx) {
				
				he.vert = 0;
			}
		}

		vertexData = pointCloud;
		tempPlanarVertices.clear();
	}
}

/* DOING */
// Forms inital hull from extreme values
void ConvexHull::setupInitialTetrahedron()
{

	const size_t vertexCount = vertexData.size();

	if (vertexCount == 0) {
		std::cerr << "Error: No vertices to process" << std::endl;
		return;
	}

	// 0. Degenerate case -- just return degenerate tetrahedron
	if (vertexCount <= 4)
	{
		size_t v[4] = {0, std::min((size_t)1, vertexCount - 1), std::min((size_t)2, vertexCount - 1), std::min((size_t)3, vertexCount - 1)};
		const glm::vec3 normal = getTriangleNormal(vertexData[v[0]], vertexData[v[1]], vertexData[v[2]]);
		const Plane trianglePlane(normal, vertexData[v[0]]);

		// normal should point outwards away from tetrahedrons
		if (trianglePlane.isPointAbovePlane(vertexData[v[3]]))
		{
			std::swap(v[0], v[1]);
		}

		return mesh.setup(v[0], v[1], v[2], v[3]);
	}

	// 1a. Form a line between two furthest extrema vertices
	float maxDist = epsilonSquared;
	std::pair<size_t, size_t> selectedPoints;

	for (int i = 0; i < 6; i++)
	{
		// Check if extremaIndices[i] is valid
		if (extremaIndices[i] >= vertexCount) {
			std::cerr << "Error: extremaIndices[" << i << "] = " << extremaIndices[i]
				<< " is out of range for vertexData size " << vertexCount << std::endl;
			continue;
		}

		for (int j = i + 1; j < 6; j++)
		{

			// Check if extremaIndices[j] is valid
			if (extremaIndices[j] >= vertexCount) {
				std::cerr << "Error: extremaIndices[" << j << "] = " << extremaIndices[j]
					<< " is out of range for vertexData size " << vertexCount << std::endl;
				continue;
			}

			const float dist = glm::distance2(vertexData[extremaIndices[i]], vertexData[extremaIndices[j]]);	// get squared distance between extreme values

			if (dist > maxDist)
			{
				maxDist = dist;
				selectedPoints = {extremaIndices[i], extremaIndices[j]};
			}
		}
	}

	// 1b. Degenerate case -- point cloud seems to consist of a single point
	if (maxDist == epsilonSquared)
	{
		std::cerr << "1b. error \n";
		return mesh.setup(0, std::min((size_t)1, vertexCount - 1), std::min((size_t)2, vertexCount - 1), std::min((size_t)3, vertexCount - 1));
	}
	assert(selectedPoints.first != selectedPoints.second);

	// 2a. Find point furthest from the line -- forms a triangle face
	maxDist = epsilonSquared;
	size_t maxIdx = std::numeric_limits<size_t>::max();

	Ray r(vertexData[selectedPoints.first], (vertexData[selectedPoints.second] - vertexData[selectedPoints.first]));

	for (int i = 0; i < vertexCount; i++)
	{

		const float distToRay = getSquaredDistanceToRay(vertexData[i], r);

		if (distToRay > maxDist)
		{
			maxDist = distToRay;
			maxIdx = i;
		}
	}

	// 2b. Degenerate case -- point cloud seems to belong to 1D subspace of R^3
	if (maxDist == epsilonSquared)
	{
		std::cerr << "2b. error \n";
		// Pick any distinct point and return a thin triangle
		auto it = std::find_if(vertexData.begin(), vertexData.end(), [&](const glm::vec3 &ve)
							   { return ve != vertexData[selectedPoints.first] && ve != vertexData[selectedPoints.second]; });
		
		const size_t thirdPoint = (it == vertexData.end()) ? selectedPoints.first : std::distance(vertexData.begin(), it);

		it = std::find_if(vertexData.begin(), vertexData.end(), [&](const glm::vec3 &ve)
						  { return ve != vertexData[selectedPoints.first] && ve != vertexData[selectedPoints.second] && ve != vertexData[thirdPoint]; });

		const size_t fourthPoint = (it == vertexData.end()) ? selectedPoints.first : std::distance(vertexData.begin(), it);
	
		return mesh.setup(selectedPoints.first, selectedPoints.second, thirdPoint, fourthPoint);
	}
	assert(maxIdx != selectedPoints.first && maxIdx != selectedPoints.second);

	// 2c. Forms base triangle for tetrahedron
	std::array<glm::vec3, 3> baseTriangle{
		vertexData[selectedPoints.first],
		vertexData[selectedPoints.second],
		vertexData[maxIdx]};

	std::array<size_t, 3> baseTriangleIndices{
		selectedPoints.first,
		selectedPoints.second,
		maxIdx};

	// 3a. Find point furthest from the plane (forms a tetrahedron)
	maxDist = epsilon;
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

	// 3b. Degenerate case -- point cloud seems to lie on 2D subspace of R^3
	if (maxDist == epsilon)
	{
		std::cerr << "3b. error \n";
		isPlanar = true;
		const glm::vec3 tempNormal = getTriangleNormal(baseTriangle[1], baseTriangle[2], baseTriangle[0]);

		tempPlanarVertices.clear();
		tempPlanarVertices.insert(tempPlanarVertices.begin(), vertexData.begin(), vertexData.end());

		const glm::vec3 newPoint = tempNormal + vertexData[0];
		tempPlanarVertices.push_back(newPoint);
		maxIdx = tempPlanarVertices.size() - 1;
		vertexData = tempPlanarVertices;
	}

	// Enforce CCW winding -- OpenGL considers all CCW polygons to be front-facing by default
	const Plane trianglePlane(normal, baseTriangle[0]);
	if (trianglePlane.isPointAbovePlane(vertexData[maxIdx])) {
		std::swap(baseTriangleIndices[0], baseTriangleIndices[1]);
	}


	// 3c. Create initial tetrahedron half edge mesh
	std::cout << "Creating tetrahedron with vertices: " << baseTriangleIndices[0]
		<< ", " << baseTriangleIndices[1] << ", " << baseTriangleIndices[2]
		<< ", " << maxIdx << std::endl;

	mesh.setup(baseTriangleIndices[0], baseTriangleIndices[1], baseTriangleIndices[2], maxIdx);

	std::cout << "Tetrahedron created with " << mesh.faces.size() << " faces" << std::endl;


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

	// 4. Assign points to each face if "outside" -- vertices inside tetrahedron are ignored
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


void ConvexHull::createConvexHalfEdgeMesh()
{

	visibleFaces.clear();
	horizonEdges.clear();
	possibleVisibleFaces.clear();

	// Compute base tetrahedron
	setupInitialTetrahedron();
	assert(mesh.faces.size() == 4);


	if (mesh.faces.size() != 4) {
		std::cerr << "Error: Expected 4 faces after setupInitialTetrahedron(), got "
			<< mesh.faces.size() << std::endl;
		return;
	}

	// Initialize face stack w/ newly assigned conflict lists
	faceStack.clear();
	for (size_t i = 0; i < 4; i++)
	{

		auto &f = mesh.faces[i];

		// Ensure face has a populated conflict list before pushing onto stack
		if (f.pointsOnPositiveSide && f.pointsOnPositiveSide->size() > 0)
		{
			faceStack.push_back(i);
			f.inFaceStack = 1; /*WTF is this a uint_8*/
		}
	}

	// Iterate over face stack until no "outside" points exist
	// Mark visited faces with current iteration counter
	size_t iter = 0;
	while (!faceStack.empty())
	{
		iter++;
		if (iter == std::numeric_limits<size_t>::max())
		{
			// Max iter represents unvisited faces, thus reset counter
			iter = 0;
		}

		// Pop top face off stack
		const size_t topIdx = faceStack.front();
		faceStack.pop_front();

		auto &topFace = mesh.faces[topIdx];
		topFace.inFaceStack = 0;

		assert(!topFace.pointsOnPositiveSide || topFace.pointsOnPositiveSide->size() > 0);

		// Ignore faces with empty conflict lists or disabled faces
		if (!topFace.pointsOnPositiveSide || topFace.isDisabled())
		{
			continue;
		}

		// Choose furthest point (eye point) as new potential vertex in convex hull
		const glm::vec3 &eyePoint = vertexData[topFace.furthestPoint];
		const size_t eyePointIdx = topFace.furthestPoint;

		
		// Find all faces visible to eye point -- on positive side of face plane
		// Build a list of horizon edges
		horizonEdges.clear();
		possibleVisibleFaces.clear();
		visibleFaces.clear();

		// Mark all faces as unvisited w/ numeric_limits<size_t>::max()
		possibleVisibleFaces.emplace_back(topIdx, std::numeric_limits<size_t>::max());

		// Find all visible faces
		while (possibleVisibleFaces.size())
		{

			const auto faceData = possibleVisibleFaces.back();
			possibleVisibleFaces.pop_back();

			auto &testFace = mesh.faces[faceData.faceIdx];
			assert(!testFace.isDisabled());

			// Face visibility already checked
			if (testFace.visibilityCheckedOnIteration == iter)
			{

				if (testFace.isVisibleFaceOnCurrentIteration)
				{
					continue;
				}
			}
			else
			{

				const Plane &P = testFace.P;
				testFace.visibilityCheckedOnIteration = iter;
				const float dist = getSignedDistanceToPlane(eyePoint, P);

				// Point is visible if outside plane
				if (dist > 0)
				{

					testFace.isVisibleFaceOnCurrentIteration = 1;
					testFace.horizonEdgesOnCurrentIteration = 0;
					visibleFaces.push_back(faceData.faceIdx);

					// Add/mark adjacent faces as possibly visible
					for (auto he : mesh.getFaceHalfEdges(testFace))
					{
						if (mesh.halfEdges[he].twin != faceData.enteredFromHalfEdge)
						{
							possibleVisibleFaces.emplace_back(mesh.halfEdges[mesh.halfEdges[he].twin].face, he);
						}
					}

					continue;
				}

				assert(faceData.faceIdx != topIdx);
			}

			// Face is not visible, thus half edge is part of the horizon edge
			testFace.isVisibleFaceOnCurrentIteration = 0;
			horizonEdges.push_back(faceData.enteredFromHalfEdge);

			// Get all half edges of non-visible face
			const auto halfEdges = mesh.getFaceHalfEdges(mesh.faces[mesh.halfEdges[faceData.enteredFromHalfEdge].face]);

			// Determine index of horizon half edge -- other half edges not part of final mesh
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

			std::cerr << convexhullModel.fileName << ": Failed to solve horizon edge." << std::endl;

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


		newFaces.clear();
		newHalfEdges.clear();
		disabledFaceConflictLists.clear();
		size_t disabledCount = 0;

		// Disable all visible faces and half-edges, we reuse std::vector memory allocated to their assigned points
		for (auto faceIdx : visibleFaces)
		{

			auto &disabledFace = mesh.faces[faceIdx];
			auto halfEdges = mesh.getFaceHalfEdges(disabledFace);

			// disable all half edges part associated with face
			for (size_t j = 0; j < 3; j++)
			{
				// exclude horizon half edge -- part of final convex hull
				if ((disabledFace.horizonEdgesOnCurrentIteration & (1 << j)) == 0)
				{

					if (disabledCount < horizonEdgesCount * 2)
					{
						newHalfEdges.push_back(halfEdges[j]);
						disabledCount++;
					}
					else
					{
						// disable half edge for future reuse
						mesh.disableHalfEdge(halfEdges[j]);
					}
				}
			}

			// Disable face
			auto ptr = mesh.disableFace(faceIdx);
			if(ptr){
		 		
				// Retain pointer to conflict list for reassignment to new faces
				assert(ptr->size());
				disabledFaceConflictLists.push_back(std::move(ptr));
			}
		}

		if(disabledCount < horizonEdgesCount * 2){

			const size_t newHalfEdgesNeeded = horizonEdgesCount * 2 - disabledCount;

			for(size_t i = 0; i < newHalfEdgesNeeded; i++){

				newHalfEdges.push_back(mesh.addHalfEdge());

			}
		}

		// Build new faces and half edges with horizon edge loop
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

			mesh.halfEdges[AB].next = BC;
			mesh.halfEdges[BC].next = CA;
			mesh.halfEdges[CA].next = AB;

			mesh.halfEdges[BC].face = newFaceIdx;
			mesh.halfEdges[CA].face = newFaceIdx;
			mesh.halfEdges[AB].face = newFaceIdx;

			mesh.halfEdges[CA].vert = A;
			mesh.halfEdges[BC].vert = C;

			// New face
			auto &newFace = mesh.faces[newFaceIdx];

			const glm::vec3 planeNormal = getTriangleNormal(vertexData[A], vertexData[B], eyePoint);
			newFace.P = Plane(planeNormal, eyePoint);
			newFace.he = AB;

			mesh.halfEdges[CA].twin = newHalfEdges[i > 0 ? (i * 2 - 1) : (2 * horizonEdgesCount - 1)];
			mesh.halfEdges[BC].twin = newHalfEdges[((i + 1) * 2) % (horizonEdgesCount * 2)];
		}

		// Assign disabled conflict lists to new faces
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

						// Skip to next point if successfully assigned to conflict list
						break;
					}
				}
			}

			// Recycle std::vector memory for reuse -- points reassigned to to new conflict list
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


// Check if horizon edges form connected loop
bool ConvexHull::connectHorizonEdges(std::vector<size_t> &horizonEdges)
{

	const size_t horizonEdgeCount = horizonEdges.size();
	for (size_t i = 0; i < horizonEdgeCount - 1; i++)
	{

		// inital end vertex
		const size_t endVertex = mesh.halfEdges[horizonEdges[i]].vert;

		bool foundNext = false;
		for (size_t j = i + 1; j < horizonEdgeCount; j++)
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
	assert(mesh.halfEdges[horizonEdges[horizonEdges.size() - 1]].vert == mesh.halfEdges[mesh.halfEdges[horizonEdges[0]].twin].vert);
	return true;
	
}


// Returns indices to extreme values
std::array<size_t, 6> ConvexHull::getExtrema()
{

	std::array<size_t, 6> outIndices{0, 0, 0, 0, 0, 0};
	std::array<float, 6> extremeValues{vertexData[0].x, vertexData[0].x, vertexData[0].y, vertexData[0].y, vertexData[0].z, vertexData[0].z};

	for (size_t i = 0; i < vertexData.size(); i++)
	{

		const glm::vec3 &pos = vertexData[i];

		// X-axis
		if (pos.x > extremeValues[0])
		{
			extremeValues[0] = pos.x;
			outIndices[0] = i;
		}

		else if (pos.x < extremeValues[1])
		{
			extremeValues[1] = pos.x;
			outIndices[1] = i;
		}

		// Y-axis
		if (pos.y > extremeValues[2])
		{
			extremeValues[2] = pos.y;
			outIndices[2] = i;
		}

		else if (pos.y < extremeValues[3])
		{
			extremeValues[3] = pos.y;
			outIndices[3] = i;
		}

		// Z-axis
		if (pos.z > extremeValues[4])
		{
			extremeValues[4] = pos.z;
			outIndices[4] = i;
		}

		else if (pos.z < extremeValues[5])
		{
			extremeValues[5] = pos.z;
			outIndices[5] = i;
		}
	}

	return outIndices;
}

// Returns scale for computing epsilon
float ConvexHull::getScale()
{

	float s = 0.0f;
	for (int i = 0; i < 6; i++)
	{
		// raw pointer to ith extrema vertex data
		const float *v = (const float *)(&vertexData[extremaIndices[i]]);

		// pointer offset for desired component of vertex  data -- [0, 1] (x-component), [2, 3] (y-component), [4, 5] (z-component)
		v += i / 2;

		s = std::max(s, std::abs(*v));
	}

	return s;
}


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


// Recycle conflict list memory to pool when face is removed
void ConvexHull::reclaimConflictList(std::unique_ptr<std::vector<size_t>> &ptr)
{

	const size_t size = ptr->size();

	//
	if ((size + 1) * 128 < ptr->capacity())
	{

		ptr.reset(nullptr);
		return;
	}

	conflictListsPool.push_back(std::move(ptr));
}

/* DONE */
void ConvexHull::writeOBJ(const std::string &fileName, const std::string &objectName) const
{


    string const& path = std::string(PROJECT_SOURCE_DIR) + "/resources/" + fileName;
	std::ofstream objFile;
	objFile.open(path);
	objFile << "o " << objectName << "\n";
	for (const auto &v : getVertices())
	{

		objFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
	}

	const auto &indBuf = getIndices();
	size_t triangleCount = indBuf.size() / 3;
	for (size_t i = 0; i < triangleCount; i++)
	{
		size_t a = indBuf[i * 3];
		size_t b = indBuf[i * 3 + 1];
		size_t c = indBuf[i * 3 + 2];

		if (a != b && b != c && a != c) {
			objFile << "f " << a + 1 << " " << b + 1 << " " << c + 1 << "\n";
		}


		//objFile << "f " << indBuf[i * 3] + 1 << " " << indBuf[i * 3 + 1] + 1 << " " << indBuf[i * 3 + 2] + 1 << "\n";
	}
	objFile.close();
}

void ConvexHull::Draw(Shader &shader)
{
	convexhullModel.Draw(shader);
}

/* DONE */
std::vector<glm::vec3>& ConvexHull::getVertices()
{

	return vertices;
}

/* DONE */
std::vector<size_t>& ConvexHull::getIndices()
{

	return indices;
}

/* DONE */
const std::vector<glm::vec3>& ConvexHull::getVertices() const
{

	return vertices;
}

/* DONE */
const std::vector<size_t>& ConvexHull::getIndices() const
{

	return indices;
}

/* DONE */
WorldTransform& ConvexHull::getWorldTransform() {
	return worldTrans;
}


void ConvexHull::debugPrintState() const
{
	std::cout << "=== ConvexHull Debug Info ===" << std::endl;
	std::cout << "vertexData.size(): " << vertexData.size() << std::endl;
	std::cout << "mesh.faces.size(): " << mesh.faces.size() << std::endl;
	std::cout << "mesh.halfEdges.size() : " << mesh.halfEdges.size() << std::endl;
	std::cout << "extremaIndices: ";
	for (size_t i = 0; i < 6; i++) {
		std::cout << extremaIndices[i] << " ";
	}
	std::cout << std::endl;
	std::cout << "scale: " << scale << std::endl;
	std::cout << "epsilon: " << epsilon << std::endl;
	std::cout << "hull vertices: \n";
	for (size_t i = 0; i < vertices.size(); i++) {
		std::cout << "idx: " << i << " " << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << "\n";
	}
	std::cout << "hull indices: ";
	for (size_t i = 0; i < indices.size(); i++) {
		std::cout << indices[i] << " ";
	}
	std::cout << std::endl;
	std::cout << "============================" << std::endl;
}

void ConvexHull::debugPrintUniqueVertices(std::vector<glm::vec3> vertices) const
{

	std::set<glm::vec3, Vec3Compare> uniqueVert(vertices.begin(), vertices.end());
	std::cout << uniqueVert.size() << std::endl;

}