#ifndef CONVEXHULL_HPP
#define CONVEXHULL_HPP

#include <filesystem>
#include <glm/glm.hpp>


#include "../inc/Shader.hpp"
#include "../inc/Mesh.hpp"
#include "../inc/Model.hpp"
#include "../inc/Plane.hpp"
#include "../inc/Ray.hpp"
#include "../inc/WorldTransform.hpp"
#include "HalfEdgeMesh.hpp"
#include "MeshBuilder.hpp"

class ConvexHull {
private:

	// Writing to .obj file
	Model convexhullModel;
	std::string convexhullName;

	// Face processing
	//HalfEdgeMesh mesh;

	// Rendering 
	std::unique_ptr<std::vector<glm::vec3>> optimizedVBO;
	std::vector<glm::vec3> vertices;
	std::vector<size_t> indices;

	// Transformations
	WorldTransform worldTrans;
	glm::vec3 localCentroid;
	glm::vec3 worldCentroid;

public:

	HalfEdgeMesh mesh;


	/*
	ConvexHull(const ConvexHull& o)
	{
		indices = o.indices;
		if (o.optimizedVBO)
		{
			optimizedVBO.reset(new std::vector<glm::vec3>(*o.optimizedVBO));
			vertices = *optimizedVBO;
		}
		else
		{
			vertices = o.vertices;
		}
	}

	ConvexHull& operator=(const ConvexHull& o)
	{
		if (&o == this)
		{
			return *this;
		}
		indices = o.indices;
		if (o.optimizedVBO)
		{
			optimizedVBO.reset(new std::vector<glm::vec3>(*o.optimizedVBO));
			vertices = *optimizedVBO;
		}
		else
		{
			vertices = o.vertices;
		}
		return *this;
	}

	ConvexHull(ConvexHull&& o)
	{
		indices = std::move(o.indices);
		if (o.optimizedVBO)
		{
			optimizedVBO = std::move(o.optimizedVBO);
			o.vertices = std::vector<glm::vec3>();
			vertices = *optimizedVBO;
		}
		else
		{
			vertices = o.vertices;
		}
	}

	ConvexHull& operator=(ConvexHull&& o)
	{
		if (&o == this)
		{
			return *this;
		}
		indices = std::move(o.indices);
		if (o.optimizedVBO)
		{
			optimizedVBO = std::move(o.optimizedVBO);
			o.vertices = std::vector<glm::vec3>();
			vertices = *optimizedVBO;
		}
		else
		{
			vertices = o.vertices;
		}
		return *this;
	}
	*/

	ConvexHull() = default;

	ConvexHull(const MeshBuilder& buildMesh, const std::vector<glm::vec3>& pointCloud, bool CCW, bool useOriginalIndices) {

		if (!useOriginalIndices) {
			optimizedVBO.reset(new std::vector<glm::vec3>());
		}

		std::vector<bool> faceProcessed(buildMesh.faces.size(), false);
		std::vector<size_t> faceStack;
		std::unordered_map<size_t, size_t> mapVertex2Index;

		indices.clear();

		for (size_t i = 0; i < buildMesh.faces.size(); i++)
		{

			if (!buildMesh.faces[i].isDisabled())
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
		const size_t faceCount = buildMesh.faces.size() - buildMesh.disabledFaces.size();
		indices.reserve(faceCount * 3);

		size_t i = 0;
		while (faceStack.size())
		{

			auto itf = faceStack.end() - 1;
			size_t topIdx = *itf;
			assert(!buildMesh.faces[topIdx].isDisabled());
			faceStack.erase(itf);

			if (faceProcessed[topIdx])
			{
				continue;
			}
			else
			{

				faceProcessed[topIdx] = true;
				auto he = buildMesh.getFaceHalfEdges(buildMesh.faces[topIdx]);

				// Push neighboring faces onto stack
				size_t adjacentFaces[] = { buildMesh.halfEdges[buildMesh.halfEdges[he[0]].twin].face, buildMesh.halfEdges[buildMesh.halfEdges[he[1]].twin].face, buildMesh.halfEdges[buildMesh.halfEdges[he[2]].twin].face };
				for (auto f : adjacentFaces)
				{
					if (!faceProcessed[f] && !buildMesh.faces[f].isDisabled())
					{
						faceStack.push_back(f);
					}
				}

				// Process face vertices
				auto faceVertices = buildMesh.getFaceVertices(buildMesh.faces[topIdx]);
				if (!useOriginalIndices)
				{
					for (auto& v : faceVertices)
					{
						auto itv = mapVertex2Index.find(v);
						if (itv == mapVertex2Index.end())
						{

							optimizedVBO->push_back(pointCloud[v]);
							size_t newIndex = optimizedVBO->size() - 1;
							mapVertex2Index[v] = newIndex;
							v = newIndex;

						}
						else
						{
							v = itv->second;
						}
					}
				}

				indices.push_back(faceVertices[0]);
				indices.push_back(faceVertices[1 + isCCW]);
				indices.push_back(faceVertices[2 - isCCW]);
			}
		}

		if (!useOriginalIndices) {
			vertices = std::vector<glm::vec3>(*optimizedVBO);
		}
		else {
			vertices = pointCloud;
		}

		this->mesh = HalfEdgeMesh(buildMesh, pointCloud);
	}

	void computeConvexHull(const Model& objectModel, WorldTransform objectTrans) {

		std::string objectName = objectModel.fileName;
		size_t dotPos = objectName.find_last_of(".");
		std::string name = objectName.substr(0, dotPos);
		std::string ext = objectName.substr(dotPos);
		convexhullName = name + "_convexhull" + ext;
		std::string const& path = std::string(PROJECT_SOURCE_DIR) + "/resources/" + convexhullName;


		// Rendering & Writing
		// Compute convex hull centroid for rendering & transformations
		

		if (std::filesystem::exists(path)) {

			// Use Pre-computed hull
			convexhullModel.LoadModel(convexhullName);
		}
		else {

			// Save convex hull for future iterations
			writeOBJ(convexhullName, name);

		}
		
		// hull centroid not model centroid?
		//localCentroid = computeCentroid();
		//glm::vec4 objectWorldTrans = objectTrans.GetMatrix() * glm::vec4(localCentroid, 1.0f);

		// same COM as model 
		worldCentroid = objectTrans.GetPosition();
		worldTrans.SetPosition(worldCentroid);
		

		//debugState();

	}

	void writeOBJ(const std::string& fileName, const std::string& objectName = "quickhull") const
	{


		string const& path = std::string(PROJECT_SOURCE_DIR) + "/resources/" + fileName;
		std::ofstream objFile;
		objFile.open(path);
		objFile << "o " << objectName << "\n";
		for (const auto& v : getVertices())
		{

			objFile << "v " << v.x << " " << v.y << " " << v.z << "\n";
		}

		const auto& indBuf = getIndices();
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

	std::vector<glm::vec3>& getVertices()
	{

		return vertices;
	}

	std::vector<size_t>& getIndices()
	{

		return indices;
	}

	const std::vector<glm::vec3>& getVertices() const
	{

		return vertices;
	}

	const std::vector<size_t>& getIndices() const
	{

		return indices;
	}

	/*
	// DOESNT WORK
	// Möller–Trumbore intersection algorithm
	bool ConvexHull::computeRayIntersection(const Ray& r, float& t) {

		for (auto& f : mesh.faces) {

			auto faceVertices = mesh.getFaceVertices(f);

			float t = 0, u = 0, v = 0;

			size_t v0 = faceVertices[0];
			size_t v1 = faceVertices[1];
			size_t v2 = faceVertices[2];

			glm::vec3 v0v1 = vertices[v1] - vertices[v0];
			glm::vec3 v0v2 = vertices[v2] - vertices[v0];
			glm::vec3 pvec = glm::cross(r.direction, v0v2);
			float det = glm::dot(v0v1, pvec);

			float eps = std::numeric_limits<float>::epsilon();

			// triangle is backfacing if determinant is negative
			if (det < eps) {

				continue;
			}

			// ray and triangle are parallel
			if (std::abs(det) < eps) {

				std::cerr << "parallel\n";
				continue;
			}

			float invDet = 1 / det;

			glm::vec3 tvec = r.origin - vertices[v0];
			u = glm::dot(tvec, pvec) * invDet;
			if (u < 0 || u > 1) {

				std::cerr << "u\n";
				continue;
			}

			glm::vec3 qvec = glm::cross(tvec, v0v1);
			v = glm::dot(r.direction, qvec) * invDet;
			if (v < 0 || u + v > 1) {

				std::cerr << "v\n";
				continue;
			}

			t = glm::dot(v0v2, qvec) * invDet;

			std::cerr << "hit\n";
			return true;
		}

		std::cerr << "miss\n";
		return false;
	}
	*/
	

	/*
	// DOESNT WORK
	bool ConvexHull::computeRayIntersection(const Ray& r, float& outT) {
		const float EPS = 1e-6f;

		float tEnter = -std::numeric_limits<float>::infinity();
		float tExit = std::numeric_limits<float>::infinity();
		size_t enterFace = SIZE_MAX;

		for (size_t i = 0; i < mesh.faces.size(); ++i) {
			const auto& F = mesh.faces[i];
			const Plane& P = F.P;  // assume P.normal is unit‐length, P.distance = –dot(P.normal, a known point on the plane)

			float denom = glm::dot(P.normal, r.direction);
			float numer = glm::dot(P.normal, r.origin) + P.distance;

			if (std::abs(denom) < EPS) {
				// Ray nearly parallel: if origin is outside (numer>0) -> no hit at all
				if (numer > 0.0f) {
					std::cerr << " PARALLEL MISS on face " << i << "\n";
					return false;
				}
				// otherwise no constraint from this plane
				continue;
			}

			float tHit = -numer / denom;
			if (denom < 0.0f) {
				// entering
				if (tHit > tEnter) {
					tEnter = tHit;
					enterFace = i;
				}
			}
			else {
				// exiting
				if (tHit < tExit) {
					tExit = tHit;
				}
			}

			// early‐out if the intervals no longer overlap
			if (tEnter > tExit) {
				std::cerr << " EARLY MISS on face " << i << "\n";
				return false;
			}
		}

		// At this point intervals [tEnter, tExit] overlap.
		// Two cases:
		//  1) tEnter >= 0: ray hits from outside at tEnter
		//  2) tEnter < 0 <= tExit: ray starts inside, so “hit” is at tExit
		if (tExit < 0.0f) {
			std::cerr << " MISS: both entry and exit behind the ray\n";
			return false;
		}

		// Choose the first positive intersection
		outT = (tEnter >= EPS ? tEnter : tExit);
		std::cerr << " HIT at t=" << outT
			<< " on face " << enterFace
			<< (tEnter < 0.0f ? " (origin inside, exiting)" : " (entering)")
			<< "\n";
		return true;
	}
	*/

	// same tmax as far plane from perspective matrix???
	int ConvexHull::computeRayIntersection(const Ray& r, float& t, float tmax = 500.0f) {

		// MISSED		:	return 0
		// FRONT FACE	:	return 1
		// BACK FACE	:	return -1
		
		const float eps = 0.000001f;
		float vn, vd, tnear, tfar;

		tnear = - std::numeric_limits<float>::infinity();
		tfar = tmax;

		for (size_t i = 0; i < mesh.faces.size(); i++) {

			const Plane& P = mesh.faces[i].P;

			vd = glm::dot(r.direction, P.normal);
			vn = glm::dot(r.origin, P.normal) + P.distance;

			// ray parallel to plane
			if (std::abs(vd) < eps) {

				if (vn > 0.0) {
					std::cerr << "a\n";
					return 0;
				}
				continue;
			}
			else {
				
				t = -vn / vd;

				// front facing
				if (vd < 0.0f) {

					std::cerr << "b\n";
					if (t > tfar) return 0;
					if (t > tnear) {

						tnear = t;
					}
				}

				// back facing
				else {

					std::cerr << "c\n";
					if (t < tnear) return 0;
					if (t < tfar) {

						tfar = t;
					}
				}
			}
		}

		// pass tests
		if (tnear >= eps) {

			t = tnear;
			std::cerr << "HIT\n";
			return 1;
		}
		else {

			if (tfar < tmax) {

				t = tfar;
				std::cerr << "HIT\n";
				return -1;
			}
			else {

				std::cerr << "d\n";
				return 0;
			}
		}

	}
	



	glm::vec3 computeCentroid() {

		unsigned int num = 0;
		glm::vec3 centroid(0.0f, 0.0f, 0.0f);

		for (auto& v : vertices) {

			centroid += v;
			num++;
		}

		return centroid /= static_cast<float>(num);
	}

	void updateCentroid(glm::vec3 displacement) {

		/*
		worldTrans = objectTrans;

		// Apply the object's transformation to the centroid
		glm::vec4 worldCentroidTrans = objectTrans.GetMatrix() * glm::vec4(localCentroid, 1.0f);
		worldCentroid = glm::vec3(worldCentroidTrans);
		*/

		worldCentroid += displacement;
	}


	void Draw(Shader& shader)
	{
		convexhullModel.Draw(shader);
	}

	WorldTransform& getWorldTransform() {
		return worldTrans;
	}

	void debugState() const
	{
		std::cout << "=============================" << std::endl;
		std::cout << "=== ConvexHull Debug Info ===" << std::endl;
		std::cout << "vertices.size(): " << vertices.size() << std::endl;
		std::cout << "indices.size(): " << indices.size() << std::endl;
		std::cout << "============================" << std::endl;
	}

};


#endif 
