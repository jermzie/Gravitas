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

	// same tmax as far plane from perspective matrix???
	int computeRayIntersection(const Ray& r, float& t, float tmax = 500.0f) {

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
					//std::cerr << "a\n";
					return 0;
				}
				continue;
			}
			else {
				
				t = -vn / vd;

				// front facing
				if (vd < 0.0f) {

					//std::cerr << "b\n";
					if (t > tfar) return 0;
					if (t > tnear) {

						tnear = t;
					}
				}

				// back facing
				else {

					//std::cerr << "c\n";
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
			//std::cerr << "HIT\n";
			return 1;
		}
		else {

			if (tfar < tmax) {

				t = tfar;
				//std::cerr << "HIT\n";
				return -1;
			}
			else {

				//std::cerr << "d\n";
				return 0;
			}
		}

	}
	
	bool computePlaneIntersection(const Plane& p) {

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

	// helper function for computing surface integrals
	void computeSubexpressions(float w0, float w1, float w2, float& f1, float& f2, float& f3, float& g0, float& g1, float& g2) {

		float temp0 = w0 + w1; 
		f1 = temp0 + w2; 
		
		float temp1 = w0 * w0; 
		float temp2 = temp1 + w1 * temp0;

		f2 = temp2 + w2 * f1; 
		f3 = w0 * temp1 + w1 * temp2 + w2 * f2;

		g0 = f2 + w0 * (f1 + w0); 
		g1 = f2 + w1 * (f1 + w1); 
		g2 = f2 + w2 * (f1 + w2);
	}

	void computeMassProperties(float density, float& mass, glm::vec3& com, glm::mat3& inertia) {

		const float mult[10] = { 1.0f / 6, 1.0f / 24, 1.0f / 24, 1.0f / 24, 1.0f / 60, 1.0f / 60, 1.0f / 60, 1.0f / 120, 1.0f / 120, 1.0f / 120 };
		float intg[10] = { 0.0f }; // volume integrals order: 1, x, y, z, x^2, y^2, z^2, xy, yz, zx

		for (size_t i = 0; i < mesh.faces.size(); i++) {

			auto& face = mesh.faces[i];

			// get face vertices
			auto v = mesh.getFaceVertices(face);
			
			glm::vec3 v0 = vertices[v[0]];
			glm::vec3 v1 = vertices[v[1]];
			glm::vec3 v2 = vertices[v[2]];

			// get edges and cross products
			glm::vec3 n = glm::normalize(getTriangleNormal(v1, v2, v0));
			//glm::vec3 e1 = v1 - v0;
			//glm::vec3 e2 = v2 - v0;
			//glm::vec3 n = glm::normalize(face.P.normal);

			// surface integral shortcuts

			// x-components
			float fX, fSqX, fCuX, gX0, gX1, gX2;
			computeSubexpressions(v0.x, v1.x, v2.x, fX, fSqX, fCuX, gX0, gX1, gX2); 

			// y-components
			float fY, fSqY, fCuY, gY0, gY1, gY2;
			computeSubexpressions(v0.y, v1.y, v2.y, fY, fSqY, fCuY, gY0, gY1, gY2);
			 
			// z-components
			float fZ, fSqZ, fCuZ, gZ0, gZ1, gZ2;
			computeSubexpressions(v0.z, v1.z, v2.z, fZ, fSqZ, fCuZ, gZ0, gZ1, gZ2);

			// update integrals
			// 1
			intg[0] += n.x * fX;

			// x, y, z
			intg[1] += n.x * fSqX;	intg[2] += n.y * fSqY;	intg[3] += n.z * fSqZ;

			// x^2. y^2, z^2
			intg[4] += n.x * fCuX;	intg[5] += n.y * fCuY;	intg[6] += n.z * fCuZ;

			// xy, yz, zx
			intg[7] += n.x * (v0.y * gX0 + v1.y * gX1 * v2.y * gX2);
			intg[8] += n.y * (v0.z * gY0 + v1.z * gY1 * v2.z * gY2);
			intg[9] += n.z * (v0.x * gZ0 + v1.x * gZ1 * v2.x * gZ2);


		}
	
		// multiply constants
		for (size_t i = 0; i < 10; i++) {
			intg[i] *= mult[i];
		}

		float vol = intg[0];
		mass = density * vol;

		com.x = intg[1] / vol;
		com.y = intg[2] / vol;
		com.z = intg[3] / vol;

		// xx, yx, zx
		glm::vec3 col1(
			density * (intg[5] + intg[6] - vol * (com.y * com.y + com.z * com.z)),
			density * -(intg[7] - vol * (com.y * com.x)),
			density * -(intg[9] - vol * (com.z * com.x)),
		);
		
		// xy, yy, zy
		glm::vec3 col2(
			-(intg[7] - vol * (com.x * com.y)),
			intg[4] + intg[6] - vol * (com.z * com.z + com.x * com.x),
			-(intg[8] - vol * (com.z * com.y)),
		);

		// xz, yz, zz
		glm::vec3 col3(
			-(intg[9] - vol * (com.x * com.z)),
			-(intg[8] - vol * (com.y * com.z)),
			intg[4] + intg[5] - vol * (com.x * com.x + com.y * com.y),
		);

		inertia = glm::mat3(col1, col2, col3);

	}

};


#endif 
