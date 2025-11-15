#ifndef OCTREE_H
#define OCTREE_H

#include "AABB.hpp"
#include "RigidBody.hpp"

struct OctreeBody {

	const RigidBody* body;
	const AABB* aabb;

};

struct OctreeNode {

	AABB aabb;
	glm::vec3 centre;
	std::vector<OctreeBody> bodies;
	bool isLeaf;
	OctreeNode* children[8] = { nullptr };

};

class Octree {

private:

	OctreeNode root;
	float min_size;

	void insert_recursive(OctreeNode* curr, OctreeBody, b) {

	}

	void intersects_recursive(const OctreeNode* curr, std::set<>* out) {

	}
public:


};

#endif 
