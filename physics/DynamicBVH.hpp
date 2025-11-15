#ifndef BROADPHASE_HPP
#define BROADPHASE_HPP

#include <glm/glm.hpp>

#include <vector>

#include "AABB.hpp"


class Tree {
private:

	std::vector<Node*> nodes;
	size_t nodeCount;
	size_t rootIndex;
public:

	void insert(Tree tree, int objectIdx, AABB box) {

		Node* new_leaf = new Node()
		if (tree.nodeCount == 1) {
		}


		// 1. find best candidate sibling for new leaf
		int bestSibling = 0;
		for (int i = 0; i < nodeCount; i++) {

			bestSibling = pickBest(bestSibling, i);
		}

	}

	void remove();
	void update();


};

class Node {

	AABB box;
	size_t parent;
	size_t left;
	size_t right;
	size_t leafObjectID;

	bool isLeaf;

};



#endif
