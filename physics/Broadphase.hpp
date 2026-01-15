#ifndef BROADPHASE_HPP
#define BROADPHASE_HPP

#include <glm/glm.hpp>

#include <vector>
#include <stack>
#include <queue>

#include "AABB.hpp"
#include "ConvexHull.hpp"
#include "../inc/Ray.hpp"
#include "RigidBody.hpp"
#include "PhysicsEngine.hpp"

static const int NULL_INDEX = -1;

struct Node {

	int parent = NULL_INDEX;
	int left = NULL_INDEX;
	int right = NULL_INDEX;
	int height = -1;

	AABB tight_box;		// for object movement (leaves only)
	AABB box;
	int object_id = -1;

	bool is_leaf() const { return left == NULL_INDEX && right == NULL_INDEX };
};

class DynamicBVH {
private:

	std::vector<Node> nodes;
	int free_list = NULL_INDEX;
	int node_count = 0;
	int root_index = NULL_INDEX;
	float fat_margin = 0.5f;

	// DONE?
	int allocate_node() {
		if (free_list != NULL_INDEX) {
			int index = free_list;
			free_list = nodes[index].parent;
			nodes[index] = Node();
			nodes[index].height = 0;	// ??
			node_count++;
			return index;
		}
		nodes.emplace_back();
		int index = nodes.size() - 1;
		nodes[index].height = 0;
		//nodes.push_back(Node());
		node_count++;
		//return nodes.size() - 1;
	}

	// DONE?
	void free_node(int index) {
		nodes[index].parent = free_list;
		nodes[index].height = -1;
		free_list = index;
		node_count--;
	}

	// DONE?
	// compute surface area cost of tree
	float compute_cost() {

		float cost = 0.0f;
		for (int i = 0; i < node_count; i++) {

			if (nodes[i].height < 0) continue;

			// only compute cost of internal nodes -- surface area of leaves is always same
			if (!nodes[i].is_leaf) {
				cost += nodes[i].box.surface_area();
			}
		}
		return cost;
	}

	// DONE?
	int find_best_sibling(int leaf_index) {
		if (root_index == NULL_INDEX) return NULL_INDEX;
		const AABB& leaf_box = nodes[leaf_index].box;
		float leaf_sa = leaf_box.surface_area();

		struct Candidate {
			int index;
			float min_cost;
			float inherited_cost;
			bool operator>(const Candidate& o) const { return min_cost > o.min_cost; }
		};

		std::priority_queue<Candidate, std::vector<Candidate>,std::greater<Candidate>> pq;

		float best_cost = combine(nodes[root_index].box, leaf_box).surface_area();
		int best_sibling = root_index;

		pq.push({ root_index, 0.0f, 0.0f });

		while (!pq.empty()) {

			Candidate c = pq.top();
			pq.pop();

			if (c.min_cost >= best_cost) continue;

			// get AABB
			const Node& n = nodes[c.index];
			AABB potential_sibling = combine(n.box, leaf_box);
			float direct_cost = potential_sibling.surface_area();
			float cost = direct_cost + c.inherited_cost;

			if (cost < best_cost) {
				best_cost = cost;
				best_sibling = c.index;
			}

			if (n.is_leaf()) continue;

			// calculate new inherited cost for child nodes
			float new_inherited_cost = c.inherited_cost + direct_cost - n.box.surface_area();
			float min_cost = leaf_sa + new_inherited_cost;

			if (min_cost < best_cost && !n.is_leaf()) {
				pq.push({ n.left, min_cost, new_inherited_cost });
				pq.push({ n.right, min_cost, new_inherited_cost });
			}

		}

		return best_sibling;
	}

	// AVL balancing
	int balance_tree(int index) {
		if (nodes[index].is_leaf() || nodes[index].height < 2) return INDEX;

		int left = nodes[index].left;
		int right = nodes[index].right;

		// recall AVL trees can only have balance factor of {-1, 0, 1}, otherwise need to balance
		int balance_factor = nodes[left].height - nodes[right].height;

		// rotate left up
		if (balance_factor > 1) {

			int sub_left = nodes[left].left;
			int sub_right = nodes[left].right;
		}

		// rotate right up
		if (balance_factor < -1) {

			int sub_left = nodes[right].left;
			int sub_right = nodes[right].right;
		}

		return index;
	}
	
public:
	// compute all possible collision pairs 
	// returns list of pairs of object_id values for narrowphase processing
	std::vector<std::pair<int, int>> query_tree_pairs() const {
		std::vector<std::pair<int, int>> collision_list;
		if (root_index == NULL_INDEX) return collision_list;

		//
		std::stack<std::pair<int, int>> stack;
		stack.push({ root_index, root_index });

		while (!stack.empty()) {

			std::pair<int,int>index = stack.top();
			stack.pop();

			const Node& a_node = nodes[index.first];
			const Node& b_node = nodes[index.second];

			// pruning --
			if (!aabb_intersect(a.box, b.box)) continue;

			bool a_leaf = a_node.is_leaf();
			bool b_leaf = b_node.is_leaf();

			// Both leaves
			if (a_leaf && b_leaf) {

				// make sure not same node
				if (index.first != index.second) {
					collision_list.push_back({ a_node.object_id, b_node.object_id });
				}
			}
			// A is a leaf, descend B's children
			else if (a_leaf) {

				stack.push({index.first, b_node.left});
				stack.push({index.first, b_node.right});
			}
			// B is a leaf, descend A's children
			else if (b_leaf) {

				stack.push({ index.first, b_node.left });
				stack.push({ index.first, b_node.right });
			}
			// Both internal
			else {
				if () {

				}
				else {

					// cross-query
					// descend large node for better pruning
					if (a_node.box.surface_area() > b_node.box.surface_area()) {

					}
					else {

					}
				}
			}
		}

		return collision_list;
	}

	// compute all leaves possibly colliding with AABB
	// returns list of object_id values for narrowphase processing?
	std::vector<int> query_node(const AABB& box) const {
		std::vector<int> collision_list;
		if (root_index == NULL_INDEX) return collision_list;

		std::stack<int>stack;
		stack.push(root_index);

		while (!stack.empty()) {

			int index = stack.top;
			stack.pop();

			if (index == NULL_INDEX) continue;

			const Node& node = nodes[index];
			if (!aabb_intersect(box, node.box)) continue;

			if (node.is_leaf()) {
				collision_list.push_back(node.object_id);
			}
			else {
				stack.push(node.left);
				stack.push(node.right);
			}
		}

		return collision_list;
	}

	// create new node from rigidbody
	void insert(AABB box, int object_id) {

		// allocate new leaf
		int leaf_index = allocate_node();
		Node& leaf = nodes[leaf_index];
		leaf.object_id = object_id;
		leaf.height = 0;
		leaf.tight_box = box;
		leaf.box = fatten(box, fat_margin);
	
		// base case -- zero leaves
		if (root_index == NULL_INDEX) {
			root_index = leaf_index;
			nodes[leaf_index].parent = NULL_INDEX;
			return;
		}


		// 1. -- find best sibling for new leaf
		int sibling = find_best_sibling(leaf_index);


		// 2. -- create new internal parent node
		size_t old_parent = nodes[sibling].parent;
		size_t new_parent = allocate_node();
		Node& internal = nodes[new_parent];
		internal.parent = old_parent;
		internal.box = combine(nodes[leaf_index].box, nodes[sibling].box);

		// check if sibling is root node
		if (old_parent != nullptr) {

			if (nodes[old_parent].left == sibling) {
				nodes[old_parent].left = new_parent;
			}
			else {
				nodes[old_parent].right = new_parent;
			}
		}
		// sibling is root node
		else {
			root_index = new_parent;
		}

		internal.left = sibling;
		internal.right = leaf_index;
		nodes[sibling].parent = newParent;
		nodes[leaf_index].parent = newParent;

		// 3. -- walk up tree, refitting ancestor AABBs
		int index = new_parent;
		while (index != NULL_INDEX) {

			int left = nodes[index].left;
			int right = nodes[index].right;

			nodes[index].box = combine(nodes[left].box, nodes[right].box);
			nodes[index].height = 1 + std::max(nodes[left].height, nodes[right].height);

			// balance tree

			index = nodes[index].parent;
		}
	}

	// remove node and refit its ancestors
	void remove(int leaf_index) {
		if (leaf_index < 0 || leaf_index >= nodes.size()) return;
		if (!nodes[leaf_index].is_leaf()) return;

		if (leaf_index == root_index) {
			root_index = NULL_INDEX;
			return;
		}

		int parent = nodes[leaf_index].parent;		// internal node
		int grand_parent = nodes[parent].parent;	// ? node
		int sibling = (nodes[parent].left == leaf_index) ? nodes[parent].right : nodes[parent].left;

		// check if parent is root node
		if (grand_parent != NULL_INDEX) {

			// reassign sibling as parent node
			if (nodes[grand_parent].left == parent) {
				nodes[grand_parent].left == sibling;
			}
			else {
				nodes[grand_parent].right = sibling;
			}
			nodes[sibling].parent = grand_parent;
			free_node(parent);		// remove parent internal node

			// refit ancestors
			int index = grand_parent;
			while (index != NULL_INDEX) {
				
				int left = nodes[index].left;
				int right = nodes[index].right;

				nodes[index].box = combine(nodes[left].box, nodes[right].box);
				nodes[index].height = 1 + std::max(nodes[left].height, nodes[right].height);
				index = nodes[index].parent;
			}

		}
		else {

			// siblinng becomes root node
			root_index = sibling;
			nodes[sibling].parent = NULL_INDEX;
			free_node(parent);
		}

		free_node(leaf_index);
	}


	//  rebuild bvh to follow object movement
	void update() {

		std::vector<int>moved_leaves;

		// 
		for (int i = 0; i < nodes.size(); i++) {
			if (nodes[i].height < 0) continue;
			if (!nodes[i].is_leaf()) continue;

			AABB new_box;
			//
			if (!nodes[i].box.contains(nodes[i].tight_box)) {
				nodes[i].tight_box = new_box;
				moved_leaves.push_back(i);
			}
			else {
				nodes[i].tight_box = new_box;
			}
		}

		for (int leaf_index : moved_leaves) {

			AABB leaf_box = nodes[leaf_index].tight_box;
			int object_id = nodes[leaf_index].object_id;
			remove(leaf_index);
			insert(leaf, object_id);

		}


		return query_tree_pairs();
	}

	// NEEDS WORK
	bool ray_intersection(Ray& ray, float tmax = 500.0f) {

		std::stack<int> stack;
		stack.push(root_index);
		while (!stack.empty()) {

			int index = stack.top;
			stack.pop();

			Node& node = nodes[index];

			// AABB ray test
			if (!node.box.ray_intersect(ray, tmax)) continue;

			if (node.is_leaf()) {

				int object_id = node.object_id;
				glm::vec3 hit;

				// ConvexHull ray test
				if (bodies[object_id].ray_intersection(ray, hit) {
					
					return true;
				}
				
			}
			else {

				stack.push(node.left);
				stack.push(node.right);
			}
		}
		return false;
	}


};


#endif
