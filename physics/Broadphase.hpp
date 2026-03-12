#pragma once

#include <cassert>
#include <glm/glm.hpp>

#include <queue>
#include <stack>
#include <unordered_map>
#include <vector>

#include "AABB.hpp"
#include "Config.hpp"
#include "Debugger.hpp"
#include "Ray.hpp"
#include "RigidBody.hpp"

const int NULL_INDEX = -1;

struct Node {
  int key = NULL_INDEX;
  int parent = NULL_INDEX;
  int left = NULL_INDEX;
  int right = NULL_INDEX;
  int height = -1;

  AABB box; // for object movement (leaves only)

  int body_id = -1;

  bool is_leaf() const { return left == NULL_INDEX && right == NULL_INDEX; };
};

class Broadphase {
private:
  std::vector<Node> nodes; // all nodes (active + deleted)
  std::unordered_map<int, int> body_node_map;
  int free_node = NULL_INDEX; // head of free list
  int node_count = 0;
  int root_index = NULL_INDEX;
  // float fat_margin = 0.05f;
  float fat_margin = 0.5f;

  /**
   * @return int height of node in tree
   */
  int get_height(int index) {
    if (index == NULL_INDEX) {
      return 0;
    }
    return nodes[index].height;
  }

  /**
   * @brief Reuse or allocate memory for new node
   * @return int index of new node object
   */
  int allocate_node() {
    // Reuse deleted node memory
    if (free_node != NULL_INDEX) {
      int index = free_node;
      free_node = nodes[index].parent; // point to next free node
      nodes[index] = Node();           /// reset node
      nodes[index].height = 1;
      node_count++;
      return index;
    }

    nodes.emplace_back();
    int index = nodes.size() - 1;
    nodes[index].height = 1;
    node_count++;
    return index;
  }

  /**
   * @brief Recycle node memory for reusage
   * @return void
   */
  void delete_node(int index) {
    nodes[index].parent = free_node; // point to free list head
    nodes[index].height = -1;        // mark as deleted
    free_node = index;
    node_count--;
  }

  /**
   * @return AABB tree node AABB struct
   */
  AABB get_node_aabb(int index) const {
    if (index == NULL_INDEX) {
      return AABB();
    }
    return nodes[index].box;
  }

  int find_best_sibling(int leaf_index);
  float compute_cost(int index);
  void balance_tree(int index);
  void validate(int index = -2);

public:
  std::vector<std::pair<int, int>> query_tree_pairs();
  std::vector<int> query_node(const AABB &box) const;
  void insert(AABB box, int body_id);
  void remove(int body_id);
  std::vector<std::pair<int, int>> update(const std::vector<RigidBody> &bodies, Debugger *debug = nullptr);
  bool raycast(const std::vector<RigidBody> &bodies, const Ray &ray, int &id, float &t, float tmax = 500.0f);
};
