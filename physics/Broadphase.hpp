#pragma once

#include <cassert>
#include <glm/glm.hpp>

#include <functional>
#include <queue>
#include <stack>
#include <unordered_map>
#include <vector>

#include "AABB.hpp"
#include "Config.hpp"
#include "Debugger.hpp"
#include "Ray.hpp"

const int NULL_INDEX = -1;

struct Node {
  int key = NULL_INDEX;
  int parent = NULL_INDEX;
  int left = NULL_INDEX;
  int right = NULL_INDEX;
  int height = NULL_INDEX;

  AABB box; // for object movement (leaves only)

  int entity_id = NULL_INDEX;
  bool is_leaf() const { return left == NULL_INDEX && right == NULL_INDEX; };
};

class Broadphase {
private:
  std::vector<Node> nodes; // all nodes (active + deleted)
  std::unordered_map<int, int> body_node_map;
  int free_node = NULL_INDEX; // head of free list
  int node_count = 0;
  int root_idx = NULL_INDEX;
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
      int idx = free_node;
      free_node = nodes[idx].parent; // point to next free node
      nodes[idx] = Node();
      nodes[idx].height = 1;
      node_count++;
      return idx;
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

  // current tight aabb of entity (in world-space)
  using AABBFn = std::function<AABB(int entity_id)>;

  // ray test against entity geometry
  using RayFn = std::function<RayHit(int entity_id, float t_max)>;

  /**
   * @brief rebuild bvh to follow object movement
   * @return potential collision pairs (entity ids)
   */
  std::vector<std::pair<int, int>> update(const AABBFn &get_aabb,
                                          Debugger *debug = nullptr);

  /**
   * @brief world-space closest-hit ray query against bvh tree leaves
   * @return true if any leaf was hit; id & hit hold the closest one
   */
  bool raycast(const Ray &ray, const RayFn &on_leaf, int &id, RayHit &hit,
               float t_max = 500.0f) const;
};
