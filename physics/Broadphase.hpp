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

// #define DEBUG

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
  float fat_margin = 0.05f;

  int get_height(int index) {
    if (index == NULL_INDEX) {
      return 0;
    }
    return nodes[index].height;
  }
  //
  // DONE?
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

  // DONE?
  void delete_node(int index) {
    nodes[index].parent = free_node; // point to free list head
    nodes[index].height = -1;        // mark as deleted
    free_node = index;
    node_count--;
  }

  AABB get_node_aabb(int index) const {
    if (index == NULL_INDEX) {
      return AABB();
    }
    return nodes[index].box;
  }

  // DONE?
  /**
   * @brief Implements Branch and Bound algo to optimize global search for sibling node
   * @return index of sibling node
   */
  int find_best_sibling(int leaf_index) {
    if (root_index == NULL_INDEX) {
      return NULL_INDEX;
    }

    const AABB &leaf_box = nodes[leaf_index].box;
    float leaf_sa = leaf_box.surface_area();

    struct Candidate {
      int index;
      float min_cost;
      float inherited_cost;
      bool operator>(const Candidate &o) const { return min_cost > o.min_cost; }
    };

    std::priority_queue<Candidate, std::vector<Candidate>, std::greater<Candidate>> pq;

    // float best_cost = combine(nodes[root_index].fat_box, leaf_box).surface_area();
    float best_cost = combine(get_node_aabb(root_index), leaf_box).surface_area();
    int best_sibling = root_index;

    pq.push({root_index, 0.0f, 0.0f});

    while (!pq.empty()) {
      Candidate c = pq.top();
      pq.pop();

      if (c.min_cost >= best_cost) {
        continue;
      }

      // get AABB
      const Node &n = nodes[c.index];
      // AABB potential_sibling = combine(n.fat_box, leaf_box);
      AABB potential_sibling = combine(get_node_aabb(c.index), leaf_box);
      float direct_cost = potential_sibling.surface_area();
      float cost = direct_cost + c.inherited_cost;

      if (cost < best_cost) {
        best_cost = cost;
        best_sibling = c.index;
      }

      if (n.is_leaf()) {
        continue;
      }

      // calculate new inherited cost for child nodes
      float new_inherited_cost = c.inherited_cost + direct_cost - get_node_aabb(c.index).surface_area();
      float min_cost = leaf_sa + new_inherited_cost;

      if (min_cost < best_cost && !n.is_leaf()) {
        pq.push({n.left, min_cost, new_inherited_cost});
        pq.push({n.right, min_cost, new_inherited_cost});
      }
    }

    return best_sibling;
  }

  // DONE?
  // compute surface area cost of tree
  float compute_cost(int index) {
    float cost = 0.0f;

    std::stack<int> stack;
    const Node &n = nodes[index];
    stack.push(n.left);
    stack.push(n.right);
    for (int i = 0; i < node_count; i++) {
      if (nodes[i].height < 0) {
        continue;
      }

      // only compute cost of internal nodes -- surface area of leaves is always
      // same
      if (!nodes[i].is_leaf()) {
        cost += nodes[i].box.surface_area();
      }
    }
    return cost;
  }

  // Surface area based balancing
  void balance_tree(int index) {
    if (index == NULL_INDEX || nodes[index].is_leaf() || nodes[index].height < 2) {
      return;
    }

    Node &parent_node = nodes[index];

    int left = parent_node.left;
    int right = parent_node.right;
    Node &left_node = nodes[left];
    Node &right_node = nodes[right];
    if (left == NULL_INDEX || right == NULL_INDEX) {
      return;
    }

    if (!left_node.is_leaf()) {
      int sub_left = left_node.left;
      int sub_right = left_node.right;

      AABB swap = combine(nodes[sub_right].box, right_node.box);
      if (swap.surface_area() < left_node.box.surface_area()) {
        // swap sub_left with right
        parent_node.right = sub_left;
        nodes[sub_left].parent = index;

        // FIXME:?????????
        left_node.right = right;
        right_node.parent = left;

        // refit parent
        left_node.box = swap;
        parent_node.box = combine(left_node.box, nodes[sub_left].box);
        return;
      }

      swap = combine(nodes[sub_left].box, nodes[right].box);
      if (swap.surface_area() < left_node.box.surface_area()) {

        return;
      }
    }

    if (!right_node.is_leaf()) {
      int rl_index = right_node.left;
      int rr_index = right_node.right;

      AABB swap = combine(nodes[rr_index].box, left_node.box);
      if (swap.surface_area() < right_node.box.surface_area()) {

        right_node.right = left;
        left_node.parent = right;
        parent_node.left = rl_index;
        nodes[rl_index].parent = index;

        right_node.box = swap;
        parent_node.box = combine(right_node.box, nodes[rl_index].box);
        return;
      }

      swap = combine(nodes[rl_index].box, left_node.box);
      if (swap.surface_area() < right_node.box.surface_area()) {

        right_node.right = left;
        left_node.parent = right;
        parent_node.left = rr_index;
        nodes[rr_index].parent = index;

        right_node.box = swap;
        parent_node.box = combine(right_node.box, nodes[rr_index].box);
        return;
      }
    }
  }

  void validate(int index = -2) {
    if (index == -2) {
      index = root_index;
    }
    if (index == NULL_INDEX) {
      return;
    }

    const Node &n = nodes.at(index);
    if (n.is_leaf()) {
      assert(n.left == NULL_INDEX && n.right == NULL_INDEX);
      assert(n.body_id >= 0);
      assert(n.height == 1);
      return;
    }

    int left = n.left;
    int right = n.right;
    assert(left != NULL_INDEX);
    assert(right != NULL_INDEX);
    assert(nodes.at(left).parent == index);
    assert(nodes.at(right).parent == index);

    int expected_height = 1 + std::max(get_height(left), get_height(right));
    assert(n.height == expected_height);

    // TODO: BAD AVL BALANCING STRATEGY
    // int bf = nodes.at(left).height - nodes.at(right).height;
    // assert(bf >= -1 && bf <= 1);

    validate(left);
    validate(right);
  }

public:
  // compute all possible collision pairs
  // returns list of pairs of object_id values for narrowphase processing
  std::vector<std::pair<int, int>> query_tree_pairs() {
    std::vector<std::pair<int, int>> collision_list;
    if (root_index == NULL_INDEX) {
      CLOGW("No Bodies");
      return collision_list;
    }

    const Node &root = nodes[root_index];

    // Single body - no possible collisions
    if (root.is_leaf()) {
      return collision_list;
    }

    std::stack<std::pair<int, int>> stack;
    if (root.left != NULL_INDEX)
      stack.push({root.left, root.left});
    if (root.right != NULL_INDEX)
      stack.push({root.right, root.right});
    if (root.left != NULL_INDEX && root.right != NULL_INDEX)
      stack.push({root.left, root.right});

    while (!stack.empty()) {

      std::pair<int, int> pair = stack.top();
      stack.pop();

      if (pair.first == NULL_INDEX || pair.second == NULL_INDEX)
        continue;
      if (pair.first >= nodes.size() || pair.second >= nodes.size())
        continue;

      if (pair.first == pair.second) {
        const Node &node = nodes[pair.first];
        if (!node.is_leaf()) {
          // Test children against each other and themselves
          // WARNING: QUESTIONABLE PERFORMANCE - DUPE COLL. CHECKS (A, B) vs (B, A)
          stack.push({node.left, node.left});
          stack.push({node.right, node.right});
          stack.push({node.left, node.right});
        }
        continue;
      }

      const Node &a_node = nodes[pair.first];
      const Node &b_node = nodes[pair.second];

      // pruning non-intersecting pairs
      /// if (!aabb_intersect(a_node.fat_box, b_node.fat_box)) {
      if (!aabb_intersect(get_node_aabb(pair.first), get_node_aabb(pair.second))) {
        continue;
      }

      bool a_leaf = a_node.is_leaf();
      bool b_leaf = b_node.is_leaf();

      // Both leaves - POTENTIAL COLLISION
      if (a_leaf && b_leaf) {
        // make sure not same node
        if (a_node.body_id != b_node.body_id) {
          // if (a_node.body_id < b_node.body_id) {
          collision_list.push_back({a_node.body_id, b_node.body_id});
        }
      }
      // A is a leaf, descend B's children
      else if (a_leaf) {
        stack.push({pair.first, b_node.left});
        stack.push({pair.first, b_node.right});
        continue;
      }
      // B is a leaf, descend A's children
      else if (b_leaf) {
        stack.push({a_node.left, pair.second});
        stack.push({a_node.right, pair.second});
        continue;
      }
      // Both internal
      else {
        // cross-query
        // descend large node for better pruning
        float a_sa = a_node.box.surface_area();
        float b_sa = b_node.box.surface_area();

        if (a_sa > b_sa) {
          stack.push({a_node.left, pair.second});
          stack.push({a_node.right, pair.second});

        } else {

          stack.push({pair.first, b_node.left});
          stack.push({pair.first, b_node.right});
        }
      }
    }

    return collision_list;
  }

  // compute all leaves possibly colliding with AABB
  // returns list of object_id values for narrowphase processing?
  std::vector<int> query_node(const AABB &box) const {
    std::vector<int> collision_list;
    if (root_index == NULL_INDEX) {
      return collision_list;
    }

    std::stack<int> stack;
    stack.push(root_index);

    while (!stack.empty()) {
      int index = stack.top();
      stack.pop();

      if (index == NULL_INDEX) {
        continue;
      }

      const Node &node = nodes[index];

      // Skip if no collision
      if (!aabb_intersect(box, get_node_aabb(index))) {
        continue;
      }

      if (node.is_leaf()) {
        collision_list.push_back(node.body_id);
      } else {
        stack.push(node.left);
        stack.push(node.right);
      }
    }

    return collision_list;
  }

  // create new node from rigidbody
  void insert(AABB box, int body_id) {

    // allocate new leaf
    int leaf_index = allocate_node();
    if (leaf_index < 0 || leaf_index >= nodes.size() || !nodes[leaf_index].is_leaf()) {
      // ?
      // body_node_map.erase(body_id);
      CLOGE("Failed to allocate new leaf node");
      return;
    }

    body_node_map[body_id] = leaf_index;
    // Node &leaf = nodes[leaf_index];
    nodes[leaf_index].body_id = body_id;
    nodes[leaf_index].height = 1;
    nodes[leaf_index].box = fatten(box, fat_margin); // store leaf aabb as fat box

    // base case -- zero leaves
    if (root_index == NULL_INDEX) {
      root_index = leaf_index;
      nodes[leaf_index].parent = NULL_INDEX;
      return;
    }

    // 1. -- find best leaf sibling for new leaf based on SA heuristic
    int sibling = find_best_sibling(leaf_index);

    // 2. -- create new internal parent node
    int old_parent = nodes[sibling].parent;
    int new_parent = allocate_node();
    // Node &internal = nodes[new_parent];
    nodes[new_parent].parent = old_parent;
    // internal.box = combine(nodes[leaf_index].fat_box, nodes[sibling].fat_box);
    nodes[new_parent].box = combine(get_node_aabb(leaf_index), get_node_aabb(sibling));

    if (old_parent != NULL_INDEX) {

      // old parent points to new parent
      if (nodes[old_parent].left == sibling) {
        nodes[old_parent].left = new_parent;
      } else {
        nodes[old_parent].right = new_parent;
      }
    }
    // sibling is root node
    else {
      root_index = new_parent;
    }

    nodes[new_parent].left = sibling;
    nodes[new_parent].right = leaf_index;
    nodes[sibling].parent = new_parent;
    nodes[leaf_index].parent = new_parent;

    // 3. -- walk up tree, refitting ancestor AABBs
    int index = new_parent;
    int itr = 0;
    while (index != NULL_INDEX) {
      int left = nodes[index].left;
      int right = nodes[index].right;

      // nodes[index].box = combine(nodes[left].box, nodes[right].box);
      nodes[index].box = combine(get_node_aabb(left), get_node_aabb(right));
      nodes[index].height = 1 + std::max(get_height(left), get_height(right));

      index = nodes[index].parent;
    }
  }

  void remove(int body_id) {
    auto itr = body_node_map.find(body_id);
    if (itr == body_node_map.end()) {
      return;
    }
    int leaf_index = itr->second;
    if (leaf_index < 0 || leaf_index >= nodes.size() || !nodes[leaf_index].is_leaf()) {
      // ?
      // body_node_map.erase(body_id);
      return;
    }

    if (leaf_index == root_index) {
      root_index = NULL_INDEX;
      body_node_map.erase(body_id);
      delete_node(leaf_index);
      return;
    }

    int parent = nodes[leaf_index].parent;   // internal node
    int grand_parent = nodes[parent].parent; // ? node
    int sibling = (nodes[parent].left == leaf_index) ? nodes[parent].right : nodes[parent].left;

    // check if parent is root node
    if (grand_parent != NULL_INDEX) {

      // reassign sibling as parent node
      if (nodes[grand_parent].left == parent) {
        nodes[grand_parent].left = sibling;
      } else {
        nodes[grand_parent].right = sibling;
      }
      nodes[sibling].parent = grand_parent;
      delete_node(parent); // remove parent internal node

      // refit ancestors
      int index = grand_parent;
      while (index != NULL_INDEX) {
        int left = nodes[index].left;
        int right = nodes[index].right;

        // nodes[index].box = combine(nodes[left].box, nodes[right].box);
        nodes[index].box = combine(get_node_aabb(left), get_node_aabb(right));
        nodes[index].height = 1 + std::max(get_height(left), get_height(right));

        index = nodes[index].parent;
      }

    } else {

      // siblinng becomes root node
      root_index = sibling;
      nodes[sibling].parent = NULL_INDEX;
      delete_node(parent);
    }

    body_node_map.erase(body_id);
    delete_node(leaf_index);
  }

  //  rebuild bvh to follow object movement
  std::vector<std::pair<int, int>> update(const std::vector<RigidBody> &bodies, Debugger *debug = nullptr) {
    std::vector<int> moved_leaves;
    std::vector<int> change_leaves;

    for (int i = 0; i < nodes.size(); i++) {

      if (debug) {
        debug->draw_aabb(nodes[i].box, glm::vec3(1.0f, 0.0f, 0.0f));
      }

      if (nodes[i].height < 0 || !nodes[i].is_leaf()) {
        continue;
      }

      int object_id = nodes[i].body_id;
      if (object_id < 0 || object_id >= bodies.size()) {
        continue;
      }

      AABB new_tight_box = bodies[object_id].collider.get_world_aabb(bodies[object_id].transform);
      if (!nodes[i].box.contains(new_tight_box)) {
        // needs to be rebuilt
        nodes[i].box = new_tight_box;
        moved_leaves.push_back(i);
      } else {
        // update internal aabb box
        change_leaves.push_back(i);
        // nodes[i].box = new_tight_box;
      }
    }

    for (int leaf_index : change_leaves) {
      int parent = nodes[leaf_index].parent;
      while (parent != NULL_INDEX) {
        int left = nodes[parent].left;
        int right = nodes[parent].right;
        AABB old_box = nodes[parent].box;
        nodes[parent].box = combine(get_node_aabb(left), get_node_aabb(right));

        // Early exit if AABB didn't change (common case)
        if (nodes[parent].box.min == old_box.min && nodes[parent].box.max == old_box.max) {
          break;
        }
        parent = nodes[parent].parent;
      }
    }

    for (int leaf_index : moved_leaves) {
      int object_id = nodes[leaf_index].body_id;
      AABB leaf_box = nodes[leaf_index].box;
      remove(object_id);
      insert(leaf_box, object_id);
    }

    // FIXME:
    return query_tree_pairs();
  }

  // FIXME: SLIDES say can be optimized
  bool raycast(const std::vector<RigidBody> &bodies, const Ray &ray, int &id, float &t, float tmax = 500.0f) {
    if (root_index == NULL_INDEX) {
      return false;
    }

    std::stack<int> stack;
    stack.push(root_index);
    while (!stack.empty()) {

      int index = stack.top();
      stack.pop();

      Node &node = nodes[index];

      // AABB ray test
      if (!node.box.ray_intersect(ray)) {
        continue;
      }

      if (node.is_leaf()) {

        int object_id = node.body_id;

        // ConvexHull ray test
        // FIXME:
        if (bodies[object_id].raycast(ray, t)) {
          id = object_id;
          return true;
        }

      } else {

        stack.push(node.left);
        stack.push(node.right);
      }
    }
    return false;
  }
};
