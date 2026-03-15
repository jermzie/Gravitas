#include "Broadphase.hpp"
#include "RigidBody.hpp"

/**
 * @brief Implements Branch and Bound algo to optimize global search for sibling
 * node
 * @return int idx of sibling node
 */
int Broadphase::find_best_sibling(int leaf_idx) {
  if (root_idx == NULL_INDEX) {
    return NULL_INDEX;
  }

  const AABB &leaf_box = nodes[leaf_idx].box;
  float leaf_sa = leaf_box.surface_area();

  struct Candidate {
    int idx;
    float min_cost;
    float inherited_cost;
    bool operator>(const Candidate &o) const { return min_cost > o.min_cost; }
  };

  std::priority_queue<Candidate, std::vector<Candidate>,
                      std::greater<Candidate>>
      pq;

  float best_cost = combine(get_node_aabb(root_idx), leaf_box).surface_area();
  int best_sibling = root_idx;

  pq.push({root_idx, 0.0f, 0.0f});

  while (!pq.empty()) {
    Candidate c = pq.top();
    pq.pop();

    if (c.min_cost >= best_cost) {
      continue;
    }

    // get AABB
    const Node &n = nodes[c.idx];
    AABB potential_sibling = combine(get_node_aabb(c.idx), leaf_box);
    float direct_cost = potential_sibling.surface_area();
    float cost = direct_cost + c.inherited_cost;

    if (cost < best_cost) {
      best_cost = cost;
      best_sibling = c.idx;
    }

    // leaf nodes don't contribute to tree cost
    if (n.is_leaf()) {
      continue;
    }

    // calculate new inherited cost for child nodes
    float new_inherited_cost =
        c.inherited_cost + direct_cost - get_node_aabb(c.idx).surface_area();
    float min_cost = leaf_sa + new_inherited_cost;

    if (min_cost < best_cost && !n.is_leaf()) {
      pq.push({n.left, min_cost, new_inherited_cost});
      pq.push({n.right, min_cost, new_inherited_cost});
    }
  }

  return best_sibling;
}

// compute surface area cost of tree
float Broadphase::compute_cost(int idx) {
  float cost = 0.0f;

  std::stack<int> stack;
  const Node &n = nodes[idx];
  stack.push(n.left);
  stack.push(n.right);
  for (int i = 0; i < node_count; i++) {
    if (nodes[i].height < 0) {
      continue;
    }

    // leaf nodes don't contribute to tree cost
    if (!nodes[i].is_leaf()) {
      cost += nodes[i].box.surface_area();
    }
  }
  return cost;
}

// Surface area based balancing (BVH is SA-based)
void Broadphase::balance_tree(int idx) {
  if (idx == NULL_INDEX || nodes[idx].is_leaf() || nodes[idx].height < 2) {
    return;
  }

  Node &parent_node = nodes[idx];

  int left = parent_node.left;
  int right = parent_node.right;
  Node &left_node = nodes[left];
  Node &right_node = nodes[right];

  if (left == NULL_INDEX || right == NULL_INDEX) {
    return;
  }

  // leaf nodes don't contribute to tree cost
  if (!left_node.is_leaf()) {

    int sub_left = left_node.left;   // LL
    int sub_right = left_node.right; // LR

    // Combine LR with R
    AABB swap = combine(get_node_aabb(sub_right), right_node.box);

    // (LR + R) < (LR + LL)
    if (swap.surface_area() < left_node.box.surface_area()) {

      // swap LL with R
      parent_node.right = sub_left;
      nodes[sub_left].parent = idx;

      left_node.left = right;
      right_node.parent = left;

      // refit parent
      left_node.box = swap;
      parent_node.box = combine(left_node.box, get_node_aabb(sub_left));
      return;
    }

    // Combine LL with R
    swap = combine(get_node_aabb(sub_left), right_node.box);

    // (LL + R) < (LL + LR)
    if (swap.surface_area() < left_node.box.surface_area()) {

      // swap LR with R
      parent_node.right = sub_right;
      nodes[sub_right].parent = idx;

      left_node.right = right;
      right_node.parent = left;

      // refit parent
      left_node.box = swap;
      parent_node.box = combine(left_node.box, get_node_aabb(sub_right));
      return;
    }
  }

  // leaf nodes don't contribute to tree cost
  if (!right_node.is_leaf()) {

    int sub_left = right_node.left;   // RL
    int sub_right = right_node.right; // RR

    // Combine RR with L
    AABB swap = combine(get_node_aabb(sub_right), left_node.box);

    // (RR + L) < (RR + RL)
    if (swap.surface_area() < right_node.box.surface_area()) {

      // swap RL with L
      parent_node.left = sub_left;
      nodes[sub_left].parent = idx;

      right_node.left = left;
      left_node.parent = right;

      // refit parent
      right_node.box = swap;
      parent_node.box = combine(right_node.box, get_node_aabb(sub_left));
      return;
    }

    // Combine RL with L
    swap = combine(get_node_aabb(sub_left), left_node.box);

    // (RL + L) < (RL + RR)
    if (swap.surface_area() < right_node.box.surface_area()) {

      // swap RR with L
      parent_node.left = sub_right;
      nodes[sub_right].parent = idx;

      right_node.right = left;
      left_node.parent = right;

      // refit parent
      right_node.box = swap;
      parent_node.box = combine(right_node.box, get_node_aabb(sub_right));
      return;
    }
  }
}

void Broadphase::validate(int idx) {
  if (idx == -2) {
    idx = root_idx;
  }
  if (idx == NULL_INDEX) {
    return;
  }

  const Node &n = nodes.at(idx);
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
  assert(nodes.at(left).parent == idx);
  assert(nodes.at(right).parent == idx);

  int expected_height = 1 + std::max(get_height(left), get_height(right));
  assert(n.height == expected_height);

  // TODO: BAD AVL BALANCING STRATEGY
  // int bf = nodes.at(left).height - nodes.at(right).height;
  // assert(bf >= -1 && bf <= 1);

  validate(left);
  validate(right);
}

/**
 * @brief Compute possible collision pairs between all bodies
 * @return std::vector<std::pair<int, int>> array of collision pairs
 */
std::vector<std::pair<int, int>> Broadphase::query_tree_pairs() {
  std::vector<std::pair<int, int>> collision_list;
  if (root_idx == NULL_INDEX) {
    return collision_list;
  }

  const Node &root = nodes[root_idx];

  // Single body - no possible collisions
  if (root.is_leaf()) {
    return collision_list;
  }

  std::stack<std::pair<int, int>> stack;

  // Push root's children onto stack
  if (root.left != NULL_INDEX)
    stack.push({root.left, root.left});
  if (root.right != NULL_INDEX)
    stack.push({root.right, root.right});
  if (root.left != NULL_INDEX && root.right != NULL_INDEX)
    stack.push({root.left, root.right});

  while (!stack.empty()) {

    std::pair<int, int> pair = stack.top();
    stack.pop();

    if (pair.first == NULL_INDEX || pair.second == NULL_INDEX) {
      continue;
    }
    if (pair.first >= nodes.size() || pair.second >= nodes.size()) {
      continue;
    }

    if (pair.first == pair.second) {
      const Node &node = nodes[pair.first];

      if (!node.is_leaf()) {

        // Test children against each other and themselves
        // WARNING: QUESTIONABLE PERFORMANCE - DUPE COLL. CHECKS (A, B) vs (B,
        // A)
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
    if (!aabb_intersect(get_node_aabb(pair.first),
                        get_node_aabb(pair.second))) {
      continue;
    }

    bool a_leaf = a_node.is_leaf();
    bool b_leaf = b_node.is_leaf();

    // Both leaves - POTENTIAL COLLISION
    if (a_leaf && b_leaf) {

      // make sure not same node
      if (a_node.body_id != b_node.body_id) {
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
/**
 * @brief Compute possible collision pairs between given AABB
 * @return std::vector<int> array of rigid body IDs
 */
std::vector<int> Broadphase::query_node(const AABB &box) const {
  std::vector<int> collision_list;
  if (root_idx == NULL_INDEX) {
    return collision_list;
  }

  std::stack<int> stack;
  stack.push(root_idx);

  while (!stack.empty()) {
    int idx = stack.top();
    stack.pop();

    if (idx == NULL_INDEX) {
      continue;
    }

    const Node &node = nodes[idx];

    // Skip if no collision
    if (!aabb_intersect(box, get_node_aabb(idx))) {
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

/**
 * @brief Insert node and refit tree
 * @return void
 */
void Broadphase::insert(AABB box, int body_id) {

  // allocate new leaf
  int leaf_idx = allocate_node();

  body_node_map[body_id] = leaf_idx;

  // WARNING: why is this bad again?
  // Node &leaf = nodes[leaf_idx];
  nodes[leaf_idx].body_id = body_id;
  nodes[leaf_idx].height = 1;
  nodes[leaf_idx].box = fatten(box, fat_margin); // store leaf aabb as fat box

  // base case - zero leaves
  if (root_idx == NULL_INDEX) {
    root_idx = leaf_idx;
    nodes[leaf_idx].parent = NULL_INDEX;
    return;
  }

  // 1. find best leaf sibling for new leaf based on SA heuristic
  int sibling = find_best_sibling(leaf_idx);

  // 2. create new internal parent node
  int old_parent = nodes[sibling].parent;
  int new_parent = allocate_node();
  // Node &internal = nodes[new_parent];
  nodes[new_parent].parent = old_parent;
  // internal.box = combine(nodes[leaf_idx].fat_box, nodes[sibling].fat_box);
  nodes[new_parent].box =
      combine(get_node_aabb(leaf_idx), get_node_aabb(sibling));

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
    root_idx = new_parent;
  }

  nodes[new_parent].left = sibling;
  nodes[new_parent].right = leaf_idx;
  nodes[sibling].parent = new_parent;
  nodes[leaf_idx].parent = new_parent;

  // 3. walk up tree, refitting ancestor AABBs
  int idx = new_parent;
  int itr = 0;
  while (idx != NULL_INDEX) {
    int left = nodes[idx].left;
    int right = nodes[idx].right;

    nodes[idx].box = combine(get_node_aabb(left), get_node_aabb(right));
    nodes[idx].height = 1 + std::max(get_height(left), get_height(right));

    balance_tree(idx);

    idx = nodes[idx].parent;
  }
}

void Broadphase::remove(int body_id) {
  auto itr = body_node_map.find(body_id);
  if (itr == body_node_map.end()) {
    return;
  }
  int leaf_idx = itr->second;
  if (leaf_idx < 0 || leaf_idx >= nodes.size() || !nodes[leaf_idx].is_leaf()) {
    // ?
    // body_node_map.erase(body_id);
    return;
  }

  if (leaf_idx == root_idx) {
    root_idx = NULL_INDEX;
    body_node_map.erase(body_id);
    delete_node(leaf_idx);
    return;
  }

  int parent = nodes[leaf_idx].parent;     // internal node
  int grand_parent = nodes[parent].parent; // ? node
  int sibling = (nodes[parent].left == leaf_idx) ? nodes[parent].right
                                                 : nodes[parent].left;

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
    int idx = grand_parent;
    while (idx != NULL_INDEX) {
      int left = nodes[idx].left;
      int right = nodes[idx].right;

      // nodes[idx].box = combine(nodes[left].box, nodes[right].box);
      nodes[idx].box = combine(get_node_aabb(left), get_node_aabb(right));
      nodes[idx].height = 1 + std::max(get_height(left), get_height(right));

      idx = nodes[idx].parent;
    }

  } else {

    // siblinng becomes root node
    root_idx = sibling;
    nodes[sibling].parent = NULL_INDEX;
    delete_node(parent);
  }

  body_node_map.erase(body_id);
  delete_node(leaf_idx);
}

//  rebuild bvh to follow object movement
std::vector<std::pair<int, int>>
Broadphase::update(const std::vector<RigidBody> &bodies, Debugger *debug) {
  std::vector<int> moved_leaves;
  std::vector<int> change_leaves;

  for (int i = 0; i < nodes.size(); i++) {

    if (debug) {
      if (nodes[i].is_leaf()) {
        debug->draw_aabb(nodes[i].box, glm::vec4(1, 0, 0, 0.5f));
      } else if (root_idx == i) {
        debug->draw_aabb(nodes[i].box, glm::vec4(0, 0, 1, 0.5f));
      } else {
        debug->draw_aabb(nodes[i].box, glm::vec4(0, 1, 0, 0.5f));
      }
    }

    if (nodes[i].height < 0 || !nodes[i].is_leaf()) {
      continue;
    }

    int id = nodes[i].body_id;
    if (id < 0 || id >= bodies.size()) {
      continue;
    }

    AABB new_tight_box =
        bodies[id].collider.get_world_aabb(bodies[id].transform);

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

  for (int leaf_idx : change_leaves) {
    int parent = nodes[leaf_idx].parent;
    while (parent != NULL_INDEX) {
      int left = nodes[parent].left;
      int right = nodes[parent].right;
      AABB old_box = nodes[parent].box;
      nodes[parent].box = combine(get_node_aabb(left), get_node_aabb(right));

      // Early exit if AABB didn't change (common case)
      if (nodes[parent].box.min == old_box.min &&
          nodes[parent].box.max == old_box.max) {
        break;
      }
      parent = nodes[parent].parent;
    }
  }

  for (int leaf_idx : moved_leaves) {
    int body_id = nodes[leaf_idx].body_id;
    AABB leaf_box = nodes[leaf_idx].box;
    remove(body_id);
    insert(leaf_box, body_id);
  }

  return query_tree_pairs();
}

bool Broadphase::raycast(const std::vector<RigidBody> &bodies, const Ray &ray,
                         int &id, float &t, float tmax) {
  if (root_idx == NULL_INDEX) {
    return false;
  }

  std::stack<int> stack;
  stack.push(root_idx);
  while (!stack.empty()) {

    int idx = stack.top();
    stack.pop();

    Node &node = nodes[idx];

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
