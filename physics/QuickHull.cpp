#include "QuickHull.hpp"
#include "ConvexMeshBuilder.hpp"

/*
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

/**
 * @brief Helper to build
 * @return ConvexMesh object with ...
 */
ConvexMesh QuickHull::build_convex_mesh(
    const std::vector<glm::vec3> &point_cloud, bool is_counter_clock_wise, bool use_original_indices, float epsilon) {
  build_mesh(point_cloud, epsilon);
  return ConvexMesh(mesh, vertex_data);
}

/**
 * @brief
 * @return
 */
std::array<float, 6> QuickHull::get_extrema_vertices() {
  std::array<float, 6> extrema_vertices;
  for (size_t i = 0; i < 6; i++) {
    const glm::vec3 &v = vertex_data[extrema_indices[i]];
    switch (i / 2) {
    case 0:
      extrema_vertices[i] = v.x;
      break;
    case 1:
      extrema_vertices[i] = v.y;
      break;
    case 2:
      extrema_vertices[i] = v.z;
      break;
    }
  }

  return extrema_vertices;
}

/**
 * @brief
 */
void QuickHull::build_mesh(const std::vector<glm::vec3> &point_cloud, float default_epsilon) {
  if (point_cloud.size() == 0) {
    mesh = ConvexMeshBuilder();
    return;
  }

  vertex_data = point_cloud;
  extrema_indices = find_extrema_points();
  scale = compute_point_cloud_scale();

  // Determines convexity tolerance of "fat planes"
  epsilon = default_epsilon * scale;
  epsilon_squared = epsilon * epsilon;

  // Planar case - all points lie on a 2D subspace of R^3
  is_planar = false;

  create_half_edge_mesh();

  if (is_planar) {
    const size_t new_point_index = temp_planar_vertices.size() - 1;
    for (auto &he : mesh.half_edges) {
      if (he.vert == new_point_index) {

        // Points to first vertex
        he.vert = 0;
      }
    }

    vertex_data = point_cloud;
    temp_planar_vertices.clear();
  }
}

/**
 * @brief Build simple tetrahedron from extrema
 */
void QuickHull::setup_initial_tetrahedron() {
  const size_t vertex_count = vertex_data.size();
  if (vertex_count == 0) {
    std::cerr << "Error: No vertices to process" << std::endl;
    return;
  }

  // 0. Degenerate case - return degenerate tetrahedron
  if (vertex_count <= 4) {
    size_t v[4] = {0,
        std::min((size_t)1, vertex_count - 1),
        std::min((size_t)2, vertex_count - 1),
        std::min((size_t)3, vertex_count - 1)};
    const glm::vec3 normal = get_triangle_normal(vertex_data[v[0]], vertex_data[v[1]], vertex_data[v[2]]);
    const Plane triangle_plane(normal, vertex_data[v[0]]);

    // normal points outwards
    if (triangle_plane.is_point_above_plane(vertex_data[v[3]])) {
      std::swap(v[0], v[1]);
    }

    return mesh.setup(v[0], v[1], v[2], v[3]);
  }

  // 1a. Build Line segment between two farthest extrema
  float max_distance = epsilon_squared;
  std::pair<size_t, size_t> selected_points;

  for (int i = 0; i < 6; i++) {
    if (extrema_indices[i] >= vertex_count) {
      std::cerr << "Error: extrema_indices[" << i << "] = " << extrema_indices[i] << " is out of range" << std::endl;
      continue;
    }

    for (int j = i + 1; j < 6; j++) {
      if (extrema_indices[j] >= vertex_count) {
        std::cerr << "Error: extrema_indices[" << j << "] = " << extrema_indices[j] << " is out of range" << std::endl;
        continue;
      }

      // Distance between extrema
      const float dist = glm::distance2(vertex_data[extrema_indices[i]], vertex_data[extrema_indices[j]]);

      if (dist > max_distance) {
        max_distance = dist;
        selected_points = {extrema_indices[i], extrema_indices[j]};
      }
    }
  }

  // 1b. Degenerate case - point cloud concentrated in single point
  if (max_distance == epsilon_squared) {
    return mesh.setup(0,
        std::min((size_t)1, vertex_count - 1),
        std::min((size_t)2, vertex_count - 1),
        std::min((size_t)3, vertex_count - 1));
  }
  assert(selected_points.first != selected_points.second);

  // 2a. Find point farthest from line
  max_distance = epsilon_squared;
  size_t max_index = std::numeric_limits<size_t>::max();

  Ray r(vertex_data[selected_points.first], (vertex_data[selected_points.second] - vertex_data[selected_points.first]));
  for (int i = 0; i < vertex_count; i++) {

    // Distance to line
    const float ray_distance = get_squared_distance_to_ray(vertex_data[i], r);

    if (ray_distance > max_distance) {
      max_distance = ray_distance;
      max_index = i;
    }
  }

  // 2b. Degenerate case - point cloud on 1D subspace of R^3
  if (max_distance == epsilon_squared) {
    // Pick unique point and return a thin triangle
    auto it = std::find_if(vertex_data.begin(), vertex_data.end(), [&](const glm::vec3 &ve) {
      return ve != vertex_data[selected_points.first] && ve != vertex_data[selected_points.second];
    });

    const size_t third_point =
        (it == vertex_data.end()) ? selected_points.first : std::distance(vertex_data.begin(), it);

    it = std::find_if(vertex_data.begin(), vertex_data.end(), [&](const glm::vec3 &ve) {
      return ve != vertex_data[selected_points.first] && ve != vertex_data[selected_points.second] &&
             ve != vertex_data[third_point];
    });

    const size_t fourth_point =
        (it == vertex_data.end()) ? selected_points.first : std::distance(vertex_data.begin(), it);

    return mesh.setup(selected_points.first, selected_points.second, third_point, fourth_point);
  }
  assert(max_index != selected_points.first && max_index != selected_points.second);

  // 2c. Build triangle face from point
  std::array<glm::vec3, 3> base_triangle{
      vertex_data[selected_points.first], vertex_data[selected_points.second], vertex_data[max_index]};
  std::array<size_t, 3> base_triangle_indices{selected_points.first, selected_points.second, max_index};

  // 3a. Find point farthest from triangle face plane
  max_distance = epsilon;
  max_index = 0;

  const glm::vec3 normal = get_triangle_normal(base_triangle[0], base_triangle[1], base_triangle[2]);
  const Plane plane(normal, base_triangle[0]);
  for (int i = 0; i < vertex_count; i++) {

    // Distance to plane
    const float plane_distance = std::abs(get_signed_distance_to_plane(vertex_data[i], plane));
    if (plane_distance > max_distance) {

      max_distance = plane_distance;
      max_index = i;
    }
  }

  // 3b. Degenerate case - point cloud lies on 2D subspace of R^3
  if (max_distance == epsilon) {
    is_planar = true;
    const glm::vec3 temp_normal = get_triangle_normal(base_triangle[1], base_triangle[2], base_triangle[0]);

    temp_planar_vertices.clear();
    temp_planar_vertices.insert(temp_planar_vertices.begin(), vertex_data.begin(), vertex_data.end());

    const glm::vec3 new_point = temp_normal + vertex_data[0];
    temp_planar_vertices.push_back(new_point);
    max_index = temp_planar_vertices.size() - 1;
    vertex_data = temp_planar_vertices;
  }

  // Enforce CCW winding - OpenGL considers all CCW polygons to be front-facing by default
  const Plane triangle_plane(normal, base_triangle[0]);
  if (triangle_plane.is_point_above_plane(vertex_data[max_index])) {
    std::swap(base_triangle_indices[0], base_triangle_indices[1]);
  }

  // 3c. Build tetrahedron half-edge mesh from point
  mesh.setup(base_triangle_indices[0], base_triangle_indices[1], base_triangle_indices[2], max_index);

  // 4a. Compute planes defined by each triangle face
  for (auto &face : mesh.faces) {

    auto face_vertices = mesh.get_face_vertices(face);
    const glm::vec3 &va = vertex_data[face_vertices[0]];
    const glm::vec3 &vb = vertex_data[face_vertices[1]];
    const glm::vec3 &vc = vertex_data[face_vertices[2]];

    const glm::vec3 vn = get_triangle_normal(va, vb, vc);
    const Plane plane(vn, va);

    face.plane = plane;
  }

  // 4b. Assign points to each face if above plane - ignore internal vertices
  for (int i = 0; i < vertex_count; i++) {
    for (auto &f : mesh.faces) {
      if (add_point_to_face(f, i)) {
        break;
      }
    }
  }
}

/**
 * @brief Iteratively add and merge new faces to convex mesh
 */
void QuickHull::create_half_edge_mesh() {

  visible_faces.clear();
  horizon_edges.clear();
  possible_visible_faces.clear();

  setup_initial_tetrahedron();
  assert(mesh.faces.size() == 4);

  if (mesh.faces.size() != 4) {
    std::cerr << "Error: Expected 4 faces after setup_initial_tetrahedron(), got " << mesh.faces.size() << std::endl;
    return;
  }

  face_stack.clear();
  for (size_t i = 0; i < 4; i++) {
    auto &f = mesh.faces[i];

    // Ensure face has a populated conflict list before pushing onto stack
    if (f.points_above_plane && f.points_above_plane->size() > 0) {
      face_stack.push_back(i);
      f.in_face_stack = 1;
    }
  }

  // Mark visited faces with current iteration count
  size_t iter = 0;
  size_t i = 0;
  while (!face_stack.empty()) {
    iter++;
    if (iter == std::numeric_limits<size_t>::max()) {
      // Max iter represents unvisited faces, thus reset counter
      iter = 0;
    }

    const size_t top_index = face_stack.front();
    face_stack.pop_front();

    auto &top_face = mesh.faces[top_index];
    top_face.in_face_stack = 0;

    assert(!top_face.points_above_plane || top_face.points_above_plane->size() > 0);

    // Ignore faces with empty conflict lists or disabled faces
    if (!top_face.points_above_plane || top_face.is_disabled()) {
      continue;
    }

    // Choose furthest point (eye point) as new potential vertex in convex hull
    const glm::vec3 &eye_point = vertex_data[top_face.farthest_point];
    const size_t eye_index = top_face.farthest_point;

    // Find all faces visible to eye point -- on positive side of face plane
    // Build a list of horizon edges
    horizon_edges.clear();
    possible_visible_faces.clear();
    visible_faces.clear();

    // Mark all faces as unvisited w/ numeric_limits<size_t>::max()
    possible_visible_faces.emplace_back(top_index, std::numeric_limits<size_t>::max());

    // Find all visible faces
    while (possible_visible_faces.size()) {

      const auto face_data = possible_visible_faces.back();
      possible_visible_faces.pop_back();

      auto &test_face = mesh.faces[face_data.face_index];
      assert(!test_face.is_disabled());

      // Face visibility already checked
      if (test_face.visibility_checked_on_iteration == iter) {

        if (test_face.is_visible_on_current_iteration) {
          continue;
        }
      } else {

        const Plane &plane = test_face.plane;
        test_face.visibility_checked_on_iteration = iter;
        const float dist = get_signed_distance_to_plane(eye_point, plane);

        // Point is visible if outside plane
        if (dist >= 0) {

          test_face.is_visible_on_current_iteration = 1;
          test_face.horizon_edges_on_current_iteration = 0;
          visible_faces.push_back(face_data.face_index);

          // Add/mark adjacent faces as possibly visible
          for (auto he : mesh.get_face_half_edges(test_face)) {
            if (mesh.half_edges[he].twin != face_data.entered_from_half_edge) {
              possible_visible_faces.emplace_back(mesh.half_edges[mesh.half_edges[he].twin].face, he);
            }
          }

          continue;
        }

        assert(face_data.face_index != top_index);
      }

      // Face is not visible, thus half edge is part of the horizon edge
      test_face.is_visible_on_current_iteration = 0;
      horizon_edges.push_back(face_data.entered_from_half_edge);

      // Get all half edges of non-visible face
      const auto half_edges =
          mesh.get_face_half_edges(mesh.faces[mesh.half_edges[face_data.entered_from_half_edge].face]);

      // Determine index of horizon half edge -- other half edges not part of
      // final mesh
      const std::int8_t he_index = (half_edges[0] == face_data.entered_from_half_edge)   ? 0
                                   : (half_edges[1] == face_data.entered_from_half_edge) ? 1
                                                                                         : 2;

      // Bitmask for horizon half edge, discard other half edges of non-visible
      // face
      mesh.faces[mesh.half_edges[face_data.entered_from_half_edge].face].horizon_edges_on_current_iteration |=
          (1 << he_index);
    }

    const size_t horizon_edges_count = horizon_edges.size();

    // Attempt to form loop between horizon edges
    if (!connect_horizon_edges(horizon_edges)) {

      std::cerr << "Failed to solve horizon edge." << std::endl;

      // Eye point is invalid and we don't add to convex hull
      auto it = std::find(top_face.points_above_plane->begin(), top_face.points_above_plane->end(), eye_index);

      // Erase eye point from future iterations
      top_face.points_above_plane->erase(it);
      if (top_face.points_above_plane->size() == 0) {
        reclaim_conflict_list(top_face.points_above_plane);
      }
      continue;
    }

    new_faces.clear();
    new_half_edges.clear();
    disabled_conflict_lists.clear();
    size_t disabled_count = 0;

    // Disable all visible faces and half-edges, we reuse std::vector memory
    // allocated to their assigned points
    for (auto face_index : visible_faces) {

      auto &disabled_face = mesh.faces[face_index];
      auto half_edges = mesh.get_face_half_edges(disabled_face);

      // disable all half edges part associated with face
      for (size_t j = 0; j < 3; j++) {
        // exclude horizon half edge -- part of final convex hull
        if ((disabled_face.horizon_edges_on_current_iteration & (1 << j)) == 0) {

          if (disabled_count < horizon_edges_count * 2) {
            new_half_edges.push_back(half_edges[j]);
            disabled_count++;
          } else {
            // disable half edge for future reuse
            mesh.disable_half_edge(half_edges[j]);
          }
        }
      }

      // Disable face
      auto ptr = mesh.disable_face(face_index);
      if (ptr) {

        // Retain pointer to conflict list for reassignment to new faces
        assert(ptr->size());
        disabled_conflict_lists.push_back(std::move(ptr));
      }
    }

    if (disabled_count < horizon_edges_count * 2) {

      const size_t new_half_edges_needed = horizon_edges_count * 2 - disabled_count;

      for (size_t i = 0; i < new_half_edges_needed; i++) {

        new_half_edges.push_back(mesh.add_half_edge());
      }
    }

    // Build new faces and half edges with horizon edge loop
    for (size_t i = 0; i < horizon_edges_count; i++) {

      // Existing half edge
      const size_t AB = horizon_edges[i];

      // Triangle vertices
      auto horizon_vertices = mesh.get_half_edge_vertices(mesh.half_edges[AB]);
      size_t A, B, C;

      A = horizon_vertices[0];
      B = horizon_vertices[1];
      C = eye_index;

      const size_t new_face_index = mesh.add_face();
      new_faces.push_back(new_face_index);

      // New half edges
      const size_t CA = new_half_edges[2 * i + 0];
      const size_t BC = new_half_edges[2 * i + 1];

      mesh.half_edges[AB].next = BC;
      mesh.half_edges[BC].next = CA;
      mesh.half_edges[CA].next = AB;

      mesh.half_edges[AB].prev = CA;
      mesh.half_edges[BC].prev = AB;
      mesh.half_edges[CA].prev = BC;

      mesh.half_edges[BC].face = new_face_index;
      mesh.half_edges[CA].face = new_face_index;
      mesh.half_edges[AB].face = new_face_index;

      mesh.half_edges[CA].vert = A;
      mesh.half_edges[BC].vert = C;

      // New face
      auto &new_face = mesh.faces[new_face_index];

      const glm::vec3 plane_normal = get_triangle_normal(vertex_data[A], vertex_data[B], eye_point);
      new_face.plane = Plane(plane_normal, eye_point);
      new_face.he = AB;

      mesh.half_edges[CA].twin = new_half_edges[i > 0 ? (i * 2 - 1) : (2 * horizon_edges_count - 1)];
      mesh.half_edges[BC].twin = new_half_edges[((i + 1) * 2) % (horizon_edges_count * 2)];
    }

    // Merge coplanar faces after adding new faces
    merge_new_faces(new_faces);

    // Assign disabled conflict lists to new faces
    for (auto &conflict_list : disabled_conflict_lists) {
      assert(conflict_list);

      for (const auto &point : *(conflict_list)) {

        // Don't assign eyePoint (part of convex hull now)
        if (point == eye_index) {
          continue;
        }

        //
        for (size_t j = 0; j < horizon_edges_count; j++) {
          // Skip if face was merged/deleted
          if (mesh.faces[new_faces[j]].is_disabled()) {
            continue;
          }

          if (add_point_to_face(mesh.faces[new_faces[j]], point)) {

            // Skip to next point if successfully assigned to conflict list
            break;
          }
        }
      }

      // Recycle std::vector memory for reuse -- points reassigned to to new
      // conflict list
      reclaim_conflict_list(conflict_list);
    }

    // Push new faces onto stack
    for (const auto new_face_index : new_faces) {

      auto &new_face = mesh.faces[new_face_index];
      if (new_face.points_above_plane) {

        assert(new_face.points_above_plane->size() > 0);
        if (!new_face.in_face_stack) {

          face_stack.push_back(new_face_index);
          new_face.in_face_stack = 1;
        }
      }
    }
  }

  conflict_list_pool.clear();
}

// Check if horizon edges form connected loop
bool QuickHull::connect_horizon_edges(std::vector<size_t> &horizon_edges) {

  const size_t horizon_edge_count = horizon_edges.size();
  for (size_t i = 0; i < horizon_edge_count - 1; i++) {

    // inital end vertex
    const size_t end_vertex = mesh.half_edges[horizon_edges[i]].vert;

    bool found_next = false;
    for (size_t j = i + 1; j < horizon_edge_count; j++) {

      // end vertex for twin edge (pointing in opposite direction to current
      // edge)
      const size_t start_vertex = mesh.half_edges[mesh.half_edges[horizon_edges[j]].twin].vert;

      // edges are connected
      if (start_vertex == end_vertex) {

        // reorder s.t. horizonEdges are in sequence
        std::swap(horizon_edges[i + 1], horizon_edges[j]);
        found_next = true;
        break;
      }
    }
    if (!found_next) {
      return false;
    }
  }

  // final vertex must match initial vertex
  assert(mesh.half_edges[horizon_edges[horizon_edges.size() - 1]].vert ==
         mesh.half_edges[mesh.half_edges[horizon_edges[0]].twin].vert);
  return true;
}

void QuickHull::merge_new_faces(const std::vector<size_t> &new_faces) {

  std::vector<size_t> edges;

  // Collect all edges from new faces
  for (size_t f_index : new_faces) {
    if (mesh.faces[f_index].is_disabled()) {
      continue;
    }

    auto half_edges = mesh.get_face_half_edges(mesh.faces[f_index]);
    for (size_t he_index : half_edges) {
      edges.push_back(he_index);
    }
  }

  // Sort by edge length (merge longer edges first for stability)
  std::sort(edges.begin(), edges.end(), [this](size_t he1, size_t he2) {
    auto v1 = mesh.get_half_edge_vertices(mesh.half_edges[he1]);
    auto v2 = mesh.get_half_edge_vertices(mesh.half_edges[he2]);

    float len1 = glm::length2(vertex_data[v1[1]] - vertex_data[v1[0]]);
    float len2 = glm::length2(vertex_data[v2[1]] - vertex_data[v2[0]]);

    return len1 > len2; // Longer edges first
  });

  // Iterate and merge
  for (size_t he_index : edges) {

    if (mesh.half_edges[he_index].is_disabled()) {
      continue;
    }

    size_t twin_index = mesh.half_edges[he_index].twin;
    if (mesh.half_edges[twin_index].is_disabled()) {
      continue;
    }

    size_t f1_index = mesh.half_edges[he_index].face;
    size_t f2_index = mesh.half_edges[twin_index].face;

    // Skip if faces are disabled or same
    if (mesh.faces[f1_index].is_disabled() || mesh.faces[f2_index].is_disabled() || f1_index == f2_index) {
      continue;
    }

    // Only merge non-convex/coplanar faces
    if (test_face_convexity(f1_index, f2_index)) {
      merge_nonconvex_faces(f1_index, f2_index, he_index);
    }
  }
}

void QuickHull::merge_nonconvex_faces(
    const size_t &absorbing_face_index, const size_t &deleted_face_index, const size_t &shared_edge_index) {

  // std::cout << "Merging faces: absorb=" << absorbing_face_index << " delete=" << deleted_face_index << "
  // shared_edge=" << shared_edge_index << std::endl;

  auto &absorb_face = mesh.faces[absorbing_face_index];
  auto &delete_face = mesh.faces[deleted_face_index];

  if (absorb_face.is_disabled() || delete_face.is_disabled()) {
    std::cerr << "ERROR: Attempting to merge disabled face" << std::endl;
    return;
  }

  auto &shared_edge = mesh.half_edges[shared_edge_index];
  size_t twin_index = shared_edge.twin; // CRITICAL: Store this before any modifications
  auto &twin_edge = mesh.half_edges[twin_index];

  // Verify edges belong to correct faces
  if (shared_edge.face != absorbing_face_index || twin_edge.face != deleted_face_index) {
    std::cerr << "ERROR: Shared edge doesn't belong to specified faces" << std::endl;
    std::cerr << "  shared_edge.face=" << shared_edge.face << " (expected " << absorbing_face_index << ")" << std::endl;
    std::cerr << "  twin_edge.face=" << twin_edge.face << " (expected " << deleted_face_index << ")" << std::endl;
    return;
  }

  // Set absorbing face reference to a non-deleted edge
  if (absorb_face.he == shared_edge_index) {
    absorb_face.he = shared_edge.prev;
  }

  // Reassign all edges from deleted face to absorbing face
  size_t current_half_edge = delete_face.he;
  size_t initial_half_edge = current_half_edge;
  size_t iter = 0;
  const size_t max_iter = mesh.half_edges.size();

  do {
    mesh.half_edges[current_half_edge].face = absorbing_face_index;
    current_half_edge = mesh.half_edges[current_half_edge].next;

    if (++iter > max_iter || current_half_edge >= mesh.half_edges.size()) {
      std::cerr << "ERROR: Invalid loop in face reassignment" << std::endl;
      return;
    }
  } while (current_half_edge != initial_half_edge);

  size_t prev_edge_index = shared_edge.next;
  size_t next_edge_index = twin_edge.next;

  // Find the edge before shared_edge in absorbing face
  size_t he_before_shared = absorb_face.he;
  iter = 0;
  while (mesh.half_edges[he_before_shared].next != shared_edge_index) {
    he_before_shared = mesh.half_edges[he_before_shared].next;

    if (++iter > max_iter || he_before_shared >= mesh.half_edges.size()) {
      std::cerr << "ERROR: Cannot find edge before shared (face=" << absorbing_face_index << " looking for edge "
                << shared_edge_index << ")" << std::endl;
      std::cerr << "  Started from he=" << absorb_face.he << std::endl;
      return;
    }
  }

  size_t he_before_twin = delete_face.he;
  iter = 0;
  while (mesh.half_edges[he_before_twin].next != twin_index) {
    he_before_twin = mesh.half_edges[he_before_twin].next;

    if (++iter > max_iter || he_before_twin >= mesh.half_edges.size()) {
      std::cerr << "ERROR: Cannot find edge before twin (face=" << deleted_face_index << " looking for edge "
                << twin_index << ")" << std::endl;
      std::cerr << "  Started from he=" << delete_face.he << std::endl;
      return;
    }
  }

  // Connect the loops:
  // absorbing_before -> deleted_after, deleted_before -> absorbing_after
  mesh.half_edges[he_before_shared].next = next_edge_index;
  mesh.half_edges[next_edge_index].prev = he_before_shared;

  mesh.half_edges[he_before_twin].next = prev_edge_index;
  mesh.half_edges[prev_edge_index].prev = he_before_twin;

  // Disable the shared edges
  mesh.disable_half_edge(shared_edge_index);
  mesh.disable_half_edge(twin_index); // FIXED: was twin_edge.twin

  // Rebuild face plane using Newell's method for polygonal face
  // (before handling conflict lists, which depend on the plane)
  std::vector<size_t> merged_vertices;
  size_t collect_he = absorb_face.he;
  size_t collect_initial = collect_he;
  size_t collect_iter = 0;

  do {
    merged_vertices.push_back(mesh.half_edges[collect_he].vert);
    collect_he = mesh.half_edges[collect_he].next;

    if (++collect_iter > max_iter || collect_he >= mesh.half_edges.size()) {
      std::cerr << "ERROR: Invalid loop collecting merged vertices" << std::endl;
      return;
    }
  } while (collect_he != collect_initial);

  // Update plane for merged polygonal face
  absorb_face.plane = compute_newell_plane(merged_vertices);

  // Now handle conflict lists: merge deleted face's conflict list into absorbing face
  auto deleted_conflict_list = mesh.disable_face(deleted_face_index);
  if (deleted_conflict_list && deleted_conflict_list->size() > 0) {
    if (!absorb_face.points_above_plane) {
      // Absorbing face has no conflict list, just take the deleted one
      absorb_face.points_above_plane = std::move(deleted_conflict_list);

      // Recompute furthest point with new plane
      absorb_face.farthest_distance = 0.0f;
      for (size_t point_idx : *absorb_face.points_above_plane) {
        float dist = get_signed_distance_to_plane(vertex_data[point_idx], absorb_face.plane);
        if (dist > absorb_face.farthest_distance) {
          absorb_face.farthest_distance = dist;
          absorb_face.farthest_point = point_idx;
        }
      }
    } else {
      // Merge the conflict lists
      absorb_face.points_above_plane->insert(
          absorb_face.points_above_plane->end(), deleted_conflict_list->begin(), deleted_conflict_list->end());

      // Recompute furthest point with merged list and new plane
      absorb_face.farthest_distance = 0.0f;
      for (size_t point_idx : *absorb_face.points_above_plane) {
        float dist = get_signed_distance_to_plane(vertex_data[point_idx], absorb_face.plane);
        if (dist > absorb_face.farthest_distance) {
          absorb_face.farthest_distance = dist;
          absorb_face.farthest_point = point_idx;
        }
      }

      // Reclaim the deleted list
      reclaim_conflict_list(deleted_conflict_list);
    }
  } else {
    // No conflict list from deleted face, but we still need to revalidate existing conflict list
    if (absorb_face.points_above_plane && absorb_face.points_above_plane->size() > 0) {
      absorb_face.farthest_distance = 0.0f;
      for (size_t point_idx : *absorb_face.points_above_plane) {
        float dist = get_signed_distance_to_plane(vertex_data[point_idx], absorb_face.plane);
        if (dist > absorb_face.farthest_distance) {
          absorb_face.farthest_distance = dist;
          absorb_face.farthest_point = point_idx;
        }
      }
    }
  }

  // std::cout << "  Merge complete: " << merged_vertices.size() << " vertices in merged face" << std::endl;
}

// Test convexity of faces
bool QuickHull::test_face_convexity(const size_t &face1_index, const size_t &face2_index) {

  auto &f1 = mesh.faces[face1_index];
  auto &f2 = mesh.faces[face2_index];

  if (f1.is_disabled() || f2.is_disabled()) {
    return false;
  }

  // Get face centers
  auto f1_vertices = mesh.get_face_vertices(f1);
  glm::vec3 f1_center(0.0f);
  for (size_t index : f1_vertices) {
    f1_center += vertex_data[index];
  }
  f1_center /= static_cast<float>(f1_vertices.size());

  auto f2_vertices = mesh.get_face_vertices(f2);
  glm::vec3 f2_center(0.0f);
  for (size_t index : f2_vertices) {
    f2_center += vertex_data[index];
  }
  f2_center /= static_cast<float>(f2_vertices.size());

  // Test if centers are below opposite planes
  float f1_distance = get_signed_distance_to_plane(f2_center, f1.plane);
  float f2_distance = get_signed_distance_to_plane(f1_center, f2.plane);

  // Edge is non-convex or coplanar if either center is above or on the opposite plane
  // Epsilon used for fat plane tolerance
  if (f1_distance >= -epsilon || f2_distance >= -epsilon) {
    return true; // Should merge faces
  }

  return false;
}

// Compute best-fit plane for polygonal face using Newell's method
Plane QuickHull::compute_newell_plane(const std::vector<size_t> &vertex_indices) {

  if (vertex_indices.size() < 3) {
    std::cerr << "ERROR: Cannot compute plane for face with < 3 vertices" << std::endl;
    return Plane(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(0.0f));
  }

  glm::vec3 normal = glm::vec3(0.0f);
  glm::vec3 centroid = glm::vec3(0.0f);

  size_t n = vertex_indices.size();
  for (size_t i = 0; i < n; i++) {
    const glm::vec3 &v_curr = vertex_data[vertex_indices[i]];
    const glm::vec3 &v_next = vertex_data[vertex_indices[(i + 1) % n]];

    // Newell's method: accumulate cross products
    normal.x += (v_curr.y - v_next.y) * (v_curr.z + v_next.z); // Projection on yz
    normal.y += (v_curr.z - v_next.z) * (v_curr.x + v_next.x); // Projection on xz
    normal.z += (v_curr.x - v_next.x) * (v_curr.y + v_next.y); // Projection on xy
    centroid += v_curr;
  }

  float normal_length = glm::length(normal);
  if (normal_length < 1e-10f) {
    // Degenerate case: vertices are collinear
    std::cerr << "WARNING: Collinear vertices in compute_newell_plane, using fallback" << std::endl;
    // Use cross product of first two edges as fallback
    glm::vec3 v0 = vertex_data[vertex_indices[0]];
    glm::vec3 v1 = vertex_data[vertex_indices[1]];
    glm::vec3 v2 = vertex_data[vertex_indices[2]];
    normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
  } else {
    normal = glm::normalize(normal);
  }

  centroid /= static_cast<float>(n);

  return Plane(normal, centroid);
}

std::array<size_t, 6> QuickHull::find_extrema_points() {

  std::array<size_t, 6> out_indices{0, 0, 0, 0, 0, 0};
  std::array<float, 6> extrema_vertices{
      vertex_data[0].x, vertex_data[0].x, vertex_data[0].y, vertex_data[0].y, vertex_data[0].z, vertex_data[0].z};

  for (size_t i = 1; i < vertex_data.size(); i++) {

    const glm::vec3 &pos = vertex_data[i];

    // X-axis
    if (pos.x > extrema_vertices[0]) {
      extrema_vertices[0] = pos.x;
      out_indices[0] = i;
    }

    else if (pos.x < extrema_vertices[1]) {
      extrema_vertices[1] = pos.x;
      out_indices[1] = i;
    }

    // Y-axis
    if (pos.y > extrema_vertices[2]) {
      extrema_vertices[2] = pos.y;
      out_indices[2] = i;
    }

    else if (pos.y < extrema_vertices[3]) {
      extrema_vertices[3] = pos.y;
      out_indices[3] = i;
    }

    // Z-axis
    if (pos.z > extrema_vertices[4]) {
      extrema_vertices[4] = pos.z;
      out_indices[4] = i;
    }

    else if (pos.z < extrema_vertices[5]) {
      extrema_vertices[5] = pos.z;
      out_indices[5] = i;
    }
  }

  return out_indices;
}

// Returns scale for computing epsilon
float QuickHull::compute_point_cloud_scale() {

  float s = 0.0f;
  for (int i = 0; i < 6; i++) {
    // raw pointer to ith extrema vertex data
    const float *v = (const float *)(&vertex_data[extrema_indices[i]]);

    // pointer offset for desired component of vertex  data -- [0, 1]
    // (x-component), [2, 3] (y-component), [4, 5] (z-component)
    v += i / 2;

    s = std::max(s, std::abs(*v));
  }

  return s;
}

// Adds point if above face plane and above epsilon tolerance
bool QuickHull::add_point_to_face(ConvexMeshBuilder::Face &face, size_t point_index) {

  // Negative dist means point is inside the hull
  const float plane_distance = get_signed_distance_to_plane(vertex_data[point_index], face.plane);

  // lies above plane and above epsilon tolerance -- |dist| is greater than
  // epsilon * ||normal||
  if (plane_distance > 0 && plane_distance * plane_distance > epsilon_squared * face.plane.normal_len_squared) {

    if (!face.points_above_plane) {

      // Reuse old conflict list if face has none
      face.points_above_plane = std::move(get_conflict_list());
    }

    // Push new point to conflict list
    face.points_above_plane->push_back(point_index);

    if (plane_distance > face.farthest_distance) {
      face.farthest_distance = plane_distance;
      face.farthest_point = point_index;
    }

    return true;
  }

  return false;
}

/**
 * @brief Reuses discarded conflict list memory
 */
std::unique_ptr<std::vector<size_t>> QuickHull::get_conflict_list() {

  // Allocate new memory if pool is empty
  if (conflict_list_pool.size() == 0) {
    return std::unique_ptr<std::vector<size_t>>(new std::vector<size_t>());
  }

  auto it = conflict_list_pool.end() - 1;

  std::unique_ptr<std::vector<size_t>> conflict_list_ptr = std::move(*it);
  conflict_list_pool.erase(it);

  conflict_list_ptr->clear();
  return conflict_list_ptr;
}

/**
 * @brief Recycles conflict list memory to pool when face is removed
 */
void QuickHull::reclaim_conflict_list(std::unique_ptr<std::vector<size_t>> &ptr) {

  const size_t size = ptr->size();

  //
  if ((size + 1) * 128 < ptr->capacity()) {

    ptr.reset(nullptr);
    return;
  }

  conflict_list_pool.push_back(std::move(ptr));
}
