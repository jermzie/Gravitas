#include "QuickHull.hpp"

/*

WRITE THIS SHIT FROM SCRATCH FIRST (DRAFT)
YOUR OWN IMPLEMENTATION

OPTIMIZE LATER

Convex Hull w/ QuickHull Algo

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

/*
CollisionGeometry
QuickHull::getConvexHull(const std::vector<glm::vec3> &pointCloud, bool CCW,
                         bool useOriginalIndices, float epsilon) {

  buildMesh(pointCloud, epsilon);
  // debugState();
  return ConvexHull(mesh, vertexData, CCW, useOriginalIndices);
}

HalfEdgeMesh
QuickHull::getHalfEdgeMesh(const std::vector<glm::vec3> &pointCloud, bool CCW,
                           float epsilon) {

  buildMesh(pointCloud, epsilon);
  return HalfEdgeMesh(mesh, vertexData);
}
*/

ConvexMesh
QuickHull::build_convex_mesh(const std::vector<glm::vec3> &point_cloud,
                             bool is_counter_clock_wise,
                             bool use_original_indices, float epsilon) {
  build_mesh(point_cloud, epsilon);
  return ConvexMesh(mesh, vertex_data);
}

/*
std::array<float, 6> QuickHull::getExtremaVertices() {

        std::array<float, 6> extremaValues;
        for (size_t i = 0; i < 6; i++) {

                // pointer to ith extrema vertex
                const float *v = (const float*)(&vertexData[extremaIndices[i]]);

                // integer division maps index i to each component -- [0, 1]
(pick x-component), [2, 3] (pick y-component), [4, 5] (pick z-component) v += i
/ 2; extremaValues[i] = *v;
        }

        return extremaValues;
}
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

void QuickHull::build_mesh(const std::vector<glm::vec3> &point_cloud,
                           float default_epsilon) {

  if (point_cloud.size() == 0) {

    mesh = ConvexMeshBuilder();
    return;
  }
  vertex_data = point_cloud;

  // Find extreme values and use them to compute scale of point cloud
  extrema_indices = get_extrema();
  scale = get_scale();

  // Epsilon and scale determines tolerance of "fat planes"
  epsilon = default_epsilon * scale;
  epsilon_squared = epsilon * epsilon;

  // Planar case -- when all points lie on a 2D subspace of R^3
  is_planar = false;

  create_half_edge_mesh(); // Iteratively update mesh until ...

  // Points seem to lie on a 2D subspace of R^3
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

// Forms inital hull from extreme values
void QuickHull::setup_initial_tetrahedron() {

  const size_t vertex_count = vertex_data.size();

  if (vertex_count == 0) {
    std::cerr << "Error: No vertices to process" << std::endl;
    return;
  }

  // 0. Degenerate case -- just return degenerate tetrahedron
  if (vertex_count <= 4) {
    size_t v[4] = {0, std::min((size_t)1, vertex_count - 1),
                   std::min((size_t)2, vertex_count - 1),
                   std::min((size_t)3, vertex_count - 1)};
    const glm::vec3 normal = get_triangle_normal(
        vertex_data[v[0]], vertex_data[v[1]], vertex_data[v[2]]);
    const Plane triangle_plane(normal, vertex_data[v[0]]);

    // normal should point outwards away from tetrahedrons
    if (triangle_plane.is_point_above_plane(vertex_data[v[3]])) {
      std::swap(v[0], v[1]);
    }

    return mesh.setup(v[0], v[1], v[2], v[3]);
  }

  // 1a. Form a line between two furthest extrema vertices
  float max_distance = epsilon_squared;
  std::pair<size_t, size_t> selected_points;

  for (int i = 0; i < 6; i++) {
    // Check if extremaIndices[i] is valid
    if (extrema_indices[i] >= vertex_count) {
      std::cerr << "Error: extrema_indices[" << i
                << "] = " << extrema_indices[i]
                << " is out of range for vertex_data size " << vertex_count
                << std::endl;
      continue;
    }

    for (int j = i + 1; j < 6; j++) {

      // Check if extremaIndices[j] is valid
      if (extrema_indices[j] >= vertex_count) {
        std::cerr << "Error: extrema_indices[" << j
                  << "] = " << extrema_indices[j]
                  << " is out of range for vertex_data size " << vertex_count
                  << std::endl;
        continue;
      }

      const float dist = glm::distance2(
          vertex_data[extrema_indices[i]],
          vertex_data[extrema_indices[j]]); // get squared distance between
                                            // extreme values

      if (dist > max_distance) {
        max_distance = dist;
        selected_points = {extrema_indices[i], extrema_indices[j]};
      }
    }
  }

  // 1b. Degenerate case -- point cloud seems to consist of a single point
  if (max_distance == epsilon_squared) {
    std::cerr << "1b. error \n";
    return mesh.setup(0, std::min((size_t)1, vertex_count - 1),
                      std::min((size_t)2, vertex_count - 1),
                      std::min((size_t)3, vertex_count - 1));
  }
  assert(selected_points.first != selected_points.second);

  // 2a. Find point furthest from the line -- forms a triangle face
  max_distance = epsilon_squared;
  size_t max_index = std::numeric_limits<size_t>::max();

  Ray r(vertex_data[selected_points.first],
        (vertex_data[selected_points.second] -
         vertex_data[selected_points.first]));

  for (int i = 0; i < vertex_count; i++) {

    const float ray_distance = get_squared_distance_to_ray(vertex_data[i], r);

    if (ray_distance > max_distance) {
      max_distance = ray_distance;
      max_index = i;
    }
  }

  // 2b. Degenerate case -- point cloud seems to belong to 1D subspace of R^3
  if (max_distance == epsilon_squared) {
    std::cerr << "2b. error \n";
    // Pick any distinct point and return a thin triangle
    auto it = std::find_if(vertex_data.begin(), vertex_data.end(),
                           [&](const glm::vec3 &ve) {
                             return ve != vertex_data[selected_points.first] &&
                                    ve != vertex_data[selected_points.second];
                           });

    const size_t third_point = (it == vertex_data.end())
                                   ? selected_points.first
                                   : std::distance(vertex_data.begin(), it);

    it = std::find_if(vertex_data.begin(), vertex_data.end(),
                      [&](const glm::vec3 &ve) {
                        return ve != vertex_data[selected_points.first] &&
                               ve != vertex_data[selected_points.second] &&
                               ve != vertex_data[third_point];
                      });

    const size_t fourth_point = (it == vertex_data.end())
                                    ? selected_points.first
                                    : std::distance(vertex_data.begin(), it);

    return mesh.setup(selected_points.first, selected_points.second,
                      third_point, fourth_point);
  }
  assert(max_index != selected_points.first &&
         max_index != selected_points.second);

  // 2c. Forms base triangle for tetrahedron
  std::array<glm::vec3, 3> base_triangle{vertex_data[selected_points.first],
                                         vertex_data[selected_points.second],
                                         vertex_data[max_index]};

  std::array<size_t, 3> base_triangle_indices{
      selected_points.first, selected_points.second, max_index};

  // 3a. Find point furthest from the plane (forms a tetrahedron)
  max_distance = epsilon;
  max_index = 0;
  const glm::vec3 normal =
      get_triangle_normal(base_triangle[0], base_triangle[1], base_triangle[2]);
  const Plane plane(normal, base_triangle[0]);
  for (int i = 0; i < vertex_count; i++) {

    const float plane_distance =
        std::abs(get_signed_distance_to_plane(vertex_data[i], plane));
    if (plane_distance > max_distance) {

      max_distance = plane_distance;
      max_index = i;
    }
  }

  // 3b. Degenerate case -- point cloud seems to lie on 2D subspace of R^3
  if (max_distance == epsilon) {
    std::cerr << "3b. error \n";
    is_planar = true;
    const glm::vec3 temp_normal = get_triangle_normal(
        base_triangle[1], base_triangle[2], base_triangle[0]);

    temp_planar_vertices.clear();
    temp_planar_vertices.insert(temp_planar_vertices.begin(),
                                vertex_data.begin(), vertex_data.end());

    const glm::vec3 new_point = temp_normal + vertex_data[0];
    temp_planar_vertices.push_back(new_point);
    max_index = temp_planar_vertices.size() - 1;
    vertex_data = temp_planar_vertices;
  }

  // Enforce CCW winding -- OpenGL considers all CCW polygons to be front-facing
  // by default
  const Plane triangle_plane(normal, base_triangle[0]);
  if (triangle_plane.is_point_above_plane(vertex_data[max_index])) {
    std::swap(base_triangle_indices[0], base_triangle_indices[1]);
  }

  // 3c. Create initial tetrahedron half edge mesh
  mesh.setup(base_triangle_indices[0], base_triangle_indices[1],
             base_triangle_indices[2], max_index);

  // Compute planes defined by each triangle face
  for (auto &face : mesh.faces) {

    auto face_vertices = mesh.get_face_vertices(face);
    const glm::vec3 &va = vertex_data[face_vertices[0]];
    const glm::vec3 &vb = vertex_data[face_vertices[1]];
    const glm::vec3 &vc = vertex_data[face_vertices[2]];

    const glm::vec3 vn = get_triangle_normal(va, vb, vc);
    const Plane plane(vn, va);

    face.plane = plane;
  }

  // 4. Assign points to each face if "outside" -- vertices inside tetrahedron
  // are ignored
  for (int i = 0; i < vertex_count; i++) {
    for (auto &f : mesh.faces) {

      // what if Point is coplanar???
      if (add_point_to_face(f, i)) {
        break;
      }
    }
  }
}

// Iterative hull construction
void QuickHull::create_half_edge_mesh() {

  visible_faces.clear();
  horizon_edges.clear();
  possible_visible_faces.clear();

  // Compute base tetrahedron
  setup_initial_tetrahedron();
  assert(mesh.faces.size() == 4);

  if (mesh.faces.size() != 4) {
    std::cerr
        << "Error: Expected 4 faces after setup_initial_tetrahedron(), got "
        << mesh.faces.size() << std::endl;
    return;
  }

  // Initialize face stack w/ newly assigned conflict lists
  face_stack.clear();
  for (size_t i = 0; i < 4; i++) {

    auto &f = mesh.faces[i];

    // Ensure face has a populated conflict list before pushing onto stack
    if (f.points_on_positive_side && f.points_on_positive_side->size() > 0) {
      face_stack.push_back(i);
      f.in_face_stack = 1;
    }
  }

  // Iterate over face stack until no "outside" points exist
  // Mark visited faces with current iteration counter
  size_t iter = 0;
  size_t i = 0;
  while (!face_stack.empty()) {
    iter++;
    if (iter == std::numeric_limits<size_t>::max()) {
      // Max iter represents unvisited faces, thus reset counter
      iter = 0;
    }

    // Pop top face off stack
    const size_t top_index = face_stack.front();
    face_stack.pop_front();

    auto &top_face = mesh.faces[top_index];
    top_face.in_face_stack = 0;

    assert(!top_face.points_on_positive_side ||
           top_face.points_on_positive_side->size() > 0);

    // Ignore faces with empty conflict lists or disabled faces
    if (!top_face.points_on_positive_side || top_face.is_disabled()) {
      continue;
    }

    // Choose furthest point (eye point) as new potential vertex in convex hull
    const glm::vec3 &eye_point = vertex_data[top_face.furthest_point];
    const size_t eye_index = top_face.furthest_point;

    // Find all faces visible to eye point -- on positive side of face plane
    // Build a list of horizon edges
    horizon_edges.clear();
    possible_visible_faces.clear();
    visible_faces.clear();

    // Mark all faces as unvisited w/ numeric_limits<size_t>::max()
    possible_visible_faces.emplace_back(top_index,
                                        std::numeric_limits<size_t>::max());

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
              possible_visible_faces.emplace_back(
                  mesh.half_edges[mesh.half_edges[he].twin].face, he);
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
      const auto half_edges = mesh.get_face_half_edges(
          mesh.faces[mesh.half_edges[face_data.entered_from_half_edge].face]);

      // Determine index of horizon half edge -- other half edges not part of
      // final mesh
      const std::int8_t he_index =
          (half_edges[0] == face_data.entered_from_half_edge)   ? 0
          : (half_edges[1] == face_data.entered_from_half_edge) ? 1
                                                                : 2;

      // Bitmask for horizon half edge, discard other half edges of non-visible
      // face
      mesh.faces[mesh.half_edges[face_data.entered_from_half_edge].face]
          .horizon_edges_on_current_iteration |= (1 << he_index);
    }

    const size_t horizon_edges_count = horizon_edges.size();

    // Attempt to form loop between horizon edges
    if (!connect_horizon_edges(horizon_edges)) {

      std::cerr << "Failed to solve horizon edge." << std::endl;

      // Eye point is invalid and we don't add to convex hull
      auto it = std::find(top_face.points_on_positive_side->begin(),
                          top_face.points_on_positive_side->end(), eye_index);

      // Erase eye point from future iterations
      top_face.points_on_positive_side->erase(it);
      if (top_face.points_on_positive_side->size() == 0) {
        reclaim_conflict_list(top_face.points_on_positive_side);
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
        if ((disabled_face.horizon_edges_on_current_iteration & (1 << j)) ==
            0) {

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

      const size_t new_half_edges_needed =
          horizon_edges_count * 2 - disabled_count;

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

      mesh.half_edges[BC].face = new_face_index;
      mesh.half_edges[CA].face = new_face_index;
      mesh.half_edges[AB].face = new_face_index;

      mesh.half_edges[CA].vert = A;
      mesh.half_edges[BC].vert = C;

      // New face
      auto &new_face = mesh.faces[new_face_index];

      const glm::vec3 plane_normal =
          get_triangle_normal(vertex_data[A], vertex_data[B], eye_point);
      new_face.plane = Plane(plane_normal, eye_point);
      new_face.he = AB;

      mesh.half_edges[CA].twin =
          new_half_edges[i > 0 ? (i * 2 - 1) : (2 * horizon_edges_count - 1)];
      mesh.half_edges[BC].twin =
          new_half_edges[((i + 1) * 2) % (horizon_edges_count * 2)];
    }

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

    // Push new faces onto faceStack
    for (const auto new_face_index : new_faces) {

      auto &new_face = mesh.faces[new_face_index];
      if (new_face.points_on_positive_side) {

        assert(new_face.points_on_positive_side->size() > 0);
        if (!new_face.in_face_stack) {

          face_stack.push_back(new_face_index);
          new_face.in_face_stack = 1;
        }
      }
    }

    /*getConvexHull(vertexData);
    std::string tmp = std::to_string(i++);
    writeOBJ(tmp + ".obj");*/
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
      const size_t start_vertex =
          mesh.half_edges[mesh.half_edges[horizon_edges[j]].twin].vert;

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

// HELPER FUNCTIONS

// Returns indices to extreme values
std::array<size_t, 6> QuickHull::get_extrema() {

  std::array<size_t, 6> out_indices{0, 0, 0, 0, 0, 0};
  std::array<float, 6> extrema_vertices{vertex_data[0].x, vertex_data[0].x,
                                        vertex_data[0].y, vertex_data[0].y,
                                        vertex_data[0].z, vertex_data[0].z};

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
float QuickHull::get_scale() {

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
bool QuickHull::add_point_to_face(ConvexMeshBuilder::Face &face,
                                  size_t pointIdx) {

  // Negative dist means point is inside the hull
  const float plane_distance =
      get_signed_distance_to_plane(vertex_data[pointIdx], face.plane);

  // lies above plane and above epsilon tolerance -- |dist| is greater than
  // epsilon * ||normal||
  if (plane_distance > 0 &&
      plane_distance * plane_distance >
          epsilon_squared * face.plane.normal_len_squared) {

    if (!face.points_on_positive_side) {

      // Reuse old conflict list if face has none
      face.points_on_positive_side = std::move(get_conflict_list());
    }

    // Push new point to conflict list
    face.points_on_positive_side->push_back(pointIdx);

    if (plane_distance > face.furthest_point_distance) {

      face.furthest_point_distance = plane_distance;
      face.furthest_point = pointIdx;
    }

    return true;
  }

  return false;
}

// Reuse discarded conflict list memory
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

// Recycle conflict list memory to pool when face is removed
void QuickHull::reclaim_conflict_list(
    std::unique_ptr<std::vector<size_t>> &ptr) {

  const size_t size = ptr->size();

  //
  if ((size + 1) * 128 < ptr->capacity()) {

    ptr.reset(nullptr);
    return;
  }

  conflict_list_pool.push_back(std::move(ptr));
}
