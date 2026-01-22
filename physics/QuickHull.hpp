#ifndef QUICKHULL_HPP
#define QUICKHULL_HPP

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtx/norm.hpp>

#ifndef PROJECT_SOURCE_DIR
#error "PROJECT_SOURCE_DIR is not defined"
#endif

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <deque>
#include <filesystem>
#include <iostream>
#include <memory>
#include <regex>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "../inc/Mesh.hpp"
#include "../inc/Model.hpp"
#include "../inc/Plane.hpp"
#include "../inc/Ray.hpp"
#include "../inc/Shader.hpp"
// #include "CollisionGeometry.hpp"
#include "ConvexMesh.hpp"
#include "ConvexMeshBuilder.hpp"

/*
*****************************************************************************
Build Convex Hull w/ QuickHull Algorithm

Steps:
1. Build initial polyhedron
-- setupInitialTetrahedron(); getExtremeValues();
2. Build conflict lists (assign points to each face if outside/above face plane)
-- partitionPoints(); addPointToFace();
3a. Select face w/ nonempty conflict list
3b. Pick point furthest from selected face
3c. Determine set of all faces
visible from point (dot-product & dfs) 3d. Compute horizon (boundary loop of
edges visible from point) 3e. Delete set of all visible faces 3f. Form new
triangular faces, connecting point to horizon edges 3g. Reassign Points (assign
points from deleted faces to new faces) 4.	Repeat step 3. until all
'outside' sets are empty
****************************************************************************
*/

class QuickHull {
private:
  float epsilon, epsilon_squared, scale;
  bool is_planar;

  ConvexMeshBuilder mesh;

  std::vector<glm::vec3> vertex_data;
  std::array<size_t, 6> extrema_indices;
  std::vector<glm::vec3> temp_planar_vertices;
  std::vector<std::unique_ptr<std::vector<size_t>>> conflict_list_pool;

  // Temporary variables used during iteration process
  std::vector<size_t> new_faces;
  std::vector<size_t> new_half_edges;
  std::vector<std::unique_ptr<std::vector<size_t>>> disabled_conflict_lists;
  std::vector<size_t> visible_faces;
  std::vector<size_t> horizon_edges;

  struct FaceData {
    size_t face_index;
    size_t
        entered_from_half_edge; // Mark as horizon edge if face is not visible

    FaceData() = default;
    FaceData(size_t face, size_t he)
        : face_index(face), entered_from_half_edge(he) {}
  };

  std::vector<FaceData> possible_visible_faces;
  std::deque<size_t> face_stack;

  void build_mesh(const std::vector<glm::vec3> &point_cloud,
                  float default_epsilon = 0.0001f);

  void setup_initial_tetrahedron();

  void create_half_edge_mesh();

  bool connect_horizon_edges(std::vector<size_t> &horizonEdges);

  std::array<size_t, 6> get_extrema();

  float get_scale();

  bool add_point_to_face(ConvexMeshBuilder::Face &face, size_t point_index);

  inline std::unique_ptr<std::vector<size_t>> get_conflict_list();

  inline void reclaim_conflict_list(std::unique_ptr<std::vector<size_t>> &ptr);

public:
  QuickHull() = default;

  /*
  ConvexHull getConvexHull(const std::vector<glm::vec3> &pointCloud,
                           bool CCW = true, bool useOriginalIndices = false,
                           float epsilon = 0.0001f);

  HalfEdgeMesh getHalfEdgeMesh(const std::vector<glm::vec3> &pointCloud,
                               bool CCW = true, float epsilon = 0.0001f);
  */

  ConvexMesh build_convex_mesh(const std::vector<glm::vec3> &point_cloud,
                               bool is_counter_clock_wise = true,
                               bool use_original_indices = false,
                               float epsilon = 0.0001f);

  std::array<float, 6> get_extrema_vertices();
};

#endif
