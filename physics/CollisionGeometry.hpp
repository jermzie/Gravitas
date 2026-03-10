#pragma once

// NOT DONE

//
#include <filesystem>
#include <fstream>
#include <glm/glm.hpp>
#include <string>

#include "Mesh.hpp"
#include "Model.hpp"
#include "Plane.hpp"
#include "QuickHull.hpp"
#include "Ray.hpp"
#include "Shader.hpp"
// #include "AABB.hpp"
// #include "BoundingSphere.hpp"
#include "Config.hpp"
#include "ConvexMesh.hpp"
#include "ConvexMeshBuilder.hpp"

/* Physics representation of convex hull
Handles all physics related computations
*/

class CollisionGeometry {
private:
  ConvexMesh mesh;
  std::vector<size_t> render_indices;

  void build_render_indices(const ConvexMesh &mesh, bool is_counter_clockwise) {

    std::vector<bool> is_face_processed(mesh.faces.size(), false);
    std::vector<size_t> face_stack;
    std::unordered_map<size_t, size_t> vertex_index_map;

    render_indices.clear();

    size_t ccw = is_counter_clockwise ? 1 : 0;
    for (size_t i = 0; i < mesh.faces.size(); i++) {

      /*
      if (!mesh.faces[i].is_disabled()) {
        face_stack.push_back(i);
        break;
      }
      */
      face_stack.push_back(i);
    }
    if (face_stack.size() == 0) {
      return;
    }

    // const size_t face_count = mesh.faces.size() - mesh.disabled_faces.size();
    const size_t face_count = mesh.faces.size();
    render_indices.reserve(face_count * 3);

    size_t i = 0;
    while (face_stack.size()) {

      auto face_itr = face_stack.end() - 1;
      size_t top_index = *face_itr;
      // assert(!mesh.faces[top_index].is_disabled());
      face_stack.erase(face_itr);

      if (is_face_processed[top_index]) {
        continue;
      } else {

        is_face_processed[top_index] = true;
        auto he = mesh.get_face_half_edges(mesh.faces[top_index]);

        // Push neighboring faces onto stack
        size_t adjacent_faces[] = {
            mesh.half_edges[mesh.half_edges[he[0]].twin].face,
            mesh.half_edges[mesh.half_edges[he[1]].twin].face,
            mesh.half_edges[mesh.half_edges[he[2]].twin].face};
        for (auto f : adjacent_faces) {
          // if (!is_face_processed[f] && !mesh.faces[f].is_disabled()) {
          if (!is_face_processed[f]) {
            face_stack.push_back(f);
          }
        }

        // Process face vertices
        auto face_vertices = mesh.get_face_vertices(mesh.faces[top_index]);
        /*
        if (!use_original_indices) {
          for (auto &v : face_vertices) {
            auto vertex_itr = vertex_index_map.find(v);
            if (vertex_itr == vertex_index_map.end()) {

              optimizedVBO->push_back(point_cloud[v]);
              size_t new_index = optimizedVBO->size() - 1;
              vertex_index_map[v] = new_index;
              v = new_index;

            } else {
              v = vertex_itr->second;
            }
          }
        }
        */

        render_indices.push_back(face_vertices[0]);
        render_indices.push_back(face_vertices[1 + ccw]);
        render_indices.push_back(face_vertices[2 - ccw]);
      }
    }
  }

  static std::string hash_model(const std::string &path) {

    // Use last modified timestamp as seed
    auto seed = std::filesystem::last_write_time(path);
    size_t hash = std::hash<std::string>{}(path);

    // Based on boost::hash_combine
    hash ^= std::hash<long long>{}(seed.time_since_epoch().count()) +
            0x9e3779b9 + (hash << 6) + (hash >> 2);

    return std::to_string(hash);
  }

  static void save_mesh(const std::string &path, const ConvexMesh &mesh) {
    std::ofstream f(path, std::ios::binary);

    // Vertices
    uint32_t nv = mesh.vertices.size();
    f.write((char *)&nv, sizeof(nv));
    f.write((char *)mesh.vertices.data(), nv * sizeof(glm::vec3));

    // Faces
    uint32_t nf = mesh.faces.size();
    f.write((char *)&nf, sizeof(nf));
    f.write((char *)mesh.faces.data(), nf * sizeof(ConvexMesh::Face));

    // Half-edges
    uint32_t nhe = mesh.half_edges.size();
    f.write((char *)&nhe, sizeof(nhe));
    f.write((char *)mesh.half_edges.data(), nhe * sizeof(ConvexMesh::HalfEdge));
  }

  static ConvexMesh load_mesh(const std::string &path) {
    std::ifstream f(path, std::ios::binary);
    ConvexMesh mesh;

    uint32_t nv;
    f.read((char *)&nv, sizeof(nv));
    mesh.vertices.resize(nv);
    f.read((char *)mesh.vertices.data(), nv * sizeof(glm::vec3));

    uint32_t nf;
    f.read((char *)&nf, sizeof(nf));
    mesh.faces.resize(nf);
    f.read((char *)mesh.faces.data(), nf * sizeof(ConvexMesh::Face));

    uint32_t nhe;
    f.read((char *)&nhe, sizeof(nhe));
    mesh.half_edges.resize(nhe);
    f.read((char *)mesh.half_edges.data(), nhe * sizeof(ConvexMesh::HalfEdge));

    return mesh;
  }

public:
  CollisionGeometry() = default;

  CollisionGeometry(const ConvexMesh &mesh, bool is_counter_clockwise = true,
                    bool use_original_indices = true) {

    // Build render indices from topology
    build_render_indices(mesh, is_counter_clockwise);
    this->mesh = mesh;
  }

  // same tmax as far plane from perspective matrix???
  // int raycast(const Ray &ray, glm::vec3 &hit_point, float t_max = 500.0f) {
  int raycast(const Ray &ray, float &t, float t_max = 500.0f) const {

    // MISSED		:	return 0
    // FRONT FACE	:	return 1
    // BACK FACE	:	return -1

    const float epsilon = 0.000001f;
    float vn, vd, t_near, t_far;

    t_near = -std::numeric_limits<float>::infinity();
    t_far = t_max;

    for (size_t i = 0; i < mesh.faces.size(); i++) {

      const Plane &plane = mesh.faces[i].plane;

      vd = glm::dot(ray.direction, plane.normal);
      vn = glm::dot(ray.origin, plane.normal) + plane.distance;

      // ray parallel to plane
      if (std::abs(vd) < epsilon) {

        if (vn > 0.0) {
          // std::cerr << "a\n";
          return 0;
        }
        continue;
      } else {

        t = -vn / vd;

        // front facing
        if (vd < 0.0f) {

          // std::cerr << "b\n";
          if (t > t_far)
            return 0;
          if (t > t_near) {

            t_near = t;
          }
        }

        // back facing
        else {

          // std::cerr << "c\n";
          if (t < t_near)
            return 0;
          if (t < t_far) {

            t_far = t;
          }
        }
      }
    }

    // pass tests
    if (t_near >= epsilon) {

      t = t_near;
      // std::cerr << "HIT\n";
      // hit_point = ray.origin + ray.direction * t_hit;
      return 1;
    } else {

      if (t_far < t_max) {

        t = t_far;
        // hit_point = ray.origin + ray.direction * t_hit;
        //  std::cerr << "HIT\n";
        return -1;
      } else {

        // std::cerr << "d\n";
        return 0;
      }
    }
  }

  const ConvexMesh &get_mesh() const { return mesh; }

  const std::vector<size_t> &get_render_indices() const {
    return render_indices;
  };

  static ConvexMesh get_mesh_cache(const std::string &file_name,
                                   const std::vector<glm::vec3> &vertices) {
    namespace fs = std::filesystem;

    const std::string &resource_dir =
        std::string(PROJECT_SOURCE_DIR) + "/resources/";
    const std::string &cache_dir = resource_dir + "cache/";
    fs::create_directories(cache_dir);

    std::string file_path = resource_dir + file_name;
    std::string cache_path = cache_dir + hash_model(file_path) + ".hull";

    if (fs::exists(cache_path)) {
      CLOGI("%s cache exists. Fetching from disk...", file_name.c_str());
      return load_mesh(cache_path);
    }

    CLOGI("%s cache doesn't exist. Rebuilding mesh...", file_name.c_str());
    QuickHull qh;
    ConvexMesh mesh = qh.build_convex_mesh(vertices);
    save_mesh(cache_path, mesh);
    return mesh;
  }
};
