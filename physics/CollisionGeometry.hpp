#pragma once

// NOT DONE

//
#include <filesystem>
#include <glm/glm.hpp>

#include "Mesh.hpp"
#include "Model.hpp"
#include "Plane.hpp"
#include "Ray.hpp"
#include "Shader.hpp"
// #include "AABB.hpp"
// #include "BoundingSphere.hpp"
#include "ConvexMesh.hpp"
#include "ConvexMeshBuilder.hpp"

/* Physics representation of convex hull
Handles all physics related computations
*/

class CollisionGeometry {
private:
  ConvexMesh mesh;
  std::vector<size_t> render_indices;

  // Rendering
  // std::unique_ptr<std::vector<glm::vec3>> optimizedVBO;
  // std::vector<glm::vec3> vertices;

  void build_render_indices(const ConvexMesh &mesh, bool is_counter_clockwise) {

    /*
    if (!use_original_indices) {
      optimizedVBO.reset(new std::vector<glm::vec3>());
    }
    */

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
        size_t adjacent_faces[] = {mesh.half_edges[mesh.half_edges[he[0]].twin].face,
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

    /*
    if (!use_original_indices) {
      vertices = std::vector<glm::vec3>(*optimizedVBO);
    } else {
      vertices = point_cloud;
    }
    */
  }

public:
  CollisionGeometry() = default;

  /*
  CollisionGeometry(const ConvexMeshBuilder &mesh_builder,
                    const std::vector<glm::vec3> &point_cloud,
                    bool is_counter_clockwise = true,
                    bool use_original_indices = true) {

    // Create final mesh
    mesh = ConvexMesh(mesh_builder, point_cloud);

    // Build render indices from topology
    build_render_indices(mesh_builder, is_counter_clockwise);
  }
  */

  CollisionGeometry(const ConvexMesh &mesh, bool is_counter_clockwise = true, bool use_original_indices = true) {

    // Build render indices from topology
    build_render_indices(mesh, is_counter_clockwise);
    this->mesh = mesh;
  }
  /*
  const std::array<float, 6> get_extrema() const {

    std::array<size_t, 6> extrema_indices{0, 0, 0, 0, 0, 0};
    std::array<float, 6> extrema_vertices{vertices[0].x, vertices[0].x,
                                          vertices[0].y, vertices[0].y,
                                          vertices[0].z, vertices[0].z};

    for (size_t i = 1; i < vertices.size(); i++) {

      const glm::vec3 &pos = vertices[i];

      // X-axis
      if (pos.x > extrema_vertices[0]) {
        extrema_vertices[0] = pos.x;
        extrema_indices[0] = i;
      }

      else if (pos.x < extrema_vertices[1]) {
        extrema_vertices[1] = pos.x;
        extrema_indices[1] = i;
      }

      // Y-axis
      if (pos.y > extrema_vertices[2]) {
        extrema_vertices[2] = pos.y;
        extrema_indices[2] = i;
      }

      else if (pos.y < extrema_vertices[3]) {
        extrema_vertices[3] = pos.y;
        extrema_indices[3] = i;
      }

      // Z-axis
      if (pos.z > extrema_vertices[4]) {
        extrema_vertices[4] = pos.z;
        extrema_indices[4] = i;
      }

      else if (pos.z < extrema_vertices[5]) {
        extrema_vertices[5] = pos.z;
        extrema_indices[5] = i;
      }
    }

    return extrema_vertices;
  }
  */

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

  ConvexMesh &get_mesh() { return mesh; }

  const std::vector<size_t> &get_render_indices() const { return render_indices; };

  /*
  AABB compute_aabb() const {

    std::array<float, 6U> extrema = mesh.get_extrema();
    AABB box;
    for (int i = 0; i < 6; i++) {

       *   array structure:
       *   0      1      2      3      4      5
       *  [x_min, x_max, y_min, y_max, z_min, z_max]
      (i % 2 == 0) ? box.min[i / 2] = extrema[i] : box.max[i / 2] = extrema[i];
    }
    return box;
  }
  */

  // BoundingSphere compute_sphere() const {}
};
