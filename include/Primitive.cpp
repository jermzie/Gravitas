#include "Primitive.hpp"
#include "QuickHull.hpp"
#include "Ray.hpp"
#include <array>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace {
constexpr float epsilon = 1e-6f;
constexpr float inv_sqrt3 = 0.57735027f; // tetrahedron face normal components
} // namespace

namespace mesh {
inline Vertex make_vertex(const glm::vec3 &p, const glm::vec3 &n,
                          const glm::vec2 &uv) {
  Vertex v{};
  v.position = p;
  v.normal = n;
  v.texture_coordinates = uv;
  return v;
}

/* Append the two triangles of a quad band cell:
 *   a---b
 *   | / |
 *   c---d
 * in the winding order that makes a "row increases away from viewer"
 * grid (like a UV sphere going from +Y pole to -Y pole) face outward. */
inline void triangulate_quad_cw(std::vector<unsigned int> &out, unsigned int a,
                                unsigned int b, unsigned int c,
                                unsigned int d) {
  // tri 1
  out.push_back(a);
  out.push_back(b);
  out.push_back(c);

  // tri 2
  out.push_back(b);
  out.push_back(d);
  out.push_back(c);
}

/* Same quad, reversed winding (used where the row direction is flipped,
 * e.g. a hemisphere built from equator up to pole instead of pole down
 * to equator). */
inline void triangulate_quad_ccw(std::vector<unsigned int> &out, unsigned int a,
                                 unsigned int b, unsigned int c,
                                 unsigned int d) {

  // tri 1
  out.push_back(a);
  out.push_back(c);
  out.push_back(b);

  // tri 2
  out.push_back(b);
  out.push_back(c);
  out.push_back(d);
}
} // namespace mesh

namespace hull {
constexpr uint32_t cache_version = 1;

struct CacheHeader {
  uint32_t version;
  uint32_t vertex_size;
  uint32_t face_size;
  uint32_t half_edge_size;
};

inline CacheHeader current_header() {
  CacheHeader h{};
  h.version = cache_version;
  h.vertex_size = sizeof(glm::vec3);
  h.face_size = sizeof(HalfEdgeMesh::Face);
  h.half_edge_size = sizeof(HalfEdgeMesh::HalfEdge);
  return h;
}

inline std::string hash_model(const std::string &path) {

  // Use last modified timestamp as seed
  auto seed = std::filesystem::last_write_time(path);
  size_t hash = std::hash<std::string>{}(path);

  // Based on boost::hash_combine
  hash ^= std::hash<long long>{}(seed.time_since_epoch().count()) + 0x9e3779b9 +
          (hash << 6) + (hash >> 2);

  return std::to_string(hash);
}

inline void write_disk(const std::string &path, const HalfEdgeMesh &mesh) {

  // write to temp file, so an interrupted write never leaves truncated cache
  const std::string tmp_path = path + ".tmp";
  {
    std::ofstream f(tmp_path, std::ios::binary);

    CacheHeader header = current_header();
    f.write((char *)&header, sizeof(header));

    // Vertices
    uint32_t nv = mesh.vertices.size();
    f.write((char *)&nv, sizeof(nv));
    f.write((char *)mesh.vertices.data(), nv * sizeof(glm::vec3));

    // Faces
    uint32_t nf = mesh.faces.size();
    f.write((char *)&nf, sizeof(nf));
    f.write((char *)mesh.faces.data(), nf * sizeof(HalfEdgeMesh::Face));

    // Half-edges
    uint32_t nhe = mesh.half_edges.size();
    f.write((char *)&nhe, sizeof(nhe));
    f.write((char *)mesh.half_edges.data(),
            nhe * sizeof(HalfEdgeMesh::HalfEdge));

    if (!f) {
      CLOGE("Failed to write hull cache %s", tmp_path.c_str());
      return;
    }
  }

  std::error_code err;
  std::filesystem::rename(tmp_path, path, err);
  if (err) {
    CLOGE("Failed to write hull cache %s: %s", path.c_str(),
          err.message().c_str());
  }
}

// false on a stale, truncated or corrupt cache
inline bool read_disk(const std::string &path, HalfEdgeMesh &out) {

  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) {
    return false;
  }
  const uint64_t file_size = static_cast<uint64_t>(f.tellg());
  f.seekg(0);

  // check header
  CacheHeader header{}, expected = current_header();
  f.read((char *)&header, sizeof(header));
  if (!f || std::memcmp(&header, &expected, sizeof(header)) != 0) {
    return false;
  }

  // read count + array, reject count if greater than the file capacity
  auto read_array = [&](auto &vec) {
    using T = typename std::decay_t<decltype(vec)>::value_type;
    uint32_t n;
    if (!f.read((char *)&n, sizeof(n))) {
      return false;
    }
    if (uint64_t(n) * sizeof(T) > file_size) {
      return false;
    }
    vec.resize(n);
    return bool(f.read((char *)vec.data(), n * sizeof(T)));
  };

  HalfEdgeMesh mesh;
  if (!read_array(mesh.vertices) || !read_array(mesh.faces) ||
      !read_array(mesh.half_edges) || f.peek() != EOF) {
    return false;
  }

  // every index must land inside the mesh
  const size_t nv = mesh.vertices.size(), nf = mesh.faces.size(),
               nhe = mesh.half_edges.size();
  for (const HalfEdgeMesh::Face &face : mesh.faces) {
    if (face.he >= nhe) {
      return false;
    }
  }
  for (const HalfEdgeMesh::HalfEdge &he : mesh.half_edges) {
    if (he.vert >= nv || he.twin >= nhe || he.face >= nf || he.next >= nhe ||
        he.prev >= nhe) {
      return false;
    }
  }

  out = std::move(mesh);
  return true;
}

inline HalfEdgeMesh check_cache(const std::string &file_name,
                                const std::vector<glm::vec3> &vertices) {

  namespace fs = std::filesystem;

  const std::string resource_dir =
      std::string(PROJECT_SOURCE_DIR) + "/resources/";
  const std::string cache_dir = resource_dir + "cache/";
  fs::create_directories(cache_dir);

  std::string file_path = resource_dir + file_name;
  // versioned name keeps these apart from the header-less files the
  // deprecated CollisionGeometry cache still writes to the same directory
  std::string cache_path = cache_dir + hull::hash_model(file_path) + ".v" +
                           std::to_string(cache_version) + ".hull";

  if (fs::exists(cache_path)) {
    HalfEdgeMesh mesh;
    if (hull::read_disk(cache_path, mesh)) {
      CLOGI("%s cache exists. Fetching from disk...", file_name.c_str());
      return mesh;
    }
    CLOGW("%s cache is stale or corrupt. Rebuilding mesh...",
          file_name.c_str());
  } else {
    CLOGI("%s cache doesn't exist. Rebuilding mesh...", file_name.c_str());
  }

  HalfEdgeMesh mesh = QuickHull().build_convex_mesh(vertices);
  hull::write_disk(cache_path, mesh);
  return mesh;
}
} // namespace hull

AABB Primitive::geom_aabb() const {
  return AABB().transform_arvo(base_aabb(), local_xfrm.get_matrix());
}

RayHit Primitive::geom_raycast(const Ray &r, float t_max) const {
  Ray local = world_to_local_ray(local_xfrm, r);
  RayHit res = base_raycast(local, t_max);
  if (res.is_hit) {
    res.hit_point = r.origin + r.direction * res.hit_dist;
    res.surface_norm =
        glm::normalize(local_xfrm.get_normal_matrix() * res.surface_norm);
  }
  return res;
}

/*
 For p' = L p + t with L = R S: max dot(d, L p) = max dot(L^T d, p), so query
 the base shape along L^T d and map the result back out.
*/
glm::vec3 Primitive::geom_support(glm::vec3 dir) const {
  const glm::mat4 model = local_xfrm.get_matrix();
  glm::vec3 p = base_support(glm::transpose(glm::mat3(model)) * dir);
  return glm::vec3(model * glm::vec4(p, 1.0f));
}

Polyhedron::Polyhedron(const std::shared_ptr<Model> &model) {
  base_model = model;
  topology = hull::check_cache(model->file_name, model->get_vertex_data());
}

/*
 Graphics Gems II:
 https://github.com/erich666/GraphicsGems/blob/master/gemsii/RayCPhdron.c
*/
RayHit Polyhedron::base_raycast(const Ray &ray, float t_max) const {

  RayHit res = {};
  float t, t_near, t_far, vn, vd;
  size_t front_norm = 0, back_norm = 0;

  if (topology.faces.empty()) {
    res.is_hit = false;
    return res;
  }

  t_near = -std::numeric_limits<float>::infinity(); // near plane
  t_far = t_max;                                    // far plane

  // test each face plane in polyhedron
  for (size_t i = 0; i < topology.faces.size(); i++) {
    const Plane &plane = topology.faces[i].plane;

    vd = glm::dot(ray.direction, plane.normal);
    vn = glm::dot(ray.origin, plane.normal) + plane.distance;

    if (std::abs(vd) < epsilon) {
      // ray parallel to plane - check if ray inside plane's half-space
      if (vn > 0.0) {
        // ray outside plane half-space
        res.is_hit = false;
        return res;
      }
      continue;

    } else {
      // ray not parallel - get distance to plane
      t = -vn / vd;

      if (vd < 0.0f) {
        // front facing

        if (t > t_far) {
          res.is_hit = false;
          return res;
        }
        if (t > t_near) {
          // hit near plane - update normal

          t_near = t;
          front_norm = i;
        }
      } else {
        // back facing

        if (t < t_near) {
          res.is_hit = false;
          return res;
        }
        if (t < t_far) {
          // hit far plane - update normal

          t_far = t;
          back_norm = i;
        }
      }
    }
  }

  // pass tests
  if (t_near >= epsilon) {
    // outside - hitting front face

    t = t_near;
    res.is_hit = true;
    res.hit_point = ray.origin + ray.direction * t;
    res.hit_dist = t;
    res.surface_norm = topology.faces[front_norm].plane.normal;
    return res;
  } else {
    if (t_far < t_max) {
      // inside - hitting back face

      t = t_far;
      res.is_hit = true;
      res.hit_point = ray.origin + ray.direction * t;
      res.hit_dist = t;
      res.surface_norm = topology.faces[back_norm].plane.normal;
      return res;
    } else {
      // inside - far plane beyond t_max

      res.is_hit = false;
      return res;
    }
  }
}

MassProperties Polyhedron::compute_mass_properties(float density) const {
  HalfEdgeMesh xfrm_mesh = topology;
  const glm::mat4 model = local_xfrm.get_matrix();
  const glm::mat3 norm = local_xfrm.get_normal_matrix();

  for (glm::vec3 &v : xfrm_mesh.vertices) {
    v = glm::vec3(model * glm::vec4(v, 1.0f));
  }
  // preserve face normals under non-uniform scale
  for (HalfEdgeMesh::Face &face : xfrm_mesh.faces) {
    glm::vec3 n = glm::normalize(norm * face.plane.normal);
    glm::vec3 p = glm::vec3(model * glm::vec4(face.plane.point, 1.0f));
    face.plane = Plane(n, p);
  }

  return MassProperties::compute(xfrm_mesh, density);
}

glm::vec3 Polyhedron::base_support(glm::vec3 dir) const {
  // brute-force scan for the vertex furthest along dir
  float max_proj = -std::numeric_limits<float>::max();
  glm::vec3 result(0.0f);
  for (const auto &v : topology.vertices) {
    float proj = glm::dot(v, dir);
    if (proj > max_proj) {
      max_proj = proj;
      result = v;
    }
  }
  return result;
}

// Hardcoded half-edge topology
const HalfEdgeMesh &Polyhedron::get_cube_hull() {
  static const HalfEdgeMesh hull = [] {
    HalfEdgeMesh mesh;

    // 8 vertices
    mesh.vertices = {
        {-1, -1, -1}, {1, -1, -1}, {1, 1, -1}, {-1, 1, -1}, // 0-3
        {-1, -1, 1},  {1, -1, 1},  {1, 1, 1},  {-1, 1, 1},  // 4-7
    };

    // 6 quad face loops
    struct Face {
      std::array<size_t, 4> loop;
      glm::vec3 normal;
    };
    const Face quad_faces[6] = {
        {{1, 2, 6, 5}, {1, 0, 0}},  // +X
        {{4, 7, 3, 0}, {-1, 0, 0}}, // -X
        {{7, 6, 2, 3}, {0, 1, 0}},  // +Y
        {{0, 1, 5, 4}, {0, -1, 0}}, // -Y
        {{4, 5, 6, 7}, {0, 0, 1}},  // +Z
        {{1, 0, 3, 2}, {0, 0, -1}}, // -Z
    };

    mesh.half_edges.resize(24);
    mesh.faces.resize(6);

    for (size_t f = 0; f < 6; ++f) {
      const auto &loop = quad_faces[f].loop;
      mesh.faces[f] = {4 * f,
                       Plane(quad_faces[f].normal, mesh.vertices[loop[0]])};

      // 24 half-edges
      for (size_t k = 0; k < 4; ++k) {
        mesh.half_edges[4 * f + k] = {
            loop[(k + 1) % 4],   // vert: destination of this half-edge
            0,                   // twin: patched in below
            f,                   // face
            4 * f + (k + 1) % 4, // next
            4 * f + (k + 3) % 4, // prev
        };
      }
    }

    // The 12 shared edges, derived by hand from the face loops above.
    static const std::array<std::pair<size_t, size_t>, 12> twins = {{
        {0, 23},
        {1, 9},
        {2, 17},
        {3, 13},
        {4, 19},
        {5, 11},
        {6, 21},
        {7, 15},
        {8, 18},
        {10, 22},
        {12, 20},
        {14, 16},
    }};
    for (auto [a, b] : twins) {
      mesh.half_edges[a].twin = b;
      mesh.half_edges[b].twin = a;
    }

    return mesh;
  }();
  return hull;
}

// Hardcoded half-edge topology
const HalfEdgeMesh &Polyhedron::get_tetrahedron_hull() {
  static const HalfEdgeMesh hull = [] {
    HalfEdgeMesh mesh;

    // 4 vertices
    mesh.vertices = {
        {1, 1, 1},
        {1, -1, -1},
        {-1, 1, -1},
        {-1, -1, 1},
    };

    // 4 triangular face loops
    struct Face {
      std::array<size_t, 3> loop;
      glm::vec3 normal;
    };
    const float n = inv_sqrt3;
    const Face tri_faces[4] = {
        {{0, 1, 2}, {n, n, -n}},
        {{0, 3, 1}, {n, -n, n}},
        {{0, 2, 3}, {-n, n, n}},
        {{1, 3, 2}, {-n, -n, -n}},
    };

    mesh.half_edges.resize(12);
    mesh.faces.resize(4);

    for (size_t f = 0; f < 4; ++f) {
      const auto &loop = tri_faces[f].loop;
      mesh.faces[f] = {3 * f,
                       Plane(tri_faces[f].normal, mesh.vertices[loop[0]])};

      // 12 half-edges
      for (size_t k = 0; k < 3; ++k) {
        mesh.half_edges[3 * f + k] = {
            loop[(k + 1) % 3], 0, f, 3 * f + (k + 1) % 3, 3 * f + (k + 2) % 3,
        };
      }
    }

    // The 6 shared edges, derived by hand from the face loops above.
    static const std::array<std::pair<size_t, size_t>, 6> twins = {{
        {0, 5},
        {1, 11},
        {2, 6},
        {3, 8},
        {4, 9},
        {7, 10},
    }};
    for (auto [a, b] : twins) {
      mesh.half_edges[a].twin = b;
      mesh.half_edges[b].twin = a;
    }

    return mesh;
  }();
  return hull;
}

// Hardcoded render model
std::shared_ptr<Model> Polyhedron::get_cube_model() {
  static std::shared_ptr<Model> cached = [] {
    using namespace mesh;

    std::vector<Vertex> vertices = {
        /* +X */
        make_vertex({1, -1, -1}, {1, 0, 0}, {0, 0}),
        make_vertex({1, 1, -1}, {1, 0, 0}, {0, 1}),
        make_vertex({1, 1, 1}, {1, 0, 0}, {1, 1}),
        make_vertex({1, -1, 1}, {1, 0, 0}, {1, 0}),
        /* -X */
        make_vertex({-1, -1, 1}, {-1, 0, 0}, {0, 0}),
        make_vertex({-1, 1, 1}, {-1, 0, 0}, {0, 1}),
        make_vertex({-1, 1, -1}, {-1, 0, 0}, {1, 1}),
        make_vertex({-1, -1, -1}, {-1, 0, 0}, {1, 0}),
        /* +Y */
        make_vertex({-1, 1, 1}, {0, 1, 0}, {0, 0}),
        make_vertex({1, 1, 1}, {0, 1, 0}, {1, 0}),
        make_vertex({1, 1, -1}, {0, 1, 0}, {1, 1}),
        make_vertex({-1, 1, -1}, {0, 1, 0}, {0, 1}),
        /* -Y */
        make_vertex({-1, -1, -1}, {0, -1, 0}, {0, 0}),
        make_vertex({1, -1, -1}, {0, -1, 0}, {1, 0}),
        make_vertex({1, -1, 1}, {0, -1, 0}, {1, 1}),
        make_vertex({-1, -1, 1}, {0, -1, 0}, {0, 1}),
        /* +Z */
        make_vertex({-1, -1, 1}, {0, 0, 1}, {0, 0}),
        make_vertex({1, -1, 1}, {0, 0, 1}, {1, 0}),
        make_vertex({1, 1, 1}, {0, 0, 1}, {1, 1}),
        make_vertex({-1, 1, 1}, {0, 0, 1}, {0, 1}),
        /* -Z */
        make_vertex({1, -1, -1}, {0, 0, -1}, {0, 0}),
        make_vertex({-1, -1, -1}, {0, 0, -1}, {1, 0}),
        make_vertex({-1, 1, -1}, {0, 0, -1}, {1, 1}),
        make_vertex({1, 1, -1}, {0, 0, -1}, {0, 1}),
    };
    std::vector<unsigned int> indices = {
        0,  1,  2,  0,  2,  3,  /* +X */
        4,  5,  6,  4,  6,  7,  /* -X */
        8,  9,  10, 8,  10, 11, /* +Y */
        12, 13, 14, 12, 14, 15, /* -Y */
        16, 17, 18, 16, 18, 19, /* +Z */
        20, 21, 22, 20, 22, 23, /* -Z */
    };

    return std::make_shared<Model>(Model({Mesh(vertices, indices)}));
  }();
  return cached;
}

// Hardcoded render model
std::shared_ptr<Model> Polyhedron::get_tetrahedron_model() {
  static std::shared_ptr<Model> cached = [] {
    using namespace mesh;

    const glm::vec3 n0(inv_sqrt3, inv_sqrt3, -inv_sqrt3);
    const glm::vec3 n1(inv_sqrt3, -inv_sqrt3, inv_sqrt3);
    const glm::vec3 n2(-inv_sqrt3, inv_sqrt3, inv_sqrt3);
    const glm::vec3 n3(-inv_sqrt3, -inv_sqrt3, -inv_sqrt3);

    std::vector<Vertex> vertices = {
        /* face 0: v0, v1, v2 */
        make_vertex({1, 1, 1}, n0, {0.5f, 1.0f}),
        make_vertex({1, -1, -1}, n0, {0.0f, 0.0f}),
        make_vertex({-1, 1, -1}, n0, {1.0f, 0.0f}),
        /* face 1: v0, v3, v1 */
        make_vertex({1, 1, 1}, n1, {0.5f, 1.0f}),
        make_vertex({-1, -1, 1}, n1, {0.0f, 0.0f}),
        make_vertex({1, -1, -1}, n1, {1.0f, 0.0f}),
        /* face 2: v0, v2, v3 */
        make_vertex({1, 1, 1}, n2, {0.5f, 1.0f}),
        make_vertex({-1, 1, -1}, n2, {0.0f, 0.0f}),
        make_vertex({-1, -1, 1}, n2, {1.0f, 0.0f}),
        /* face 3: v1, v3, v2 */
        make_vertex({1, -1, -1}, n3, {0.5f, 1.0f}),
        make_vertex({-1, -1, 1}, n3, {0.0f, 0.0f}),
        make_vertex({-1, 1, -1}, n3, {1.0f, 0.0f}),
    };
    std::vector<unsigned int> indices = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

    return std::make_shared<Model>(Model({Mesh(vertices, indices)}));
  }();
  return cached;
}

void Polyhedron::generate_cube() {
  base_model = get_cube_model();
  topology = get_cube_hull();
}

void Polyhedron::generate_tetrahedron() {
  base_model = get_tetrahedron_model();
  topology = get_tetrahedron_hull();
}

Sphere::Sphere(float radius) { generate_sphere(16, 32, radius); }

/*
 Ray vs. Sphere centered at origin.
 solve |O + D * t|^2 - R^2 = 0 for t
*/
RayHit Sphere::base_raycast(const Ray &ray, float t_max) const {

  RayHit res = {};

  float a = glm::dot(ray.direction, ray.direction);             // D^2
  float b = glm::dot(ray.origin, ray.direction);                // O * D
  float c = glm::dot(ray.origin, ray.origin) - radius * radius; // O^2 - R^2

  float discriminant = b * b - a * c;
  if (a < epsilon || discriminant < 0.0f) {
    // degenerate ray or ray misses sphere
    res.is_hit = false;
    return res;
  }

  // solve for roots w/ quadratic equation
  float sqrtd = std::sqrt(discriminant);
  float t_near = (-b - sqrtd) / a;
  float t_far = (-b + sqrtd) / a;
  float t;

  if (t_near >= epsilon) {
    // front face - outside sphere
    t = t_near;
  } else if (t_far >= epsilon) {
    // back face - inside sphere
    t = t_far;
  } else {
    // sphere behind ray
    res.is_hit = false;
    return res;
  }

  if (t > t_max) {
    res.is_hit = false;
    return res;
  }

  res.is_hit = true;
  res.hit_point = ray.origin + ray.direction * t;
  res.hit_dist = t;
  res.surface_norm = res.hit_point / radius;
  return res;
}

Primitive &Sphere::scale(glm::vec3 factor) {
  if (std::abs(factor.x - factor.y) > epsilon ||
      std::abs(factor.x - factor.z) > epsilon) {
    CLOGW("Sphere only supports uniform scale - ignoring (%f, %f, %f)",
          factor.x, factor.y, factor.z);
    return *this;
  }
  local_xfrm.set_scale(factor);
  return *this;
}

MassProperties Sphere::compute_mass_properties(float density) const {
  // scale is guaranteed uniform by Sphere::scale() override
  const float r = radius * std::abs(local_xfrm.get_scale().x);

  // volume = 4/3 * π * r^3
  float vol = (4.0f / 3.0f) * glm::pi<float>() * r * r * r;
  float mass = density * vol;

  // inertia = 2/5 * m * r^2
  glm::mat3 inertia = glm::mat3(2.0f / 5.0f * mass * r * r);

  return MassProperties{.mass = mass,
                        .inv_mass = 1.0f / mass,
                        .centre_of_mass = local_xfrm.get_position(),
                        .inertia_tensor = inertia,
                        .inv_inertia_tensor = glm::inverse(inertia)};
}

glm::vec3 Sphere::base_support(glm::vec3 dir) const {
  float len = glm::length(dir);
  if (len < epsilon) {
    return glm::vec3(radius, 0.0f, 0.0f);
  }
  return dir * (radius / len);
}

void Sphere::generate_sphere(int rings, int slices, float radius) {
  using namespace mesh;

  if (rings < 2 || slices < 3 || radius <= 0.0f) {
    throw std::invalid_argument("Sphere: invalid parameters (rings " +
                                std::to_string(rings) + ", slices " +
                                std::to_string(slices) + ", radius " +
                                std::to_string(radius) + ")");
  }

  this->radius = radius;

  const int width = slices + 1;
  std::vector<Vertex> vertices(static_cast<size_t>(rings + 1) * width);
  std::vector<unsigned int> indices;
  indices.reserve(static_cast<size_t>(rings) * slices * 6);

  for (int r = 0; r <= rings; ++r) {
    // phi = [0, pi], +Y pole to -Y pole
    float phi = static_cast<float>(r) / rings * glm::pi<float>();
    float sin_phi = std::sin(phi), cos_phi = std::cos(phi);

    for (int s = 0; s <= slices; ++s) {

      // theta = [0, 2pi]
      float theta = static_cast<float>(s) / slices * glm::two_pi<float>();

      // normal
      glm::vec3 n(sin_phi * std::cos(theta), cos_phi,
                  sin_phi * std::sin(theta));

      // texture coordinates
      glm::vec2 uv(static_cast<float>(s) / slices,
                   1.0f - static_cast<float>(r) / rings);
      // vertex
      vertices[static_cast<size_t>(r) * width + s] =
          make_vertex(radius * n, n, uv);
    }
  }

  for (int r = 0; r < rings; ++r) {
    for (int s = 0; s < slices; ++s) {

      // triangulation
      unsigned int a = static_cast<unsigned int>(r * width + s);
      unsigned int b = static_cast<unsigned int>(r * width + s + 1);
      unsigned int c = static_cast<unsigned int>((r + 1) * width + s);
      unsigned int d = static_cast<unsigned int>((r + 1) * width + s + 1);
      triangulate_quad_cw(indices, a, b, c, d);
    }
  }

  base_model = std::make_shared<Model>(Model({Mesh(vertices, indices)}));
}

Frustum::Frustum(float r_bot, float r_top, float half_height) {
  generate_frustum(32, r_bot, r_top, half_height);
}

Primitive &Frustum::scale(glm::vec3 factor) {
  if (std::abs(factor.x - factor.z) > epsilon) {
    CLOGW("Frustum only supports uniform radial scale (x == z) - ignoring "
          "(%f, %f, %f)",
          factor.x, factor.y, factor.z);
    return *this;
  }
  local_xfrm.set_scale(factor);
  return *this;
}

/*
 Ray vs. frustum centered at origin, axis +Y.
 side: x^2 + z^2 = (alpha * y + beta)^2 for |y| <= h, where
       r(y) = alpha * y + beta interpolates r_bottom (y = -h) to r_top (y = +h)
 caps: planes y = +-h, clipped to their disc
 Takes the nearest valid hit, so a ray starting inside hits the back face.
*/
RayHit Frustum::base_raycast(const Ray &ray, float t_max) const {

  RayHit res = {};
  res.is_hit = false;

  const glm::vec3 &p = ray.origin;
  const glm::vec3 &v = ray.direction;
  const float alpha = (r_top - r_bot) / (2.0f * half_height);
  const float beta = 0.5f * (r_top + r_bot);

  float t_best = t_max;
  glm::vec3 n_best(0.0f);

  auto consider = [&](float t, const glm::vec3 &n) {
    if (t >= epsilon && t <= t_best) {
      t_best = t;
      n_best = n;
      res.is_hit = true;
    }
  };

  // side - substitute p + tv into F(x) = x^2 + z^2 - r(y)^2 = 0
  //   a t^2 + 2 b t + c = 0
  const float q = alpha * p.y + beta; // r(y) at the ray origin
  const float w = alpha * v.y;        // rate of change of r(y) along the ray
  const float a = v.x * v.x + v.z * v.z - w * w;
  const float b = p.x * v.x + p.z * v.z - q * w;
  const float c = p.x * p.x + p.z * p.z - q * q;

  auto consider_side = [&](float t) {
    glm::vec3 x = p + v * t;
    if (std::abs(x.y) > half_height) {
      return;
    }
    // grad F / 2 = (x, -alpha * r(y), z)
    glm::vec3 grad(x.x, -alpha * (alpha * x.y + beta), x.z);
    float len = glm::length(grad);
    if (len < epsilon) {
      // exactly at a cone apex - fall back to the axis
      grad = glm::vec3(0.0f, alpha < 0.0f ? 1.0f : -1.0f, 0.0f);
      len = 1.0f;
    }
    consider(t, grad / len);
  };

  if (std::abs(a) > epsilon) {
    float disc = b * b - a * c;
    if (disc >= 0.0f) {
      float sqrt_disc = std::sqrt(disc);
      consider_side((-b - sqrt_disc) / a);
      consider_side((-b + sqrt_disc) / a);
    }
  } else if (std::abs(b) > epsilon) {
    // ray parallel to a slant line of the cone - single root
    consider_side(-c / (2.0f * b));
  }

  // caps
  if (std::abs(v.y) > epsilon) {
    auto consider_cap = [&](float y, float radius, float ny) {
      if (radius <= 0.0f) {
        return;
      }
      float t = (y - p.y) / v.y;
      glm::vec3 x = p + v * t;
      if (x.x * x.x + x.z * x.z <= radius * radius) {
        consider(t, glm::vec3(0.0f, ny, 0.0f));
      }
    };
    consider_cap(half_height, r_top, 1.0f);
    consider_cap(-half_height, r_bot, -1.0f);
  }

  if (res.is_hit) {
    res.hit_point = p + v * t_best;
    res.hit_dist = t_best;
    res.surface_norm = n_best;
  }
  return res;
}

/*
 Frustum is the convex hull of its two cap discs, so the support point is
 the furthest point of whichever disc reaches further along dir.
*/
glm::vec3 Frustum::base_support(glm::vec3 dir) const {
  glm::vec2 d_xz(dir.x, dir.z);
  float len = glm::length(d_xz);

  // dir parallel to the axis - every cap point ties, take the cap centre
  glm::vec2 radial = (len < epsilon) ? glm::vec2(0.0f) : d_xz / len;

  glm::vec3 top(r_top * radial.x, half_height, r_top * radial.y);
  glm::vec3 bot(r_bot * radial.x, -half_height, r_bot * radial.y);
  return glm::dot(top, dir) >= glm::dot(bot, dir) ? top : bot;
}

/*
 Stack of thin discs of radius r(y) along the axis, with base-relative
 height y' in [0, H], R = bottom radius, r = top radius:

   V      = pi H / 3 * (R^2 + Rr + r^2)
   y_com  = H (R^2 + 2Rr + 3r^2) / (4 (R^2 + Rr + r^2))  (above the base)
   I_axis = 3m/10 * (R^4 + R^3 r + R^2 r^2 + R r^3 + r^4) / (R^2 + Rr + r^2)
   I_perp = I_axis / 2 + density pi H^3 (R^2 + 3Rr + 6r^2) / 30 - m y_com^2

 (I_perp: each disc's own inertia about a diameter plus its offset from the
 base, then the parallel axis theorem to move to the centre of mass.)
 Limits: r = R gives the cylinder, r = 0 the cone.
*/
MassProperties Frustum::compute_mass_properties(float density) const {
  // radial scale is guaranteed uniform by Frustum::scale() override
  const glm::vec3 s = glm::abs(local_xfrm.get_scale());
  const float R = r_bot * s.x;
  const float r = r_top * s.x;
  const float H = 2.0f * half_height * s.y;

  const float pi = glm::pi<float>();
  const float k = R * R + R * r + r * r;

  float vol = pi * H / 3.0f * k;
  float mass = density * vol;

  float y_com = H * (R * R + 2.0f * R * r + 3.0f * r * r) / (4.0f * k);

  float i_axis = 0.3f * mass *
                 (R * R * R * R + R * R * R * r + R * R * r * r +
                  R * r * r * r + r * r * r * r) /
                 k;
  float i_perp =
      0.5f * i_axis +
      density * pi * H * H * H * (R * R + 3.0f * R * r + 6.0f * r * r) / 30.0f -
      mass * y_com * y_com;

  // principal axes are the local frame - rotate into the parent frame
  const glm::mat3 rot = glm::mat3_cast(local_xfrm.get_orientation());
  glm::mat3 inertia_local(0.0f);
  inertia_local[0][0] = i_perp;
  inertia_local[1][1] = i_axis;
  inertia_local[2][2] = i_perp;
  glm::mat3 inertia = rot * inertia_local * glm::transpose(rot);

  // y_com is from the base; the local origin sits at mid-height
  glm::vec3 com_local(0.0f, y_com - 0.5f * H, 0.0f);

  return MassProperties{.mass = mass,
                        .inv_mass = 1.0f / mass,
                        .centre_of_mass =
                            local_xfrm.get_position() + rot * com_local,
                        .inertia_tensor = inertia,
                        .inv_inertia_tensor = glm::inverse(inertia)};
}

/*
 A zero-radius end collapses to an apex: no cap, and the half of each side
 quad that would be zero-area is dropped.
*/
void Frustum::generate_frustum(int slices, float r_bottom, float r_top,
                               float half_height) {
  using namespace mesh;

  if (slices < min_slices) {
    CLOGW("Frustum needs >= %d slices to match its round physics shape - "
          "clamping %d",
          min_slices, slices);
    slices = min_slices;
  }
  // validate before touching any state, so a bad call never leaves the
  // physics dimensions and render model out of sync
  if (r_bottom < 0.0f || r_top < 0.0f || half_height <= 0.0f ||
      (r_bottom <= 0.0f && r_top <= 0.0f)) {
    throw std::invalid_argument("Frustum: invalid dimensions (r_bottom " +
                                std::to_string(r_bottom) + ", r_top " +
                                std::to_string(r_top) + ", half_height " +
                                std::to_string(half_height) + ")");
  }

  this->r_bot = r_bottom;
  this->r_top = r_top;
  this->half_height = half_height;

  const int width = slices + 1;
  std::vector<Vertex> vertices(static_cast<size_t>(2 * width));

  // side ring 0 = top rim, side ring 1 = bot rim
  const unsigned int side_top = 0;
  const unsigned int side_bot = static_cast<unsigned int>(width);

  for (int s = 0; s <= slices; ++s) {
    float theta = static_cast<float>(s) / slices * glm::two_pi<float>();
    float ct = std::cos(theta), st = std::sin(theta);

    // slant normal: profile runs (r_bottom, -h) -> (r_top, +h), so the outward
    // normal in the (radial, y) plane is (2h, r_bottom - r_top)
    glm::vec3 n = glm::normalize(glm::vec3(
        2.0f * half_height * ct, r_bottom - r_top, 2.0f * half_height * st));

    float u = static_cast<float>(s) / slices;

    vertices[side_top + s] =
        make_vertex({r_top * ct, half_height, r_top * st}, n, {u, 1.0f});
    vertices[side_bot + s] =
        make_vertex({r_bottom * ct, -half_height, r_bottom * st}, n, {u, 0.0f});
  }

  // cap = centre vertex + rim vertices, normal (0, ny, 0); bottom cap V is
  // mirrored so its texture isn't flipped when viewed from below
  auto push_cap = [&](float y, float radius, float ny) {
    const unsigned int centre = static_cast<unsigned int>(vertices.size());
    vertices.push_back(make_vertex({0, y, 0}, {0, ny, 0}, {0.5f, 0.5f}));

    for (int s = 0; s <= slices; ++s) {
      float theta = static_cast<float>(s) / slices * glm::two_pi<float>();
      float ct = std::cos(theta), st = std::sin(theta);
      vertices.push_back(
          make_vertex({radius * ct, y, radius * st}, {0, ny, 0},
                      {0.5f + 0.5f * ct, 0.5f + 0.5f * ny * st}));
    }
    return centre;
  };

  const bool has_top = r_top > 0.0f;
  const bool has_bot = r_bottom > 0.0f;
  const unsigned int top_centre =
      has_top ? push_cap(half_height, r_top, 1.0f) : 0;
  const unsigned int bot_centre =
      has_bot ? push_cap(-half_height, r_bottom, -1.0f) : 0;

  std::vector<unsigned int> indices;
  indices.reserve(static_cast<size_t>(slices) * 12);
  for (int s = 0; s < slices; ++s) {
    unsigned int a = side_top + s, b = side_top + s + 1;
    unsigned int c = side_bot + s, d = side_bot + s + 1;

    // same winding as triangulate_quad_cw: (a,b,c) + (b,d,c). (a,b,c) spans
    // the top rim (zero-area when r_top == 0), (b,d,c) the bottom rim
    if (has_top) {
      indices.insert(indices.end(), {a, b, c});
    }
    if (has_bot) {
      indices.insert(indices.end(), {b, d, c});
    }
  }
  if (has_top) {
    // top cap outward normal is +Y: (centre, rim[s+1], rim[s])
    const unsigned int rim0 = top_centre + 1;
    for (int s = 0; s < slices; ++s) {
      indices.insert(indices.end(), {top_centre, rim0 + s + 1, rim0 + s});
    }
  }
  if (has_bot) {
    // bottom cap outward normal is -Y: (centre, rim[s], rim[s+1])
    const unsigned int rim0 = bot_centre + 1;
    for (int s = 0; s < slices; ++s) {
      indices.insert(indices.end(), {bot_centre, rim0 + s, rim0 + s + 1});
    }
  }

  base_model = std::make_shared<Model>(Model({Mesh(vertices, indices)}));
}

/*
 Cylindrical band between two equators, plus a hemisphere fanning out to a pole
 at each end. No flat caps, fully smooth. radius + half_height == 1 keeps it
 within [-1, 1].
*/
void Capsule::generate_capsule(int cap_rings, int slices, float radius,
                               float half_height) {
  using namespace mesh;

  if (cap_rings < 1 || slices < 3 || radius <= 0.0f || half_height < 0.0f) {
    throw std::invalid_argument("Capsule: invalid parameters (cap_rings " +
                                std::to_string(cap_rings) + ", slices " +
                                std::to_string(slices) + ", radius " +
                                std::to_string(radius) + ", half_height " +
                                std::to_string(half_height) + ")");
  }

  this->radius = radius;
  this->half_height = half_height;

  const int width = slices + 1;
  const int rows_per_hemi = cap_rings + 1;

  const unsigned int top_base = 0;
  const unsigned int bot_base =
      static_cast<unsigned int>(rows_per_hemi * width);

  std::vector<Vertex> vertices(static_cast<size_t>(2 * rows_per_hemi) * width);

  // row 0 = equator (y = +-h), row cap_rings = pole (y = +-(h + r));
  // sign = +1 builds the top hemisphere, -1 the bottom
  auto push_hemisphere = [&](unsigned int base, float sign) {
    for (int r = 0; r <= cap_rings; ++r) {
      float t = static_cast<float>(r) / cap_rings;
      float phi = glm::half_pi<float>() * (1.0f - t);
      float sin_phi = std::sin(phi), cos_phi = std::cos(phi);

      for (int s = 0; s <= slices; ++s) {
        float theta = static_cast<float>(s) / slices * glm::two_pi<float>();
        glm::vec3 n(sin_phi * std::cos(theta), sign * cos_phi,
                    sin_phi * std::sin(theta));
        glm::vec3 pos(radius * n.x, sign * half_height + radius * n.y,
                      radius * n.z);
        glm::vec2 uv(static_cast<float>(s) / slices, 0.5f + sign * 0.5f * t);

        vertices[base + static_cast<size_t>(r) * width + s] =
            make_vertex(pos, n, uv);
      }
    }
  };
  push_hemisphere(top_base, 1.0f);
  push_hemisphere(bot_base, -1.0f);

  std::vector<unsigned int> indices;
  indices.reserve(static_cast<size_t>(2 * cap_rings + 1) * slices * 6);

  // top hemisphere: rows run toward the +Y pole -> reversed winding
  for (int row = 0; row < cap_rings; ++row) {
    for (int s = 0; s < slices; ++s) {
      triangulate_quad_ccw(indices, top_base + row * width + s,
                           top_base + row * width + s + 1,
                           top_base + (row + 1) * width + s,
                           top_base + (row + 1) * width + s + 1);
    }
  }

  // cylinder band: top equator to bottom equator
  for (int s = 0; s < slices; ++s) {
    triangulate_quad_cw(indices, top_base + s, top_base + s + 1, bot_base + s,
                        bot_base + s + 1);
  }

  // bottom hemisphere: rows run toward the -Y pole -> forward winding
  for (int row = 0; row < cap_rings; ++row) {
    for (int s = 0; s < slices; ++s) {
      triangulate_quad_cw(indices, bot_base + row * width + s,
                          bot_base + row * width + s + 1,
                          bot_base + (row + 1) * width + s,
                          bot_base + (row + 1) * width + s + 1);
    }
  }

  base_model = std::make_shared<Model>(Model({Mesh(vertices, indices)}));
}
