#pragma once

#include <glm/glm.hpp>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "Debugger.hpp"
#include "Entity.hpp"
#include "Mesh.hpp"
#include "Model.hpp"
#include "Primitive.hpp"
#include "Shader.hpp"
#include "SurfaceMesh.hpp"

/*
  Maps physics primitives to GPU models. Procedural primitives are uploaded
  lazily from their SurfaceMesh (shared surfaces upload once); primitives
  built from a loaded file register that Model explicitly.
 */
class RenderCache {
  struct Upload {
    std::weak_ptr<const SurfaceMesh> source; // detects address reuse
    std::shared_ptr<Model> model;
  };

  std::unordered_map<const Primitive *, std::shared_ptr<Model>> overrides;
  std::unordered_map<const SurfaceMesh *, Upload> uploads;

  static std::shared_ptr<Model> upload(const SurfaceMesh &surface) {
    std::vector<Vertex> vertices(surface.vertices.size());
    for (size_t i = 0; i < vertices.size(); ++i) {
      vertices[i] = Vertex{};
      vertices[i].position = surface.vertices[i].position;
      vertices[i].normal = surface.vertices[i].normal;
      vertices[i].texture_coordinates = surface.vertices[i].uv;
    }
    return std::make_shared<Model>(Model({Mesh(vertices, surface.indices)}));
  }

public:
  // draw `prim` with `model` instead of its SurfaceMesh
  void set_model(const Primitive *prim, std::shared_ptr<Model> model) {
    overrides[prim] = std::move(model);
  }

  // null if the primitive has neither an override nor a surface
  Model *get(const Primitive &prim) {
    if (auto it = overrides.find(&prim); it != overrides.end()) {
      return it->second.get();
    }

    const auto &surface = prim.get_base_surface();
    if (!surface) {
      return nullptr;
    }

    Upload &entry = uploads[surface.get()];
    if (!entry.model || entry.source.expired()) {
      entry.source = surface;
      entry.model = upload(*surface);
    }
    return entry.model.get();
  }
};

// draws each child at (parent * child local_xfrm), uploaded to `uniform` -
// for a gWVP-style shader, pass projection * view * body matrix as parent
inline void draw_entity(RenderCache &cache, const Entity &entity,
                        Shader &shader, const glm::mat4 &parent,
                        const std::string &uniform = "model",
                        Debugger *debug = nullptr) {

  // base surface is untransformed - place it in the geometry frame, then the
  // parent (body) frame, same chain the raycast inverts
  for (const auto &child : entity.geom->get_children()) {

    if (debug) {
      debug->draw_aabb(transform_arvo(child->geom_aabb(), parent),
                       glm::vec4(1.0f, 0.5f, 0.0f, 1.0f));
    }

    if (Model *model = cache.get(*child)) {
      shader.set_mat4(uniform, parent * child->get_local_xfrm().get_matrix());
      model->draw(shader);
    }
  }
}
