# gravitas: physics library (no rendering dependencies)
add_library(gravitas STATIC
  geometry/primitive.cpp
  geometry/quickhull.cpp
  physics/broadphase.cpp
  physics/collide_poly.cpp
  physics/collide_quadric.cpp
  physics/constraint_solver.cpp
  physics/contacts.cpp
  physics/entity.cpp
  physics/narrowphase.cpp
  physics/physics_engine.cpp
  physics/rigid_body.cpp
)

target_compile_features(gravitas PUBLIC cxx_std_20)

# Headers are included as "gravitas/<module>/<file>.hpp".
target_include_directories(gravitas PUBLIC ${PROJECT_SOURCE_DIR})

target_link_libraries(gravitas PUBLIC glm::glm)
