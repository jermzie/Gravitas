cmake_minimum_required(VERSION 3.21)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project(Gravitas LANGUAGES C CXX)

find_package(assimp CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

# ---------------------------------------------------------------------------
# gravitas: physics library (no rendering dependencies)
# ---------------------------------------------------------------------------
add_library(gravitas STATIC
  gravitas/geometry/primitive.cpp
  gravitas/geometry/quickhull.cpp
  gravitas/physics/broadphase.cpp
  gravitas/physics/collide_poly.cpp
  gravitas/physics/collide_quadric.cpp
  gravitas/physics/constraint_solver.cpp
  gravitas/physics/contacts.cpp
  gravitas/physics/entity.cpp
  gravitas/physics/narrowphase.cpp
  gravitas/physics/physics_engine.cpp
  gravitas/physics/rigid_body.cpp
)

target_compile_features(gravitas PUBLIC cxx_std_20)

target_include_directories(gravitas PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/gravitas/core
  ${CMAKE_CURRENT_SOURCE_DIR}/gravitas/geometry
  ${CMAKE_CURRENT_SOURCE_DIR}/gravitas/physics
)

target_link_libraries(gravitas PUBLIC
  glm::glm
)

# ---------------------------------------------------------------------------
# gravitas_demo: OpenGL sandbox app
# ---------------------------------------------------------------------------
add_executable(gravitas_demo
  demo/main.cpp
  demo/third_party/stb_image.cpp
)

target_compile_definitions(gravitas_demo PRIVATE
  RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/demo/resources"
)

target_include_directories(gravitas_demo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/demo/include
  ${CMAKE_CURRENT_SOURCE_DIR}/demo/third_party
)

target_link_libraries(gravitas_demo PRIVATE
  gravitas
  glad::glad
  glfw
  assimp::assimp
  imgui::imgui
)
