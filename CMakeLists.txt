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
# gravitas: physics library
# ---------------------------------------------------------------------------
add_library(gravitas STATIC
  gravitas/src/Broadphase.cpp
  gravitas/src/ConstraintSolver.cpp
  gravitas/src/Contacts.cpp
  gravitas/src/Entity.cpp
  gravitas/src/Narrowphase.cpp
  gravitas/src/PhysicsEngine.cpp
  gravitas/src/Poly.cpp
  gravitas/src/Primitive.cpp
  gravitas/src/Quadric.cpp
  gravitas/src/QuickHull.cpp
  gravitas/src/RigidBody.cpp
)

target_compile_features(gravitas PUBLIC cxx_std_20)
target_include_directories(gravitas PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/gravitas/include
)

target_link_libraries(gravitas PUBLIC
  glm::glm
)

# ---------------------------------------------------------------------------
# demo: OpenGL sandbox app
# ---------------------------------------------------------------------------
add_executable(gravitas_demo
  demo/src/main.cpp
  demo/lib/stb_image.cpp
)

target_compile_definitions(gravitas_demo PRIVATE
  RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/demo/resources"
)

target_include_directories(gravitas_demo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/demo/include
  ${CMAKE_CURRENT_SOURCE_DIR}/demo/lib/include
)

target_link_libraries(gravitas_demo PRIVATE
  gravitas
  glad::glad
  glfw
  assimp::assimp
  imgui::imgui
)
