# gravitas_demo: OpenGL sandbox app
add_executable(gravitas_demo
  main.cpp
  third_party/stb_image.cpp
)

target_compile_definitions(gravitas_demo PRIVATE
  RESOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/resources"
)

target_include_directories(gravitas_demo PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/third_party
)

target_link_libraries(gravitas_demo PRIVATE
  gravitas
  glad::glad
  glfw
  assimp::assimp
  imgui::imgui
)
