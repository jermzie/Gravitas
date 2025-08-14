# CMakeList.txt : CMake project for Gravitas, include source and define
# project specific logic here.
#
cmake_minimum_required (VERSION 3.10)

# Enable Hot Reload for MSVC compilers if supported.
if (POLICY CMP0141)
  cmake_policy(SET CMP0141 NEW)
  set(CMAKE_MSVC_DEBUG_INFORMATION_FORMAT "$<IF:$<AND:$<C_COMPILER_ID:MSVC>,$<CXX_COMPILER_ID:MSVC>>,$<$<CONFIG:Debug,RelWithDebInfo>:EditAndContinue>,$<$<CONFIG:Debug,RelWithDebInfo>:ProgramDatabase>>")
endif()

project (Gravitas)
set(EXECUTABLE_NAME gravitas)
add_compile_definitions(PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
add_executable (${EXECUTABLE_NAME} "./src/Gravitas.cpp" "./src/stb_image.cpp" "./physics/QuickHull.cpp")

set_property(TARGET ${EXECUTABLE_NAME} PROPERTY CXX_STANDARD 17)

# path for cmake config files
# set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/build/vcpkg_installed/x64-windows/share")

find_package(assimp REQUIRED)
find_package(ZLIB REQUIRED)
find_package(BZip2 REQUIRED)
find_package(EXPAT REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(glm CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)

target_link_libraries(${EXECUTABLE_NAME} PRIVATE
  assimp::assimp
  ZLIB::ZLIB
  BZip2::BZip2
  EXPAT::EXPAT
  glad::glad
  glm::glm
  glfw
  imgui::imgui
)

# target_include_directories(${EXECUTABLE_NAME} PRIVATE include)
