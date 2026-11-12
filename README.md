# Gravitas
3D physics engine using OpenGL, GLFW


## About The Project
Built a 3D physics engine to learn more about graphics APIs and physics simulations.

- broadphase collision detection uses dBVH tree to compute colliding pairs.
- narrowphase collision detection uses SAT to compute contacts and is optimized via Gauss maps to reduce edge-edge checks.
- solver uses Sequential Impulse to resolve constraints.

supported collision primitives are spheres, capsules, and convex polyhedrons.

For imported non-convex 3D models, Quickhull is used to generate convex hull mesh for collider.

## Installation

Requires CMake 3.21+, Ninja, a C++20 compiler, and [vcpkg](https://github.com/microsoft/vcpkg) with `VCPKG_ROOT` set to its install directory (absolute path, e.g. `export VCPKG_ROOT="$HOME/vcpkg"`).

1. Clone the repo
```
git clone https://github.com/jermzie/Gravitas.git
cd Gravitas/
```
2. Configure (vcpkg fetches and builds dependencies on first run, may take a while) and build
```
cmake --preset debug          # or: release
cmake --build --preset debug
```
On Windows, run these from a Visual Studio Developer prompt, or open the folder in Visual Studio / VS Code, which pick up the presets automatically.

Machine-specific overrides go in `CMakeUserPresets.json` (git-ignored).

## Usage

### Importing Models


## To-Do
- Reduce solver instability
- Parallelize broadphase & narrowphase
- Add simulation islands
- Add more constraints/joints
  - distance
  - hinge
  - ball & socket
  - prismatic
  - cylindrical
- Scene serialization and basic editing



