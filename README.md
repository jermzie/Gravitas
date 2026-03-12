# Gravitas
3D physics engine built in OpenGL


## About The Project

Built a 3D physics engine to learn more about graphics APIs and physics simulations.

broad-phase collision detection uses dBVH tree to compute colliding pairs.
narrow-phase collision detection uses SAT to compute contacts and is optimized via Gauss maps to reduce edge-edge checks.
solver uses Sequential Impulse to resolve constraints.

supported collision primitives are spheres, cylinders, capsules, cones, and convex polyhedrons.

For non-convex 3D models, Quickhull is used to generate convex hull mesh for collisions.

## Installation

### Dependencies

### Windows
1. Clone the repo
```
git clone https://github.com/jermzie/Gravitas.git
```
2. Link and build libraries with vcpkg (may take a while)
```
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
```

## Usage

### Demo Scenes

### Scene Editing

### Importing Models

## Roadmap
- [ ] Add representation for concave polyhedra as compound of convex polyhedra
- [ ] Add mouse dragging force accumulation
- [ ] Add more ImGui functionality
- [ ] Add more Constraints and Joints to solver
- [ ] Add threads to parallelize tasks
- [ ] Add scene serialization and basic editing



