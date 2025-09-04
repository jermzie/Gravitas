# Gravitas
3D Physics Engine


## About The Project

## Getting Started

### Dependencies

### Installation
1. Clone the repo
```
git clone https://github.com/jermzie/Gravitas.git
```
2. Link and install libraries with vcpkg
```
cmake -S . -B build -DCMAKE_TOOLCHAIN_FILE=C:/path/to/vcpkg/scripts/buildsystems/vcpkg.cmake
```

## Usage

### Importing Models

## Roadmap
- [ ] Fix centre of mass and inertia tensor computation
- [ ] Fix edge-edge collisions (SAT)
- [ ] Add representation for concave polyhedra as compound of convex polyhedra
- [ ] Add broad phase collision detection (BVH/Octrees)
- [ ] Add collision response (Sequential Impulse/PGS iterative solvers)
- [ ] Add better lighting?
- [ ] Add ground plane/grid
- [ ] Add debugger to visualize points, vectors, & planes
- [ ] Add mouse dragging force accumulation
- [ ] Add more ImGui functionality

## Acknowledgements

Dirk Gregorius, my beloved.


