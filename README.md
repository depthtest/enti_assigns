# enti_assigns

This repository contains some of the assignments on the physics and the computer graphics assignments at ENTI-UB from years 2016 to 2020. All assignments share the same glframework base. Visual Studio solutions tested on VS2022.

## phys
There were 4 assignments, P1 to P4, which can be changed at the physics.cpp file, commenting and uncommenting the corresponding function calls (haven't bothered to parameterize the executable, so requires recompilation each time).

- P1 - Particle simulation
- P2 - Mesh Simulation with Springs
- P3 - Rigid Body Objects
- P4 - Gerstner Waves

## cg
There were multiple assingments from camera parameters to shader stages and transforms (vertex, geometry, fragment).
The only code I still have:

- instancing - render a moving mesh composed of two different PLY models (Stanford bunny and Suzzanne, compare between render each model in the mesh, instance them or do multidrawindirect which further reduces draw calls
- computeshaders - implement a simple particle simulation through compute shaders

-----
The MIT license of this repository only affects to the GLframework and the assignment solutions, not on the depending libraries (GLM, GLEW, SDL, Dear IMGUI), each having their own license.
