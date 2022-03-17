# mc-mesher
by Kyle J Burgess (github.com/kyy13)


![disp](https://user-images.githubusercontent.com/58697284/154577110-bcabfc84-7365-446f-804d-63e563f7a53b.png)

## Summary

mc-mesher is a lightweight marching cubes mesh generator for `c`, `c++`, `c#`, and `Unity`.

## Features

<details>
<summary>
details
</summary>

#### Mesh Generation
- Generate 3D Marching Cubes meshes from a 3D scalar field and custom ISO level.
- Produces mesh indices, vertices, and normal vector data
- Choose between face normal vectors, and vertex normal vectors.

#### Algorithms
- Trace rays through a scalar field used to generate a mesh without requiring the mesh to be generated first; detects collision with the mesh and returns the point of intersection. This method will detect collision much faster than ray tracing implementations operating on the mesh data itself.
</details>

## Performance

<details>
<summary>
details
</summary>

#### Notes
- mc-mesher is designed to produce indexed marching cubes meshes with minimal vertices.
- Meshes generated with face normals tend to be much larger due to the inability to index vertices that share the same position, because they have different normal vectors.
- Mesh generation functions have `O(X*Y*Z)` time complexity where *X*, *Y*, and *Z* refer to the dimensions of the scalar field used to produce the mesh.
- Meshes with face normals have `O(1)` temporary memory usage.
- Meshes with vertex normals have `O(X*Y)` temporary memory usage.

</details>

## Quick Start

<details>
<summary>
details
</summary>

#### Unity Steps
1. Download the latest release from the releases page.
2. Drop the DLL into the your *Assets* folder
3. Drop the mc_mesher.cs wrapper file into your *Assets* folder.

</details>

## Build

<details>
<summary>
details
</summary>

#### Notes
* There are precompiled binaries available on the *releases* page.
* See the build scripts in the *scripts* folder for examples on how to build with cmake.<br>The scripts are setup to target `mingw-w64` for `64-bit windows`.

#### Requirements
1. A working `c++17` (or higher) compiler.
2. `CMake` version 3.7 or higher

#### Steps
1. Run cmake with DCMAKE_BUILD_TYPE=Release to generate the build files
2. Run make to compile

</details>

##

mc-mesher uses lookup tables modified from the transvoxel algorithm [1].

[1] Lengyel, Eric. “Voxel-Based Terrain for Real-Time Virtual Simulations”. PhD diss., University of California at Davis, 2010.
