# MCmesher
by Kyle J Burgess (github.com/kyy13)


![disp](https://user-images.githubusercontent.com/58697284/154577110-bcabfc84-7365-446f-804d-63e563f7a53b.png)

## Summary

MCmesher is a lightweight marching cubes mesh generator for c++, c#, and Unity.

## Features

<details>
<summary>
details
</summary>

#### Mesh Generation
- Generate 3D Marching Cubes meshes with a 3D data field and custom ISO level.
- Produces index data, vertex data, and normal vector data
- Choose between triangle face normal vectors, and triangle vertex normal vectors.

#### Algorithms
- Trace rays through a scalar field and detect virtual mesh collision
</details>

## Quick Start

<details>
<summary>
details
</summary>

#### Unity Steps
1. Download the latest release from the releases page.
2. Drop the DLL into the your *Assets* folder
3. Drop the MCmesher.cs wrapper file into your *Assets* folder.

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

---

MCmesher uses lookup tables modified from the transvoxel algorithm [1].

[1] Lengyel, Eric. “Voxel-Based Terrain for Real-Time Virtual Simulations”. PhD diss., University of California at Davis, 2010.
