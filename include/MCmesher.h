// MCmesher
// Kyle J Burgess

#ifndef MC_MESHER_H
#define MC_MESHER_H

#include "Vector.h"
#include <cstdint>

extern "C"
{
    struct Mesh;

    // Create a mesh and return a handle to the mesh
    Mesh* __cdecl CreateMesh();

    // Delete a mesh invalidating the handle
    void __cdecl DeleteMesh(Mesh* mesh);

    // Generate geometry for a mesh
    void __cdecl GenerateMesh(
        Mesh* mesh,
        const float* data,
        Vector3<uint32_t> dataSize,   // size of scalar field data (points)
        Vector3<uint32_t> meshOrigin, // mesh origin in scalar field (cubes)
        Vector3<uint32_t> meshSize,   // mesh size in scalar field (cubes)
        float isoLevel,
        bool vertexNormals = true);

    // Count the number of vertices in the mesh
    uint32_t __cdecl CountVertices(const Mesh* mesh);

    // Returns a pointer to the mesh vertices
    const Vector3<float>* __cdecl GetVertices(const Mesh* mesh);

    // Copies vertices into another array
    void __cdecl CopyVertices(const Mesh* mesh, Vector3<float>* dst);

    // Count the number of normals in the mesh
    uint32_t __cdecl CountNormals(const Mesh* mesh);

    // Returns a pointer to the mesh normal vectors
    const Vector3<float>* __cdecl GetNormals(const Mesh* mesh);

    // Copies normals into another array
    void __cdecl CopyNormals(const Mesh* mesh, Vector3<float>* dst);

    // Count the number of indices in the mesh
    uint32_t __cdecl CountIndices(const Mesh* mesh);

    // Returns a pointer to the mesh indices
    const uint32_t* __cdecl GetIndices(const Mesh* mesh);

    // Copies mesh indices into another array
    void __cdecl CopyIndices(const Mesh* mesh, uint32_t* dst);
}

#endif
