// MCmesher
// Kyle J Burgess

#ifndef K13_MARCHING_CUBES_H
#define K13_MARCHING_CUBES_H

#include "Vector.h"
#include <cstdint>

extern "C"
{
    struct Mesh;

    // Describes the dimensions of a float* set of values
    struct ScalarField
    {
        ScalarField();

        ScalarField(uint32_t width, uint32_t height, uint32_t depth);

        uint32_t width; // x axis
        uint32_t height; // y axis
        uint32_t depth; // z axis
    };

    // Defines a region of a scalar field
    struct CubeSlice
    {
        CubeSlice();

        CubeSlice(uint32_t x0, uint32_t y0, uint32_t z0, uint32_t width, uint32_t height, uint32_t depth);

        uint32_t x0;
        uint32_t y0;
        uint32_t z0;

        uint32_t width; // x axis (in cubes, e.g. max is field width - 1)
        uint32_t height; // y axis (in cubes, e.g. max is field width - 1)
        uint32_t depth; // z axis (in cubes, e.g. max is field width - 1)
    };

    // Create a mesh and return a handle to the mesh
    Mesh* __cdecl CreateMesh();

    // Delete a mesh invalidating the handle
    void __cdecl DeleteMesh(Mesh* mesh);

    // Generate geometry for a mesh
    void __cdecl GenerateMesh(
        Mesh* mesh,
        const float* data,
        const ScalarField& field,
        const CubeSlice& slice,
        float isoLevel,
        bool computeFaceNormals,
        bool computeVertexNormals);

    // Count the number of vertices in the mesh
    uint32_t __cdecl CountVertices(const Mesh* mesh);

    // Returns a pointer to the mesh vertices
    const Vector3* __cdecl GetVertices(const Mesh* mesh);

    // Copies vertices into another array
    void __cdecl CopyVertices(const Mesh* mesh, Vector3* dst);

    // Count the number of face normals in the mesh
    uint32_t __cdecl CountFaceNormals(const Mesh* mesh);

    // Returns a pointer to the mesh face normal vectors
    const Vector3* __cdecl GetFaceNormals(const Mesh* mesh);

    // Copies face normals into another array
    void __cdecl CopyFaceNormals(const Mesh* mesh, Vector3* dst);

    // Count the number of vertex normals in the mesh
    uint32_t __cdecl CountVertexNormals(const Mesh* mesh);

    // Returns a pointer to the mesh vertex normal vectors
    const Vector3* __cdecl GetVertexNormals(const Mesh* mesh);

    // Copies vertex normals into another array
    void __cdecl CopyVertexNormals(const Mesh* mesh, Vector3* dst);

    // Count the number of indices in the mesh
    uint32_t __cdecl CountIndices(const Mesh* mesh);

    // Returns a pointer to the mesh indices
    const uint32_t* __cdecl GetIndices(const Mesh* mesh);

    // Copies mesh indices into another array
    void __cdecl CopyIndices(const Mesh* mesh, uint32_t* dst);
}

#endif
