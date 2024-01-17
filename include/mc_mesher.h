// mc-mesher
// Kyle J Burgess

#ifndef MC_MESHER_H
#define MC_MESHER_H

#include "McmVector.h"
#include <cstdint>

extern "C"
{
    // Stores vertices, indices, and normals
    // generated by GenerateMesh functions
    struct                        McmMeshBuffer;

    // Result of mc-mesher functions
    enum                          McmResult : uint32_t
    {
        MCM_SUCCESS                   = 0x0,            // Function was successful
        MCM_FAILURE                   = 0x1,            // False condition, primarily for geometry functions
        MCM_MESH_BUFFER_IS_NULL       = 0x2,            // Error, the mesh buffer passed to the function was NULL
        MCM_OUT_OF_BOUNDS_X           = 0x3,            // Error, an argument passed to the function was out of bounds in the x-axis
        MCM_OUT_OF_BOUNDS_Y           = 0x4,            // Error, an argument passed to the function was out of bounds in the y-axis
        MCM_OUT_OF_BOUNDS_Z           = 0x5,            // Error, an argument passed to the function was out of bounds in the z-axis
    };

    // Mesh generation flags
    enum                          McmFlag   : uint32_t
    {
        MCM_WINDING_RHCS_CW           = 0x0,            // Vertex winding order is CW for a right-handed coordinate system
        MCM_WINDING_RHCS_CCW          = 0x1,            // Vertex winding order is CCW for a right-handed coordinate system
        MCM_WINDING_LHCS_CW           = MCM_WINDING_RHCS_CCW,
        MCM_WINDING_LHCS_CCW          = MCM_WINDING_RHCS_CW,
        MCM_VERTEX_NORMALS            = 0x0,            // Calculate per-vertex normals from the voxel field
        MCM_FACE_NORMALS              = 0x2,            // Calculate per-vertex normals using the containing triangles' normals
        MCM_EDGE_LERP                 = 0x0,            // Lerp vertex position between edges based on weight
        MCM_EDGE_CENTER               = 0x4,            // Place vertex in the middle of the edge
    };

    using McmFlags = uint32_t;

    // Create an McmMeshBuffer and return its handle (pointer)
    McmMeshBuffer*        __cdecl mcmCreateMeshBuffer();

    // Delete an McmMeshBuffer, invalidating the handle
    void                  __cdecl mcmDeleteMeshBuffer(
        McmMeshBuffer*                meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Generate a marching cubes mesh from a scalar float32 field, and store the results in an McmMeshBuffer
    McmResult             __cdecl mcmGenerateMesh(
        McmMeshBuffer*                meshBuffer,     // Handle to a valid mcmMeshBuffer object
        const float*                  data,           // 3D field of scalar floating-point values as a contiguous array
        Vector3<uint32_t>             dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        Vector3<uint32_t>             meshOrigin,     // Origin of the mesh to generate (in cubes) within the 3D field
        Vector3<uint32_t>             meshSize,       // Size of the mesh to generate (in cubes) within the 3D field
        float                         isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        McmFlags                      flags);         // Mesh generation flags

    // Determine if a scalar field of floats contains a point (the mesh does not need to be generated)
    // If the mesh contains the point, then mcmMeshContainsPoint returns MCM_SUCCESS
    // otherwise, mcmMeshContainsPoint returns MCM_FAILURE
    McmResult             __cdecl mcmMeshContainsPoint(
        const float*                  data,           // 3D field of scalar floating-point values as a contiguous array
        Vector3<uint32_t>             dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        float                         isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        Vector3<float>                point,          // The point to check
        McmFlags                      flags);         // Mesh generation flags

    // Intersect a scalar field of floats with a ray (gives the same results as mesh-ray intersection except faster, and the mesh does not need to be generated)
    // If an intersection occurs, then mcmMeshIntersectRay returns MCM_SUCCESS, and the point of intersection is set,
    // otherwise, mcmMeshIntersectRay returns MCM_FAILURE
    McmResult             __cdecl mcmMeshIntersectRay(
        const float*                  data,           // 3D field of scalar floating-point values as a contiguous array
        Vector3<uint32_t>             dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        float                         isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        Vector3<float>                rayPos,         // Starting point of the ray
        Vector3<float>                rayDir,         // Direction of the ray (does not need to be normalized)
        McmFlags                      flags,          // Mesh generation flags
        Vector3<float>&               pIntersect);    // The point of intersection if an intersection occurred

    // Intersect a scalar field of floats with a segment (gives the same results as mesh-segment intersection except faster, and the mesh does not need to be generated)
    // If an intersection occurs, then mcmMeshIntersectSegment returns MCM_SUCCESS, and the point of intersection is set,
    // otherwise, mcmMeshIntersectSegment returns MCM_FAILURE
    McmResult             __cdecl mcmMeshIntersectSegment(
        const float*                  data,           // 3D field of scalar floating-point values as a contiguous array
        Vector3<uint32_t>             dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        float                         isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        Vector3<float>                segPos,         // Starting point of the segment
        Vector3<float>                segEnd,         // End point of the segment
        McmFlags                      flags,          // Mesh generation flags
        Vector3<float>&               pIntersect);    // The point of intersection if an intersection occurred

    // Count the number of vertices in the mesh
    uint32_t              __cdecl mcmCountVertices(
        const McmMeshBuffer*          meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Returns a pointer to the mesh vertices
    const Vector3<float>* __cdecl mcmGetVertices(
        const McmMeshBuffer*          meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Copies vertices into a buffer
    void                  __cdecl mcmCopyVertices(
        const McmMeshBuffer*          meshBuffer,     // Handle to a valid mcmMeshBuffer object
        Vector3<float>*               dstBuffer);     // Destination buffer of Vector3<float> (allocated size must be >= mcmCountVertices(meshBuffer))

    // Count the number of normals in the mesh
    uint32_t              __cdecl mcmCountNormals(
        const McmMeshBuffer*          meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Returns a pointer to the mesh normal vectors
    const Vector3<float>* __cdecl mcmGetNormals(
        const McmMeshBuffer*          meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Copies normals into a buffer
    void                  __cdecl mcmCopyNormals(
        const McmMeshBuffer*          meshBuffer,     // Handle to a valid mcmMeshBuffer object
        Vector3<float>*               dstBuffer);     // Destination buffer of Vector3<float> (allocated size must be >= mcmCountNormals(meshBuffer))

    // Count the number of indices in the mesh
    uint32_t              __cdecl mcmCountIndices(
        const McmMeshBuffer*          meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Returns a pointer to the mesh indices
    const uint32_t*       __cdecl mcmGetIndices(
        const McmMeshBuffer*          meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Copies indices into a buffer
    void                  __cdecl mcmCopyIndices(
        const McmMeshBuffer*          meshBuffer,     // Handle to a valid mcmMeshBuffer object
        uint32_t*                     dstBuffer);     // Destination buffer of uint32_t (allocated size must be >= mcmCountIndices(meshBuffer))
}

#endif
