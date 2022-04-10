// mc-mesher
// Kyle J Burgess

using System;
using System.Runtime.InteropServices;
using UnityEngine;
using UnityEngine.Rendering;
using Unity.Collections.LowLevel.Unsafe;

public class McmMeshBuffer : IDisposable
{
    public enum McmFlag : uint
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

    public McmMeshBuffer()
    {
        m_meshBuffer = McmCreateMeshBuffer();
    }

    public void Dispose()
    {
        TryDispose();
        GC.SuppressFinalize(this);
    }

    // Generate a procedural mesh (with 3D array)
    // Pass an existing meshFilter.mesh into GenerateMesh as a reference,
    // avoid using Mesh.Clear() and creating a new Mesh(), because GenerateMesh() is designed to
    // handle overwriting existing meshes.
    // voxelData       the scalara data field of the marching cubes mesh
    // dataSize        size of scalar field data (points)
    // meshOrigin      mesh origin in scalar field (cubes)
    // meshSize        mesh size in scalar field (cubes)
    // isoLevel        voxelData value > isoLevel means it's inside the surface
    // vertexNormals   true means use iterpolated vertex normals, false means use face normals
    public void GenerateMesh(
        ref Mesh mesh,
        float[,,] voxelData,
        Vector3u dataSize,
        Vector3u meshOrigin,
        Vector3u meshSize,
        float isoLevel,
        McmFlag flags = 0)
    {
        McmResult r = McmGenerateMesh(m_meshBuffer, voxelData, dataSize, meshOrigin, meshSize, isoLevel, flags);

        if (r != McmResult.MCM_SUCCESS)
        {
            throw new Exception(r.ToString());
        }

        ApplyMesh(ref mesh, dataSize);
    }

    public void GenerateMesh(
        ref Mesh mesh,
        float[] voxelData,
        Vector3u dataSize,
        Vector3u meshOrigin,
        Vector3u meshSize,
        float isoLevel,
        McmFlag flags = 0)
    {
        McmResult r = McmGenerateMesh(m_meshBuffer, voxelData, dataSize, meshOrigin, meshSize, isoLevel, flags);

        if (r != McmResult.MCM_SUCCESS)
        {
            throw new Exception(r.ToString());
        }

        ApplyMesh(ref mesh, dataSize);
    }

    // Determine if a scalar field of floats contains a point (the mesh does not need to be generated)
    // If the mesh contains the point, then mcmMeshContainsPoint returns MCM_SUCCESS
    // otherwise, mcmMeshContainsPoint returns MCM_FAILURE
    public static bool MeshContainsPoint(float[] data, Vector3u dataSize, float isoLevel, Vector3 point, McmFlag flags)
    {
        return McmMeshContainsPoint(data, dataSize, isoLevel, point, flags) == McmResult.MCM_SUCCESS;
    }

    public static bool MeshContainsPoint(float[,,] data, Vector3u dataSize, float isoLevel, Vector3 point, McmFlag flags)
    {
        return McmMeshContainsPoint(data, dataSize, isoLevel, point, flags) == McmResult.MCM_SUCCESS;
    }

    // Intersect a scalar field of floats with a ray (gives the same results as mesh-ray intersection except faster, and the mesh does not need to be generated)
    // If an intersection occurs, then returns the point of intersection, otherwise returns null
    public static Vector3? MeshIntersectRay(float[] data, Vector3u dataSize, float isoLevel, Vector3 rayPos, Vector3 rayDir, McmFlag flags)
    {
        Vector3 pIntersect;

        if (McmMeshIntersectRay(data, dataSize, isoLevel, rayPos, rayDir, flags, out pIntersect) == McmResult.MCM_SUCCESS)
        {
            return pIntersect;
        }

        return null;
    }

    public static Vector3? MeshIntersectRay(float[,,] data, Vector3u dataSize, float isoLevel, Vector3 rayPos, Vector3 rayDir, McmFlag flags)
    {
        Vector3 pIntersect;

        if (McmMeshIntersectRay(data, dataSize, isoLevel, rayPos, rayDir, flags, out pIntersect) == McmResult.MCM_SUCCESS)
        {
            return pIntersect;
        }

        return null;
    }

    // Intersect a scalar field of floats with a segment (gives the same results as mesh-segment intersection except faster, and the mesh does not need to be generated)
    // If an intersection occurs, then mcmMeshIntersectSegment returns MCM_SUCCESS, and the point of intersection is set,
    // otherwise, mcmMeshIntersectSegment returns MCM_FAILURE
    public static Vector3? MeshIntersectSegment(float[] data, Vector3u dataSize, float isoLevel, Vector3 segPos, Vector3 segEnd, McmFlag flags)
    {
        Vector3 pIntersect;

        if (McmMeshIntersectSegment(data, dataSize, isoLevel, segPos, segEnd, flags, out pIntersect) == McmResult.MCM_SUCCESS)
        {
            return pIntersect;
        }

        return null;
    }

    protected void ApplyMesh(ref Mesh mesh, Vector3u boundsMaxU)
    {
        var dataArray = Mesh.AllocateWritableMeshData(1);
        var data = dataArray[0];

        int vertexCount = (int)McmCountVertices(m_meshBuffer);
        int indexCount = (int)McmCountIndices(m_meshBuffer);

        data.SetVertexBufferParams(vertexCount,
            new VertexAttributeDescriptor(VertexAttribute.Position),
            new VertexAttributeDescriptor(VertexAttribute.Normal, stream: 1));

        var vertices = data.GetVertexData<Vector3>();

        unsafe
        {
            McmCopyVertices(m_meshBuffer, NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(vertices));
        }

        var normals = data.GetVertexData<Vector3>(1);

        unsafe
        {
            McmCopyNormals(m_meshBuffer, NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(normals));
        }

        data.SetIndexBufferParams(indexCount, IndexFormat.UInt32);

        var indices = data.GetIndexData<uint>();

        unsafe
        {
            McmCopyIndices(m_meshBuffer, NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(indices));
        }

        Vector3 boundsMax = new Vector3(
            (float)(boundsMaxU.x - 1),
            (float)(boundsMaxU.y - 1),
            (float)(boundsMaxU.z - 1));

        var subMeshDescriptor = new SubMeshDescriptor(0, indexCount);

        subMeshDescriptor.bounds.SetMinMax(Vector3.zero, boundsMax);
        subMeshDescriptor.firstVertex = 0;
        subMeshDescriptor.vertexCount = indexCount;

        data.subMeshCount = 1;
        data.SetSubMesh(0, subMeshDescriptor, MeshUpdateFlags.DontRecalculateBounds);

        Mesh.ApplyAndDisposeWritableMeshData(dataArray, mesh);

        mesh.RecalculateBounds();
    }

    ~McmMeshBuffer()
    {
        TryDispose();
    }

    protected virtual void TryDispose()
    {
        if (!m_disposed)
        {
            McmDeleteMeshBuffer(m_meshBuffer);
            m_disposed = true;
        }
    }

    protected IntPtr m_meshBuffer;

    protected bool m_disposed = false;

    // -------------------------------------------
    // mc-mesher C++ Library Methods and Structs
    // -------------------------------------------

    // Vector3 of uint
    [StructLayout(LayoutKind.Explicit)]
    public struct Vector3u
    {
        [FieldOffset(0)] public uint x;
        [FieldOffset(4)] public uint y;
        [FieldOffset(8)] public uint z;
    }

    // Result of mc-mesher functions
    protected enum McmResult : uint
    {
        MCM_SUCCESS                         = 0,            // Function was successful
        MCM_FAILURE                         = 1,            // False condition, primarily for geometry functions
        MCM_MESH_BUFFER_IS_NULL             = 2,            // Error, the mesh buffer passed to the function was NULL
        MCM_OUT_OF_BOUNDS_X                 = 3,            // Error, an argument passed to the function was out of bounds in the x-axis
        MCM_OUT_OF_BOUNDS_Y                 = 4,            // Error, an argument passed to the function was out of bounds in the y-axis
        MCM_OUT_OF_BOUNDS_Z                 = 5,            // Error, an argument passed to the function was out of bounds in the z-axis
    };

    // Create an McmMeshBuffer and return its handle (pointer)
    [DllImport("libmcmesher", EntryPoint = "mcmCreateMeshBuffer", CallingConvention = CallingConvention.Cdecl)]
    protected static extern IntPtr      McmCreateMeshBuffer();

    // Delete an McmMeshBuffer, invalidating the handle
    [DllImport("libmcmesher", EntryPoint = "mcmDeleteMeshBuffer", CallingConvention = CallingConvention.Cdecl)]
    protected static extern void        McmDeleteMeshBuffer(
        IntPtr                              meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Generate a marching cubes mesh with vertex normals, and store the results in an McmMeshBuffer
    [DllImport("libmcmesher", EntryPoint = "mcmGenerateMesh", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult   McmGenerateMesh(
        IntPtr                              meshBuffer,     // Handle to a valid mcmMeshBuffer object
        float[]                             data,           // 3D field of scalar floating-point values as a contiguous array
        Vector3u                            dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        Vector3u                            meshOrigin,     // Origin of the mesh to generate (in cubes) within the 3D field
        Vector3u                            meshSize,       // Size of the mesh to generate (in cubes) within the 3D field
        float                               isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        McmFlag                             flags);

    // Generate a marching cubes mesh with vertex normals, and store the results in an McmMeshBuffer
    [DllImport("libmcmesher", EntryPoint = "mcmGenerateMesh", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult   McmGenerateMesh(
        IntPtr                              meshBuffer,     // Handle to a valid mcmMeshBuffer object
        float[,,]                           data,           // 3D field of scalar floating-point values as a contiguous array
        Vector3u                            dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        Vector3u                            meshOrigin,     // Origin of the mesh to generate (in cubes) within the 3D field
        Vector3u                            meshSize,       // Size of the mesh to generate (in cubes) within the 3D field
        float                               isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        McmFlag                             flags);         // Mesh generation flags

    // Determine if a scalar field of floats contains a point (the mesh does not need to be generated)
    // If the mesh contains the point, then mcmMeshContainsPoint returns MCM_SUCCESS
    // otherwise, mcmMeshContainsPoint returns MCM_FAILURE
    [DllImport("libmcmesher", EntryPoint = "mcmMeshContainsPoint", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult   McmMeshContainsPoint(
        float[]                       data,                 // 3D field of scalar floating-point values as a contiguous array
        Vector3u                      dataSize,             // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        float                         isoLevel,             // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        Vector3                       point,                // The point to check
        McmFlags                      flags);               // Mesh generation flags

    // Determine if a scalar field of floats contains a point (the mesh does not need to be generated)
    // If the mesh contains the point, then mcmMeshContainsPoint returns MCM_SUCCESS
    // otherwise, mcmMeshContainsPoint returns MCM_FAILURE
    [DllImport("libmcmesher", EntryPoint = "mcmMeshContainsPoint", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult   McmMeshContainsPoint(
        float[,,]                     data,                 // 3D field of scalar floating-point values as a contiguous array
        Vector3u                      dataSize,             // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        float                         isoLevel,             // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        Vector3                       point,                // The point to check
        McmFlags                      flags);               // Mesh generation flags

    // Intersect a scalar field with a ray (gives the same results as mesh-ray intersection except faster, and the mesh does not need to be generated)
    // If an intersection occurs, then mcmRayIntersectMesh returns MCM_SUCCESS, and the point of intersection is set,
    // otherwise, mcmRayIntersectMesh returns MCM_FAILURE
    [DllImport("libmcmesher", EntryPoint = "mcmMeshIntersectRay", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult   McmMeshIntersectRay(
        float[]                             data,           // 3D field of scalar floating-point values as a contiguous array
        Vector3u                            dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        float                               isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        Vector3                             rayPos,         // Starting point of the ray
        Vector3                             rayDir,         // Direction of the ray (does not need to be normalized)
        McmFlag                             flags,          // Mesh generation flags
        out Vector3                         pIntersect);    // The point of intersection if an intersection occurred

    // Intersect a scalar field with a ray (gives the same results as mesh-ray intersection except faster, and the mesh does not need to be generated)
    // If an intersection occurs, then mcmRayIntersectMesh returns MCM_SUCCESS, and the point of intersection is set,
    // otherwise, mcmRayIntersectMesh returns MCM_FAILURE
    [DllImport("libmcmesher", EntryPoint = "mcmMeshIntersectRay", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult   McmMeshIntersectRay(
        float[,,]                           data,           // 3D field of scalar floating-point values as a contiguous array
        Vector3u                            dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        float                               isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        Vector3                             rayPos,         // Starting point of the ray
        Vector3                             rayDir,         // Direction of the ray (does not need to be normalized)
        McmFlag                             flags,          // Mesh generation flags
        out Vector3                         pIntersect);    // The point of intersection if an intersection occurred

    // Intersect a scalar field of floats with a segment (gives the same results as mesh-segment intersection except faster, and the mesh does not need to be generated)
    // If an intersection occurs, then mcmMeshIntersectSegment returns MCM_SUCCESS, and the point of intersection is set,
    // otherwise, mcmMeshIntersectSegment returns MCM_FAILURE
    [DllImport("libmcmesher", EntryPoint = "mcmMeshIntersectSegment", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult   McmMeshIntersectSegment(
        float[]                       data,                 // 3D field of scalar floating-point values as a contiguous array
        Vector3u                      dataSize,             // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        float                         isoLevel,             // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        Vector3                       segPos,               // Starting point of the segment
        Vector3                       segEnd,               // End point of the segment
        McmFlags                      flags,                // Mesh generation flags
        out Vector3                   pIntersect);          // The point of intersection if an intersection occurred

    // Intersect a scalar field of floats with a segment (gives the same results as mesh-segment intersection except faster, and the mesh does not need to be generated)
    // If an intersection occurs, then mcmMeshIntersectSegment returns MCM_SUCCESS, and the point of intersection is set,
    // otherwise, mcmMeshIntersectSegment returns MCM_FAILURE
    [DllImport("libmcmesher", EntryPoint = "mcmMeshIntersectSegment", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult   McmMeshIntersectSegment(
        float[,,]                     data,                 // 3D field of scalar floating-point values as a contiguous array
        Vector3u                      dataSize,             // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        float                         isoLevel,             // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        Vector3                       segPos,               // Starting point of the segment
        Vector3                       segEnd,               // End point of the segment
        McmFlags                      flags,                // Mesh generation flags
        out Vector3                   pIntersect);          // The point of intersection if an intersection occurred

    // Count the number of vertices in the mesh
    [DllImport("libmcmesher", EntryPoint = "mcmCountVertices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint        McmCountVertices(
        IntPtr                              meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Copies vertices into a buffer
    [DllImport("libmcmesher", EntryPoint = "mcmCopyVertices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void McmCopyVertices(
        IntPtr                              meshBuffer,     // Handle to a valid mcmMeshBuffer object
        void*                               dstBuffer);     // Destination buffer of Vector3<float> (allocated size must be >= mcmCountVertices(meshBuffer))

    // Count the number of normals in the mesh
    [DllImport("libmcmesher", EntryPoint = "mcmCountNormals", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint        McmCountNormals(
        IntPtr                              meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Copies normals into a buffer
    [DllImport("libmcmesher", EntryPoint = "mcmCopyNormals", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void McmCopyNormals(
        IntPtr                              meshBuffer,     // Handle to a valid mcmMeshBuffer object
        void*                               dstBuffer);     // Destination buffer of Vector3<float> (allocated size must be >= mcmCountNormals(meshBuffer))

    // Count the number of indices in the mesh
    [DllImport("libmcmesher", EntryPoint = "mcmCountIndices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint        McmCountIndices(
        IntPtr                              meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Copies indices into a buffer
    [DllImport("libmcmesher", EntryPoint = "mcmCopyIndices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void McmCopyIndices(
        IntPtr                              meshBuffer,     // Handle to a valid mcmMeshBuffer object
        void*                               dstBuffer);     // Destination buffer of unsigned 32-bit integers (allocated size must be >= mcmCountIndices(meshBuffer))
}
