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
        McmResult r = McmGenerateMesh(
            m_meshBuffer,
            voxelData,
            ref dataSize,
            ref meshOrigin,
            ref meshSize,
            isoLevel,
            flags);

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
        McmResult r = McmGenerateMesh(
            m_meshBuffer,
            voxelData,
            ref dataSize,
            ref meshOrigin,
            ref meshSize,
            isoLevel,
            flags);

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
        return McmMeshContainsPoint(data, ref dataSize, isoLevel, ref point, flags) == McmResult.MCM_SUCCESS;
    }

    public static bool MeshContainsPoint(float[,,] data, Vector3u dataSize, float isoLevel, Vector3 point, McmFlag flags)
    {
        return McmMeshContainsPoint(data, ref dataSize, isoLevel, ref point, flags) == McmResult.MCM_SUCCESS;
    }

    // Intersect a scalar field of floats with a ray (gives the same results as mesh-ray intersection except faster, and the mesh does not need to be generated)
    // If an intersection occurs, then returns the point of intersection, otherwise returns null
    public static Vector3? MeshIntersectRay(float[] data, Vector3u dataSize, float isoLevel, Vector3 rayPos, Vector3 rayDir, McmFlag flags)
    {
        Vector3 pIntersect = new Vector3();

        if (McmMeshIntersectRay(data, ref dataSize, isoLevel, ref rayPos, ref rayDir, flags, ref pIntersect) == McmResult.MCM_SUCCESS)
        {
            return pIntersect;
        }

        return null;
    }

    public static Vector3? MeshIntersectRay(float[,,] data, Vector3u dataSize, float isoLevel, Vector3 rayPos, Vector3 rayDir, McmFlag flags)
    {
        Vector3 pIntersect = new Vector3();

        if (McmMeshIntersectRay(data, ref dataSize, isoLevel, ref rayPos, ref rayDir, flags, ref pIntersect) == McmResult.MCM_SUCCESS)
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
        Vector3 pIntersect = new Vector3();

        if (McmMeshIntersectSegment(data, ref dataSize, isoLevel, ref segPos, ref segEnd, flags, ref pIntersect) == McmResult.MCM_SUCCESS)
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
        [FieldOffset(0)] public UInt32 x;
        [FieldOffset(4)] public UInt32 y;
        [FieldOffset(8)] public UInt32 z;
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

    [DllImport("libmcmesher", EntryPoint = "mcmCreateMeshBuffer", CallingConvention = CallingConvention.Cdecl)]
    protected static extern IntPtr McmCreateMeshBuffer();

    [DllImport("libmcmesher", EntryPoint = "mcmDeleteMeshBuffer", CallingConvention = CallingConvention.Cdecl)]
    protected static extern void McmDeleteMeshBuffer(IntPtr meshBuffer);

    [DllImport("libmcmesher", EntryPoint = "mcmGenerateMesh", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult McmGenerateMesh(IntPtr meshBuffer, float[] data, ref Vector3u dataSize, ref Vector3u meshOrigin, ref Vector3u meshSize, float isoLevel, McmFlag flags);

    [DllImport("libmcmesher", EntryPoint = "mcmGenerateMesh", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult McmGenerateMesh(IntPtr meshBuffer, float[,,] data, ref Vector3u dataSize, ref Vector3u meshOrigin, ref Vector3u meshSize, float isoLevel, McmFlag flags);

    [DllImport("libmcmesher", EntryPoint = "mcmMeshContainsPoint", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult McmMeshContainsPoint(float[] data, ref Vector3u dataSize, float isoLevel, ref Vector3 point, McmFlag flags); 

    [DllImport("libmcmesher", EntryPoint = "mcmMeshContainsPoint", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult McmMeshContainsPoint(float[,,] data, ref Vector3u dataSize, float isoLevel, ref Vector3 point, McmFlag flags);

    [DllImport("libmcmesher", EntryPoint = "mcmMeshIntersectRay", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult McmMeshIntersectRay(float[] data, ref Vector3u dataSize, float isoLevel, ref Vector3 rayPos, ref Vector3 rayDir, McmFlag flags, ref Vector3 pIntersect);

    [DllImport("libmcmesher", EntryPoint = "mcmMeshIntersectRay", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult McmMeshIntersectRay(float[,,] data, ref Vector3u dataSize, float isoLevel, ref Vector3 rayPos, ref Vector3 rayDir, McmFlag flags, ref Vector3 pIntersect);

    [DllImport("libmcmesher", EntryPoint = "mcmMeshIntersectSegment", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult McmMeshIntersectSegment(float[] data, ref Vector3u dataSize, float isoLevel, ref Vector3 segPos, ref Vector3 segEnd, McmFlag flags, ref Vector3 pIntersect);

    [DllImport("libmcmesher", EntryPoint = "mcmMeshIntersectSegment", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult McmMeshIntersectSegment(float[,,] data, ref Vector3u dataSize, float isoLevel, ref Vector3 segPos, ref Vector3 segEnd, McmFlag flags, ref Vector3 pIntersect);

    [DllImport("libmcmesher", EntryPoint = "mcmCountVertices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern UInt32 McmCountVertices(IntPtr meshBuffer);

    [DllImport("libmcmesher", EntryPoint = "mcmCopyVertices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void McmCopyVertices(IntPtr meshBuffer, void* dstBuffer);

    [DllImport("libmcmesher", EntryPoint = "mcmCountNormals", CallingConvention = CallingConvention.Cdecl)]
    protected static extern UInt32 McmCountNormals(IntPtr meshBuffer);

    [DllImport("libmcmesher", EntryPoint = "mcmCopyNormals", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void McmCopyNormals(IntPtr meshBuffer, void* dstBuffer);

    [DllImport("libmcmesher", EntryPoint = "mcmCountIndices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint McmCountIndices(IntPtr meshBuffer);

    [DllImport("libmcmesher", EntryPoint = "mcmCopyIndices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void McmCopyIndices(IntPtr meshBuffer, void* dstBuffer);
}
