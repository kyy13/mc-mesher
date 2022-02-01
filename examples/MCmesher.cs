// MCmesher
// Kyle J Burgess

using System;
using System.Runtime.InteropServices;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Rendering;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using System.Diagnostics;

public class MCmesher
{
    [StructLayout(LayoutKind.Explicit)]
    public struct Vector3u
    {
        [FieldOffset(0)] public uint x;
        [FieldOffset(4)] public uint y;
        [FieldOffset(8)] public uint z;
    }

    protected enum GenerateMeshResult
        {
            SUCCESS = 0,
            ERROR_MESH_IS_NULL = 1,
            ERROR_OUT_OF_BOUNDS_X = 2,
            ERROR_OUT_OF_BOUNDS_Y = 3,
            ERROR_OUT_OF_BOUNDS_Z = 4,
        };

    // Allocate intermediate mesh data (once for every mesh that's generated here)
    public void AllocateGeneratorMemory()
    {
        m_meshHandle = API_CreateMesh();
    }

    // Deallocate intermediate mesh data (once for every mesh that's generated here)
    public void FreeGeneratorMemory()
    {
        API_DeleteMesh(m_meshHandle);
    }

    // Generate a procedural mesh
    // dataSize    size of scalar field data (points)
    // meshOrigin  mesh origin in scalar field (cubes)
    // meshSize    mesh size in scalar field (cubes)
    public void GenerateMesh(
        MeshFilter meshFilter,
        float[,,] voxelData,
        Vector3u dataSize,
        Vector3u meshOrigin,
        Vector3u meshSize,
        float isoLevel = 0.5f,
        bool vertexNormals = true)
    {
        GenerateMeshResult r;

        if (vertexNormals)
        {
            r = API_GenerateMeshVN(m_meshHandle, voxelData, dataSize, meshOrigin, meshSize, isoLevel);
        }
        else
        {
            r = API_GenerateMeshFN(m_meshHandle, voxelData, dataSize, meshOrigin, meshSize, isoLevel);
        }

        if (r != GenerateMeshResult.SUCCESS)
        {
            switch(r)
            {
            case GenerateMeshResult.ERROR_MESH_IS_NULL:
                throw new NullReferenceException();
            case GenerateMeshResult.ERROR_OUT_OF_BOUNDS_X:
            case GenerateMeshResult.ERROR_OUT_OF_BOUNDS_Y:
            case GenerateMeshResult.ERROR_OUT_OF_BOUNDS_Z:
                throw new ArgumentOutOfRangeException();
            default:
                throw new Exception();
            };
        }

        ApplyMeshToMeshFilter(meshFilter);
    }

    // Generate a procedural mesh
    // dataSize    size of scalar field data (points)
    // meshOrigin  mesh origin in scalar field (cubes)
    // meshSize    mesh size in scalar field (cubes)
    public void GenerateMesh(
        MeshFilter meshFilter,
        float[] voxelData,
        Vector3u dataSize,
        Vector3u meshOrigin,
        Vector3u meshSize,
        float isoLevel = 0.5f,
        bool vertexNormals = true)
    {
        GenerateMeshResult r;

        if (vertexNormals)
        {
            r = API_GenerateMeshVN(m_meshHandle, voxelData, dataSize, meshOrigin, meshSize, isoLevel);
        }
        else
        {
            r = API_GenerateMeshFN(m_meshHandle, voxelData, dataSize, meshOrigin, meshSize, isoLevel);
        }

        if (r != GenerateMeshResult.SUCCESS)
        {
            switch(r)
            {
            case GenerateMeshResult.ERROR_MESH_IS_NULL:
                throw new NullReferenceException();
            case GenerateMeshResult.ERROR_OUT_OF_BOUNDS_X:
            case GenerateMeshResult.ERROR_OUT_OF_BOUNDS_Y:
            case GenerateMeshResult.ERROR_OUT_OF_BOUNDS_Z:
                throw new ArgumentOutOfRangeException();
            default:
                throw new Exception();
            };
        }

        ApplyMeshToMeshFilter(meshFilter);
    }

    protected void ApplyMeshToMeshFilter(MeshFilter meshFilter)
    {
        var dataArray = Mesh.AllocateWritableMeshData(1);
        var data = dataArray[0];

        int vertexCount = (int)API_CountVertices(m_meshHandle);
        int indexCount = (int)API_CountIndices(m_meshHandle);

        data.SetVertexBufferParams(vertexCount,
            new VertexAttributeDescriptor(VertexAttribute.Position),
            new VertexAttributeDescriptor(VertexAttribute.Normal, stream: 1));

        var vertices = data.GetVertexData<Vector3>();

        unsafe
        {
            API_CopyVertices(m_meshHandle, NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(vertices));
        }

        var normals = data.GetVertexData<Vector3>(1);

        unsafe
        {
            API_CopyNormals(m_meshHandle, NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(normals));
        }

        data.SetIndexBufferParams(indexCount, IndexFormat.UInt32);

        var indices = data.GetIndexData<uint>();

        unsafe
        {
            API_CopyIndices(m_meshHandle, NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(indices));
        }

        data.subMeshCount = 1;
        data.SetSubMesh(0, new SubMeshDescriptor(0, indexCount));

        var mesh = new Mesh();

        Mesh.ApplyAndDisposeWritableMeshData(dataArray, mesh);

        mesh.RecalculateBounds();

        meshFilter.mesh = mesh;
    }

    protected IntPtr m_meshHandle;



    [DllImport("libMCmesher", EntryPoint = "CreateMesh", CallingConvention = CallingConvention.Cdecl)]
    protected static extern IntPtr API_CreateMesh();

    [DllImport("libMCmesher", EntryPoint = "DeleteMesh", CallingConvention = CallingConvention.Cdecl)]
    protected static extern void API_DeleteMesh(IntPtr meshHandle);

    [DllImport("libMCmesher", EntryPoint = "GenerateMeshVN", CallingConvention = CallingConvention.Cdecl)]
    protected static extern GenerateMeshResult API_GenerateMeshVN(
        IntPtr meshHandle,
        float[,,] data,
        Vector3u dataSize,        // size of scalar field data (points)
        Vector3u meshOrigin,      // mesh origin in scalar field (cubes)
        Vector3u meshSize,        // mesh size in scalar field (cubes)
        float isoLevel);

    [DllImport("libMCmesher", EntryPoint = "GenerateMeshVN", CallingConvention = CallingConvention.Cdecl)]
    protected static extern GenerateMeshResult API_GenerateMeshVN(IntPtr meshHandle,
        float[] data,
        Vector3u dataSize,        // size of scalar field data (points)
        Vector3u meshOrigin,      // mesh origin in scalar field (cubes)
        Vector3u meshSize,        // mesh size in scalar field (cubes)
        float isoLevel);

    [DllImport("libMCmesher", EntryPoint = "GenerateMeshFN", CallingConvention = CallingConvention.Cdecl)]
    protected static extern GenerateMeshResult API_GenerateMeshFN(
        IntPtr meshHandle,
        float[,,] data,
        Vector3u dataSize,        // size of scalar field data (points)
        Vector3u meshOrigin,      // mesh origin in scalar field (cubes)
        Vector3u meshSize,        // mesh size in scalar field (cubes)
        float isoLevel);

    [DllImport("libMCmesher", EntryPoint = "GenerateMeshFN", CallingConvention = CallingConvention.Cdecl)]
    protected static extern GenerateMeshResult API_GenerateMeshFN(IntPtr meshHandle,
        float[] data,
        Vector3u dataSize,        // size of scalar field data (points)
        Vector3u meshOrigin,      // mesh origin in scalar field (cubes)
        Vector3u meshSize,        // mesh size in scalar field (cubes)
        float isoLevel);

    [DllImport("libMCmesher", EntryPoint = "CountVertices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint API_CountVertices(IntPtr meshHandle);

    [DllImport("libMCmesher", EntryPoint = "CopyVertices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void API_CopyVertices(IntPtr meshHandle, void* dst);

    [DllImport("libMCmesher", EntryPoint = "CountNormals", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint API_CountNormals(IntPtr meshHandle);

    [DllImport("libMCmesher", EntryPoint = "CopyNormals", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void API_CopyNormals(IntPtr meshHandle, void* dst);

    [DllImport("libMCmesher", EntryPoint = "CountIndices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint API_CountIndices(IntPtr meshHandle);

    [DllImport("libMCmesher", EntryPoint = "CopyIndices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void API_CopyIndices(IntPtr meshHandle, void* dst);
}
