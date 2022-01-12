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

public class MarchingCubeMeshGenerator
{
    [StructLayout(LayoutKind.Explicit)]
    public struct ScalarField
    {
        [FieldOffset(0)] public uint width;
        [FieldOffset(4)] public uint height;
        [FieldOffset(8)] public uint depth;
    }

    [StructLayout(LayoutKind.Explicit)]
    public struct CubeSlice
    {
        [FieldOffset(0)] public uint x0;
        [FieldOffset(4)] public uint y0;
        [FieldOffset(8)] public uint z0;
        [FieldOffset(12)] public uint width;
        [FieldOffset(16)] public uint height;
        [FieldOffset(20)] public uint depth;
    }

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
    public void GenerateMesh(MeshFilter meshFilter, float[,,] voxelData, ref ScalarField field, ref CubeSlice slice)
    {
        API_GenerateMesh(m_meshHandle, voxelData, ref field, ref slice, 0.5f, false, true);

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
            API_CopyVertexNormals(m_meshHandle, NativeArrayUnsafeUtility.GetUnsafeBufferPointerWithoutChecks(normals));
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

    [DllImport("libMCmesher", EntryPoint = "GenerateMesh", CallingConvention = CallingConvention.Cdecl)]
    protected static extern void API_GenerateMesh(IntPtr meshHandle, float[,,] data, ref ScalarField field, ref CubeSlice slice, float isoLevel, bool computeFaceNormals, bool computeVertexNormals);

    [DllImport("libMCmesher", EntryPoint = "CountVertices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint API_CountVertices(IntPtr meshHandle);

    [DllImport("libMCmesher", EntryPoint = "CopyVertices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void API_CopyVertices(IntPtr meshHandle, void* dst);

    [DllImport("libMCmesher", EntryPoint = "CountVertexNormals", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint API_CountVertexNormals(IntPtr meshHandle);

    [DllImport("libMCmesher", EntryPoint = "CopyVertexNormals", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void API_CopyVertexNormals(IntPtr meshHandle, void* dst);

    [DllImport("libMCmesher", EntryPoint = "CountIndices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint API_CountIndices(IntPtr meshHandle);

    [DllImport("libMCmesher", EntryPoint = "CopyIndices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void API_CopyIndices(IntPtr meshHandle, void* dst);
}
