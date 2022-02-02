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
    // Allocate intermediate mesh data (once for every mesh that's generated here)
    public void AllocateGeneratorMemory()
    {
        m_meshBuffer = McmCreateMeshBuffer();
    }

    // Deallocate intermediate mesh data (once for every mesh that's generated here)
    public void FreeGeneratorMemory()
    {
        McmDeleteMeshBuffer(m_meshBuffer);
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
        McmResult r;

        if (vertexNormals)
        {
            r = McmGenerateMeshVN(m_meshBuffer, voxelData, dataSize, meshOrigin, meshSize, isoLevel);
        }
        else
        {
            r = McmGenerateMeshFN(m_meshBuffer, voxelData, dataSize, meshOrigin, meshSize, isoLevel);
        }

        if (r != McmResult.MCM_SUCCESS)
        {
            switch(r)
            {
            case McmResult.MCM_MESH_BUFFER_IS_NULL:
                throw new NullReferenceException();
            case McmResult.MCM_OUT_OF_BOUNDS_X:
            case McmResult.MCM_OUT_OF_BOUNDS_Y:
            case McmResult.MCM_OUT_OF_BOUNDS_Z:
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
        McmResult r;

        if (vertexNormals)
        {
            r = McmGenerateMeshVN(m_meshBuffer, voxelData, dataSize, meshOrigin, meshSize, isoLevel);
        }
        else
        {
            r = McmGenerateMeshFN(m_meshBuffer, voxelData, dataSize, meshOrigin, meshSize, isoLevel);
        }

        if (r != McmResult.MCM_SUCCESS)
        {
            switch(r)
            {
            case McmResult.MCM_MESH_BUFFER_IS_NULL:
                throw new NullReferenceException();
            case McmResult.MCM_OUT_OF_BOUNDS_X:
            case McmResult.MCM_OUT_OF_BOUNDS_Y:
            case McmResult.MCM_OUT_OF_BOUNDS_Z:
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

        data.subMeshCount = 1;
        data.SetSubMesh(0, new SubMeshDescriptor(0, indexCount));

        var mesh = new Mesh();

        Mesh.ApplyAndDisposeWritableMeshData(dataArray, mesh);

        mesh.RecalculateBounds();

        meshFilter.mesh = mesh;
    }

    protected IntPtr m_meshBuffer;

    // -------------------------------------------
    // MCmesher C++ Library Methods and Structs
    // -------------------------------------------

    // Vector3 of uint
    [StructLayout(LayoutKind.Explicit)]
    public struct Vector3u
    {
        [FieldOffset(0)] public uint x;
        [FieldOffset(4)] public uint y;
        [FieldOffset(8)] public uint z;
    }

    // Result of MCmesher functions
    protected enum McmResult
    {
        MCM_SUCCESS                         = 0,            // Function was successful
        MCM_MESH_BUFFER_IS_NULL             = 1,            // Error, the mesh buffer passed to the function was NULL
        MCM_OUT_OF_BOUNDS_X                 = 2,            // Error, an argument passed to the function was out of bounds in the x-axis
        MCM_OUT_OF_BOUNDS_Y                 = 3,            // Error, an argument passed to the function was out of bounds in the y-axis
        MCM_OUT_OF_BOUNDS_Z                 = 4,            // Error, an argument passed to the function was out of bounds in the z-axis
    };

    // Create an McmMeshBuffer and return its handle (pointer)
    [DllImport("libMCmesher", EntryPoint = "mcmCreateMeshBuffer", CallingConvention = CallingConvention.Cdecl)]
    protected static extern IntPtr      McmCreateMeshBuffer();

    // Delete an McmMeshBuffer, invalidating the handle
    [DllImport("libMCmesher", EntryPoint = "mcmDeleteMeshBuffer", CallingConvention = CallingConvention.Cdecl)]
    protected static extern void        McmDeleteMeshBuffer(
        IntPtr                              meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Generate a marching cubes mesh with vertex normals, and store the results in an McmMeshBuffer
    [DllImport("libMCmesher", EntryPoint = "mcmGenerateMeshVN", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult   McmGenerateMeshVN(
        IntPtr                              meshBuffer,     // Handle to a valid mcmMeshBuffer object
        float[]                             data,           // 3D field of scalar floating-point values as a contiguous array
        Vector3u                            dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        Vector3u                            meshOrigin,     // Origin of the mesh to generate (in cubes) within the 3D field
        Vector3u                            meshSize,       // Size of the mesh to generate (in cubes) within the 3D field
        float                               isoLevel);      // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)

    // Generate a marching cubes mesh with vertex normals, and store the results in an McmMeshBuffer
    [DllImport("libMCmesher", EntryPoint = "mcmGenerateMeshVN", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult   McmGenerateMeshVN(
        IntPtr                              meshBuffer,     // Handle to a valid mcmMeshBuffer object
        float[,,]                           data,           // 3D field of scalar floating-point values as a contiguous array
        Vector3u                            dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        Vector3u                            meshOrigin,     // Origin of the mesh to generate (in cubes) within the 3D field
        Vector3u                            meshSize,       // Size of the mesh to generate (in cubes) within the 3D field
        float                               isoLevel);      // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)

    // Generate a marching cubes mesh with face normals, and store the results in an McmMeshBuffer
    [DllImport("libMCmesher", EntryPoint = "mcmGenerateMeshFN", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult   McmGenerateMeshFN(
        IntPtr                              meshBuffer,     // Handle to a valid mcmMeshBuffer object
        float[]                             data,           // 3D field of scalar floating-point values as a contiguous array
        Vector3u                            dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        Vector3u                            meshOrigin,     // Origin of the mesh to generate (in cubes) within the 3D field
        Vector3u                            meshSize,       // Size of the mesh to generate (in cubes) within the 3D field
        float                               isoLevel);      // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)

    // Generate a marching cubes mesh with vertex normals, and store the results in an McmMeshBuffer
    [DllImport("libMCmesher", EntryPoint = "mcmGenerateMeshFN", CallingConvention = CallingConvention.Cdecl)]
    protected static extern McmResult   McmGenerateMeshFN(
        IntPtr                              meshBuffer,     // Handle to a valid mcmMeshBuffer object
        float[,,]                           data,           // 3D field of scalar floating-point values as a contiguous array
        Vector3u                            dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        Vector3u                            meshOrigin,     // Origin of the mesh to generate (in cubes) within the 3D field
        Vector3u                            meshSize,       // Size of the mesh to generate (in cubes) within the 3D field
        float                               isoLevel);      // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)

    // Count the number of vertices in the mesh
    [DllImport("libMCmesher", EntryPoint = "mcmCountVertices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint        McmCountVertices(
        IntPtr                              meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Copies vertices into a buffer
    [DllImport("libMCmesher", EntryPoint = "mcmCopyVertices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void McmCopyVertices(
        IntPtr                              meshBuffer,     // Handle to a valid mcmMeshBuffer object
        void*                               dstBuffer);     // Destination buffer of Vector3<float> (allocated size must be >= mcmCountVertices(meshBuffer))

    // Count the number of normals in the mesh
    [DllImport("libMCmesher", EntryPoint = "mcmCountNormals", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint        McmCountNormals(
        IntPtr                              meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Copies normals into a buffer
    [DllImport("libMCmesher", EntryPoint = "mcmCopyNormals", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void McmCopyNormals(
        IntPtr                              meshBuffer,     // Handle to a valid mcmMeshBuffer object
        void*                               dstBuffer);     // Destination buffer of Vector3<float> (allocated size must be >= mcmCountNormals(meshBuffer))

    // Count the number of indices in the mesh
    [DllImport("libMCmesher", EntryPoint = "mcmCountIndices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern uint        McmCountIndices(
        IntPtr                              meshBuffer);    // Handle to a valid mcmMeshBuffer object

    // Copies indices into a buffer
    [DllImport("libMCmesher", EntryPoint = "mcmCopyIndices", CallingConvention = CallingConvention.Cdecl)]
    protected static extern unsafe void McmCopyIndices(
        IntPtr                              meshBuffer,     // Handle to a valid mcmMeshBuffer object
        void*                               dstBuffer);     // Destination buffer of unsigned 32-bit integers (allocated size must be >= mcmCountIndices(meshBuffer))
}
