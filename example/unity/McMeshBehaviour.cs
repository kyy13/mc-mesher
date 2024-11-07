// mc-mesher MonoBehaviour example
// Attach this script to a GameObject with a MeshFilter and MeshRenderer component.
// Alternatively, you can use a built-in mesh like a Cube instead, which already has a MeshFilter and configured MeshRender.
// NOTE:
//  -Follow the build/quick-start instructions in the README.md, your Assets folder should have mc_mesher.cs and libmcmesher.dll
//  -Requires Unity Mathematics library to generate 3d noise (Windows > PackageManager > Mathematics)
//  -Requires 'unsafe code' to be enabled in Player settings in order to use mc-mesher c++ interop (Edit > Project Settings > Search for "unsafe")

using UnityEngine;
using Unity.Mathematics;

public class McMeshBehaviour : MonoBehaviour
{
    private void Start()
    {
        var fieldSize = new Vector3Int(128, 128, 128);
        var field = new float[fieldSize.x, fieldSize.y, fieldSize.z];

        // The scale factor determines the frequency of the noise (how quickly the value changes relative to position)
        InitRandomField(field, fieldSize, 0.1f);

        // The tolerance affects which noise values are considered inside/outside the surface
        GenerateMesh(field, fieldSize, 0.3f);
    }

    private static void InitRandomField(float[,,] field, Vector3Int fieldSize, float scale)
    {
        for (uint z = 0; z != fieldSize.z; ++z)
        for (uint y = 0; y != fieldSize.y; ++y)
        for (uint x = 0; x != fieldSize.x; ++x)
        {
            field[x,y,z] = noise.cnoise(new Vector3(x, y, z) * scale);
        }
    }

    private void GenerateMesh(float[,,] field, Vector3Int fieldSize, float tolerance)
    {
        var meshFilter = GetComponent<MeshFilter>();

        // Find the mesh filter
        if (meshFilter == null)
        {
            Debug.Log("GameObject is missing MeshFilter");
            return;
        }

        // Create a new sharedMesh if none already exist
        if (meshFilter.sharedMesh == null)
        {
            meshFilter.sharedMesh = new Mesh();
        }

        var mesh = meshFilter.sharedMesh;

        // This is the size of the provided data
        McmMeshBuffer.Vector3u dataSize;
        dataSize.x = (uint)fieldSize.x;
        dataSize.y = (uint)fieldSize.y;
        dataSize.z = (uint)fieldSize.z;

        // The origin of the mesh
        McmMeshBuffer.Vector3u meshOrigin;
        meshOrigin.x = 0;
        meshOrigin.y = 0;
        meshOrigin.z = 0;

        // The size of the mesh
        // NOTE: when a mesh has N vertices along an axis, that creates (N-1) cubes
        McmMeshBuffer.Vector3u meshSize;
        meshSize.x = (uint)fieldSize.x - 1;
        meshSize.y = (uint)fieldSize.y - 1;
        meshSize.z = (uint)fieldSize.z - 1;

        // Use a mesh buffer to generate the mesh
        // NOTE: this block will create and destroy the mesh buffer, but we can reuse a mesh buffer
        // if we are planning on recreating the mesh often.
        using (McmMeshBuffer meshBuffer = new McmMeshBuffer())
        {
            meshBuffer.GenerateMesh(ref mesh, field, dataSize, meshOrigin, meshSize, tolerance);
        }
    }
}
