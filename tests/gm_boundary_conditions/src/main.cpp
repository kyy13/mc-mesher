// mc-mesher
// Kyle J Burgess

#include "mc_mesher.h"

#include <functional>
#include <vector>

typedef McmResult (*GenerateMesh)(const uint32_t* meshOrigin, const uint32_t* meshSize, McmFlags flags);

int TestBoundaries(McmMeshBuffer* mesh, const uint32_t dataSize[3], McmFlags flags, GenerateMesh fn)
{
    // Test: entire mesh successfully generates

    uint32_t meshOrigin[3] = {0, 0, 0};

    uint32_t meshSize[3] =
        {
            dataSize[0] - 1,
            dataSize[1] - 1,
            dataSize[2] - 1,
        };

    auto result = fn(meshOrigin, meshSize, flags);

    if (result != McmResult::MCM_SUCCESS)
    {
        return -1;
    }

    // Test: Mesh size causes x to go out of x boundary

    meshSize[0] = dataSize[0];

    result = fn(meshOrigin, meshSize, flags);

    if (result != McmResult::MCM_OUT_OF_BOUNDS_X)
    {
        return -1;
    }

    meshSize[0] = dataSize[0] - 1;

    // Test: Mesh size causes y to go out of y boundary

    meshSize[1] = dataSize[1];

    result = fn(meshOrigin, meshSize, flags);

    if (result != McmResult::MCM_OUT_OF_BOUNDS_Y)
    {
        return -1;
    }

    meshSize[1] = dataSize[1] - 1;

    // Test: Mesh size causes z to go out of z boundary

    meshSize[2] = dataSize[2];

    result = fn(meshOrigin, meshSize, flags);

    if (result != McmResult::MCM_OUT_OF_BOUNDS_Z)
    {
        return -1;
    }

    meshSize[2] = dataSize[2] - 1;

    // Test: Mesh origin causes x to go out of x boundary

    meshOrigin[0] = 1;

    result = fn(meshOrigin, meshSize, flags);

    if (result != McmResult::MCM_OUT_OF_BOUNDS_X)
    {
        return -1;
    }

    meshOrigin[0] = 0;

    // Test: Mesh size causes y to go out of y boundary

    meshOrigin[1] = 1;

    result = fn(meshOrigin, meshSize, flags);

    if (result != McmResult::MCM_OUT_OF_BOUNDS_Y)
    {
        return -1;
    }

    meshOrigin[1] = 0;

    // Test: Mesh size causes z to go out of z boundary

    meshOrigin[2] = 1;

    result = fn(meshOrigin, meshSize, flags);

    if (result != McmResult::MCM_OUT_OF_BOUNDS_Z)
    {
        return -1;
    }

    meshOrigin[2] = 0;

    // Test: Mesh of 0 size (x)

    meshSize[0] = 0;

    result = fn(meshOrigin, meshSize, flags);

    if (!(
        (result == McmResult::MCM_SUCCESS) &&
        (mcmCountVertices(mesh) == 0) &&
        (mcmCountIndices(mesh) == 0) &&
        (mcmCountNormals(mesh) == 0)))
    {
        return -1;
    }

    meshSize[0] = dataSize[0] - 1;

    // Test: Mesh of 0 size (y)

    meshSize[1] = 0;

    result = fn(meshOrigin, meshSize, flags);

    if (!(
        (result == McmResult::MCM_SUCCESS) &&
            (mcmCountVertices(mesh) == 0) &&
            (mcmCountIndices(mesh) == 0) &&
            (mcmCountNormals(mesh) == 0)))
    {
        return -1;
    }

    meshSize[1] = dataSize[1] - 1;

    // Test: Mesh of 0 size (z)

    meshSize[2] = 0;

    result = fn(meshOrigin, meshSize, flags);

    if (!(
        (result == McmResult::MCM_SUCCESS) &&
            (mcmCountVertices(mesh) == 0) &&
            (mcmCountIndices(mesh) == 0) &&
            (mcmCountNormals(mesh) == 0)))
    {
        return -1;
    }

    meshSize[2] = dataSize[2] - 1;

    // Test passes

    return 0;
}

McmMeshBuffer* mesh;

const uint32_t dataSize[3] =
    {
        16,
        16,
        16,
    };

std::vector<float> scalarField;

McmResult genMesh(const uint32_t* meshOrigin, const uint32_t* meshSize, McmFlags flags)
{
    return mcmGenerateMesh(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f, flags);
}

int main()
{
    // Create Mesh
    mesh = mcmCreateMeshBuffer();

    // Create scalar field
    const uint32_t dataSize[3] =
        {
            16,
            16,
            16,
        };

    scalarField.resize(dataSize[0] * dataSize[1] * dataSize[2]);

    for (uint32_t z = 0; z != dataSize[2]; ++z)
    {
        for (uint32_t y = 0; y != dataSize[1]; ++y)
        {
            for (uint32_t x = 0; x != dataSize[0]; ++x)
            {
                size_t i = x + dataSize[0] * (y + dataSize[1] * z);

                scalarField[i] = (x < (dataSize[0]/2))
                    ? 0.0f
                    : 1.0f;
            }
        }
    }

    McmFlags fullFlags = MCM_WINDING_RHCS_CCW | MCM_FACE_NORMALS | MCM_EDGE_CENTER;

    for (int flags = 0; flags != fullFlags; ++flags)
    {
        // Test Generating Float meshes
        auto result = TestBoundaries(mesh, dataSize, flags, &genMesh);

        if (result != 0)
        {
            return result;
        }

        // Test NULL mesh buffers
        const uint32_t meshOrigin[3] = {0, 0, 0};

        const uint32_t meshSize[3] =
            {
                dataSize[0] - 1,
                dataSize[1] - 1,
                dataSize[2] - 1,
            };

        if (mcmGenerateMesh(nullptr, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f, flags) != McmResult::MCM_MESH_BUFFER_IS_NULL)
        {
            return -1;
        }
    }

    // Clean up mesh
    mcmDeleteMeshBuffer(mesh);

    // Test passes
    return 0;
}
