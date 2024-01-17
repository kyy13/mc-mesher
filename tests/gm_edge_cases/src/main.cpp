// mc-mesher
// Kyle J Burgess

#include "mc_mesher.h"

#include <vector>

int main()
{
    const Vector3<uint32_t> dataSize =
        {
            .x = 16,
            .y = 16,
            .z = 16,
        };

    const Vector3<uint32_t> meshOrigin =
        {
            .x = 0,
            .y = 0,
            .z = 0,
        };

    const Vector3<uint32_t> meshSize =
        {
            .x = dataSize.x - 1,
            .y = dataSize.y - 1,
            .z = dataSize.z - 1,
        };

    auto mesh = mcmCreateMeshBuffer();

    std::vector<float> scalarField(dataSize.x * dataSize.y * dataSize.z);

    // Test: all values under iso level

    for (auto& scalar : scalarField)
    {
        scalar = 0.0f;
    }

    auto result = mcmGenerateMesh(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f, MCM_FACE_NORMALS);

    if (result != McmResult::MCM_SUCCESS)
    {
        return -1;
    }

    if (!(
        (mcmCountVertices(mesh) == 0) &&
        (mcmCountNormals(mesh) == 0) &&
        (mcmCountIndices(mesh) == 0)))
    {
        return -1;
    }

    result = mcmGenerateMesh(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f, MCM_VERTEX_NORMALS);

    if (result != McmResult::MCM_SUCCESS)
    {
        return -1;
    }

    if (!(
        (mcmCountVertices(mesh) == 0) &&
        (mcmCountNormals(mesh) == 0) &&
        (mcmCountIndices(mesh) == 0)))
    {
        return -1;
    }

    // Test: all values over iso level

    for (auto& scalar : scalarField)
    {
        scalar = 1.0f;
    }

    result = mcmGenerateMesh(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f, MCM_FACE_NORMALS);

    if (result != McmResult::MCM_SUCCESS)
    {
        return -1;
    }

    if (!(
        (mcmCountVertices(mesh) == 0) &&
        (mcmCountNormals(mesh) == 0) &&
        (mcmCountIndices(mesh) == 0)))
    {
        return -1;
    }

    result = mcmGenerateMesh(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f, MCM_VERTEX_NORMALS);

    if (result != McmResult::MCM_SUCCESS)
    {
        return -1;
    }

    if (!(
        (mcmCountVertices(mesh) == 0) &&
        (mcmCountNormals(mesh) == 0) &&
        (mcmCountIndices(mesh) == 0)))
    {
        return -1;
    }

    mcmDeleteMeshBuffer(mesh);

    return 0;
}
