// MCmesher
// Kyle J Burgess

#include "MCmesher.h"

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

    auto result = mcmGenerateMeshFN(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f);

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

    result = mcmGenerateMeshVN(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f);

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

    result = mcmGenerateMeshFN(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f);

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

    result = mcmGenerateMeshVN(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f);

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
