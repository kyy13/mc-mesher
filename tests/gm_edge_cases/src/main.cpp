// mc-mesher
// Kyle J Burgess

#include "mc_mesher.h"

#include <vector>

int main()
{
    const uint32_t dataSize[3] =
        {
            16,
            16,
            16,
        };

    const uint32_t meshOrigin[3] =
        {
            0,
            0,
            0,
        };

    const uint32_t meshSize[3] =
        {
            dataSize[0] - 1,
            dataSize[1] - 1,
            dataSize[2] - 1,
        };

    auto mesh = mcmCreateMeshBuffer();

    std::vector<float> scalarField(dataSize[0] * dataSize[1] * dataSize[2]);

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
