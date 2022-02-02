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

    auto mesh = CreateMesh();

    // Mesh, all values until iso level

    std::vector<float> scalarField(dataSize.x * dataSize.y * dataSize.z);

    for (auto& scalar : scalarField)
    {
        scalar = 0.0f;
    }

    auto result = GenerateMeshFN(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f);

    if (result != GenerateMeshResult::SUCCESS)
    {
        return -1;
    }

    if (!(
        (CountVertices(mesh) == 0) &&
        (CountNormals(mesh) == 0) &&
        (CountIndices(mesh) == 0)))
    {
        return -1;
    }

    result = GenerateMeshVN(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f);

    if (result != GenerateMeshResult::SUCCESS)
    {
        return -1;
    }

    if (!(
        (CountVertices(mesh) == 0) &&
        (CountNormals(mesh) == 0) &&
        (CountIndices(mesh) == 0)))
    {
        return -1;
    }

    for (auto& scalar : scalarField)
    {
        scalar = 1.0f;
    }

    result = GenerateMeshFN(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f);

    if (result != GenerateMeshResult::SUCCESS)
    {
        return -1;
    }

    if (!(
        (CountVertices(mesh) == 0) &&
        (CountNormals(mesh) == 0) &&
        (CountIndices(mesh) == 0)))
    {
        return -1;
    }

    result = GenerateMeshVN(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f);

    if (result != GenerateMeshResult::SUCCESS)
    {
        return -1;
    }

    if (!(
        (CountVertices(mesh) == 0) &&
        (CountNormals(mesh) == 0) &&
        (CountIndices(mesh) == 0)))
    {
        return -1;
    }

    DeleteMesh(mesh);

    return 0;
}
