// MCmesher
// Kyle J Burgess

#include "MCmesher.h"

#include <functional>
#include <vector>

using GenerateMeshFn = std::function<GenerateMeshResult(const Vector3<uint32_t>&, const Vector3<uint32_t>&)>;

int TestBoundaries(Mesh* mesh, const Vector3<uint32_t>& dataSize, GenerateMeshFn fn)
{
    // Test: entire mesh successfully generates

    Vector3<uint32_t> meshOrigin =
        {
            .x = 0,
            .y = 0,
            .z = 0,
        };

    Vector3<uint32_t> meshSize =
        {
            .x = dataSize.x - 1,
            .y = dataSize.y - 1,
            .z = dataSize.z - 1,
        };

    auto result = fn(meshOrigin, meshSize);

    if (result != GenerateMeshResult::SUCCESS)
    {
        return -1;
    }

    // Test: Mesh size causes x to go out of x boundary

    meshSize.x = dataSize.x;

    result = fn(meshOrigin, meshSize);

    if (result != GenerateMeshResult::ERROR_OUT_OF_BOUNDS_X)
    {
        return -1;
    }

    meshSize.x = dataSize.x - 1;

    // Test: Mesh size causes y to go out of y boundary

    meshSize.y = dataSize.y;

    result = fn(meshOrigin, meshSize);

    if (result != GenerateMeshResult::ERROR_OUT_OF_BOUNDS_Y)
    {
        return -1;
    }

    meshSize.y = dataSize.y - 1;

    // Test: Mesh size causes z to go out of z boundary

    meshSize.z = dataSize.z;

    result = fn(meshOrigin, meshSize);

    if (result != GenerateMeshResult::ERROR_OUT_OF_BOUNDS_Z)
    {
        return -1;
    }

    meshSize.z = dataSize.z - 1;

    // Test: Mesh origin causes x to go out of x boundary

    meshOrigin.x = 1;

    result = fn(meshOrigin, meshSize);

    if (result != GenerateMeshResult::ERROR_OUT_OF_BOUNDS_X)
    {
        return -1;
    }

    meshOrigin.x = 0;

    // Test: Mesh size causes y to go out of y boundary

    meshOrigin.y = 1;

    result = fn(meshOrigin, meshSize);

    if (result != GenerateMeshResult::ERROR_OUT_OF_BOUNDS_Y)
    {
        return -1;
    }

    meshOrigin.y = 0;

    // Test: Mesh size causes z to go out of z boundary

    meshOrigin.z = 1;

    result = fn(meshOrigin, meshSize);

    if (result != GenerateMeshResult::ERROR_OUT_OF_BOUNDS_Z)
    {
        return -1;
    }

    meshOrigin.z = 0;

    // Test: Mesh of 0 size (x)

    meshSize.x = 0;

    result = fn(meshOrigin, meshSize);

    if (!(
        (result == GenerateMeshResult::SUCCESS) &&
        (CountVertices(mesh) == 0) &&
        (CountIndices(mesh) == 0) &&
        (CountNormals(mesh) == 0)))
    {
        return -1;
    }

    meshSize.x = dataSize.x - 1;

    // Test: Mesh of 0 size (y)

    meshSize.y = 0;

    result = fn(meshOrigin, meshSize);

    if (!(
        (result == GenerateMeshResult::SUCCESS) &&
            (CountVertices(mesh) == 0) &&
            (CountIndices(mesh) == 0) &&
            (CountNormals(mesh) == 0)))
    {
        return -1;
    }

    meshSize.y = dataSize.y - 1;

    // Test: Mesh of 0 size (z)

    meshSize.z = 0;

    result = fn(meshOrigin, meshSize);

    if (!(
        (result == GenerateMeshResult::SUCCESS) &&
            (CountVertices(mesh) == 0) &&
            (CountIndices(mesh) == 0) &&
            (CountNormals(mesh) == 0)))
    {
        return -1;
    }

    meshSize.z = dataSize.z - 1;

    // Test passes

    return 0;
}

int main()
{
    // Create Mesh

    auto mesh = CreateMesh();

    // Create scalar field

    const Vector3<uint32_t> dataSize =
        {
            .x = 16,
            .y = 16,
            .z = 16,
        };

    std::vector<float> scalarField(dataSize.x * dataSize.y * dataSize.z);

    for (uint32_t z = 0; z != dataSize.z; ++z)
    {
        for (uint32_t y = 0; y != dataSize.y; ++y)
        {
            for (uint32_t x = 0; x != dataSize.x; ++x)
            {
                size_t i = x + dataSize.x * (y + dataSize.y * z);

                scalarField[i] = (x < (dataSize.x/2))
                    ? 0.0f
                    : 1.0f;
            }
        }
    }

    // Test Generating Face Normal Mesh

    auto result = TestBoundaries(mesh, dataSize, [&](const Vector3<uint32_t>& meshOrigin, const Vector3<uint32_t>& meshSize)
    {
        return GenerateMeshFN(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f);
    });

    if (result != 0)
    {
        return result;
    }

    // Test Generating Vertex Normal Mesh

    result = TestBoundaries(mesh, dataSize, [&](const Vector3<uint32_t>& meshOrigin, const Vector3<uint32_t>& meshSize)
    {
        return GenerateMeshVN(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f);
    });

    if (result != 0)
    {
        return result;
    }

    // Clean up mesh

    DeleteMesh(mesh);

    // Test passes

	return 0;
}
