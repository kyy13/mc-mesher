// MCmesher
// Kyle J Burgess

#include "MCmesher.h"

#include <functional>
#include <vector>

using GenerateMeshFn = std::function<GenerateMeshResult(const Vector3<uint32_t>&, const Vector3<uint32_t>&)>;

int TestBoundaries(const Vector3<uint32_t>& dataSize, GenerateMeshFn fn)
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

    const std::vector<float> scalarField(dataSize.x * dataSize.y * dataSize.z, 0.0f);

    // Test Generating Face Normal Mesh

    auto result = TestBoundaries(dataSize, [&](const Vector3<uint32_t>& meshOrigin, const Vector3<uint32_t>& meshSize)
    {
        return GenerateMeshFN(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.5f);
    });

    if (result != 0)
    {
        return result;
    }

    // Test Generating Vertex Normal Mesh

    result = TestBoundaries(dataSize, [&](const Vector3<uint32_t>& meshOrigin, const Vector3<uint32_t>& meshSize)
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
