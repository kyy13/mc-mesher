// MCmesher TEST file
// Kyle J Burgess

#include "MCmesher.h"
#include "gradient_noise.h"

#include <cstring>
#include <vector>
#include <unordered_map>

float GetValue(size_t x, size_t y, size_t z)
{
    static gnd::gradient_noise<float, 3> noise3d(5326);
    static std::unordered_map<uint32_t, float> values;

    uint32_t key = (x % 8) + 8 * (y % 8) + 64 * (z % 8);

    auto it = values.find(key);

    if (it != values.end())
    {
        return it->second;
    }
    else
    {
        float val = noise3d({
            static_cast<float>(x) * 0.1f,
            static_cast<float>(y) * 0.1f,
            static_cast<float>(z) * 0.1f,
        });

        values[key] = val;

        return val;
    }
}

int main()
{
    const Vector3<uint32_t> dataSize =
        {
            .x = 32,
            .y = 32,
            .z = 32,
        };

    const Vector3<uint32_t> meshOrigin =
        {
            .x = 2,
            .y = 2,
            .z = 2,
        };

    const Vector3<uint32_t> meshSize =
        {
            .x = dataSize.x - 5,
            .y = dataSize.y - 5,
            .z = dataSize.z - 5,
        };

    std::vector<float> scalarField(dataSize.x * dataSize.y * dataSize.z);

    for (size_t z = 0; z != dataSize.z; ++z)
    {
        for (size_t y = 0; y != dataSize.y; ++y)
        {
            for (size_t x = 0; x != dataSize.x; ++x)
            {
                size_t i = x + dataSize.x * (y + dataSize.y * z);

                scalarField[i] = GetValue(x, y, z);
            }
        }
    }

    auto meshA = mcmCreateMeshBuffer();
    auto meshB = mcmCreateMeshBuffer();

    auto result = mcmGenerateMeshFN(meshA, scalarField.data(), dataSize, meshOrigin, meshSize, 0.0f);

    if (result != McmResult::MCM_SUCCESS)
    {
        return -1;
    }

    result = mcmGenerateMeshVN(meshB, scalarField.data(), dataSize, meshOrigin, meshSize, 0.0f);

    if (result != McmResult::MCM_SUCCESS)
    {
        return -1;
    }

    size_t numIndices = mcmCountIndices(meshA);

    if (numIndices != mcmCountIndices(meshB))
    {
        return -1;
    }

    if (numIndices == 0)
    {
        return -1;
    }

    const auto* indicesA = mcmGetIndices(meshA);
    const auto* indicesB = mcmGetIndices(meshB);

    const auto* verticesA = mcmGetVertices(meshA);
    const auto* verticesB = mcmGetVertices(meshB);

    const auto vertexCountA = mcmCountVertices(meshA);
    const auto vertexCountB = mcmCountVertices(meshB);

    for (size_t i = 0; i != numIndices; ++i)
    {
        auto indexA = indicesA[i];

        if (indexA >= vertexCountA)
        {
            return -1;
        }

        auto indexB = indicesB[i];

        if (indexB >= vertexCountB)
        {
            return -1;
        }

        constexpr float epsilon = 0.00001f;

        auto dx = fabsf(verticesA[indexA].x - verticesB[indexB].x);
        auto dy = fabsf(verticesA[indexA].y - verticesB[indexB].y);
        auto dz = fabsf(verticesA[indexA].z - verticesB[indexB].z);

        if (!(dx < epsilon && dy < epsilon && dz < epsilon))
        {
            return -1;
        }
    }

    mcmDeleteMeshBuffer(meshA);
    mcmDeleteMeshBuffer(meshB);

    return 0;
}
