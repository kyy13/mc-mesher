// mc-mesher TEST file
// Kyle J Burgess

#include "mc_mesher.h"
#include "gradient_noise.h"

#include <ctime>
#include <cmath>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <unordered_map>

float GetValue(size_t x, size_t y, size_t z)
{
    static gnd::gradient_noise<float, 3> noise3d(532346);
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
            static_cast<float>(x) * 0.05f,
            static_cast<float>(y) * 0.05f,
            static_cast<float>(z) * 0.05f,
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

    auto mesh = mcmCreateMeshBuffer();

    if (mcmGenerateMesh(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, 0.0f, MCM_VERTEX_NORMALS) != MCM_SUCCESS)
    {
        return -1;
    }

    auto vertexCount = mcmCountVertices(mesh);
    auto vertexBuffer = mcmGetVertices(mesh);

    constexpr float epsilon = 1e-7f;

    for (uint32_t i = 0; i != vertexCount; ++i)
    {
        for (uint32_t j = i + 1; j != vertexCount; ++j)
        {
            const auto& a = vertexBuffer[i];
            const auto& b = vertexBuffer[j];

            if (fabsf(a.x - b.x) < epsilon &&
                fabsf(a.y - b.y) < epsilon &&
                fabsf(a.z - b.z) < epsilon)
            {
                return -1;
            }
        }
    }

    mcmDeleteMeshBuffer(mesh);

    return 0;
}
