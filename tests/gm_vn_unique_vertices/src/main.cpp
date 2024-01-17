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
    const uint32_t dataSize[3] =
        {
            32,
            32,
            32,
        };

    const uint32_t meshOrigin[3] =
        {
            0,
            0,
            0,
        };

    const uint32_t meshSize[3] =
        {
            dataSize[0]- 1,
            dataSize[1] - 1,
            dataSize[2] - 1,
        };

    std::vector<float> scalarField(dataSize[0] * dataSize[1] * dataSize[2]);

    for (size_t z = 0; z != dataSize[2]; ++z)
    {
        for (size_t y = 0; y != dataSize[1]; ++y)
        {
            for (size_t x = 0; x != dataSize[0]; ++x)
            {
                size_t i = x + dataSize[0] * (y + dataSize[1] * z);

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
            const float* a = &vertexBuffer[i * 3];
            const float* b = &vertexBuffer[j * 3];

            if (fabsf(a[0] - b[0]) < epsilon &&
                fabsf(a[1] - b[1]) < epsilon &&
                fabsf(a[2] - b[2]) < epsilon)
            {
                return -1;
            }
        }
    }

    mcmDeleteMeshBuffer(mesh);

    return 0;
}
