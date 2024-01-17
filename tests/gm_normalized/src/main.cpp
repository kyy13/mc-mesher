// mc-mesher TEST file
// Kyle J Burgess

#include "mc_mesher.h"
#include "gradient_noise.h"
#include "McmGeometry.h"

#include <cstring>
#include <vector>
#include <unordered_map>
#include <cmath>
#include <iostream>

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

bool testNormals(const float* normals, size_t count)
{
    const float epsilon = 1e-2f;

    for (size_t i = 0; i != count; ++i)
    {
        const float* n = &normals[i * 3];

        float size_2 = 
            n[0] * n[0] + 
            n[1] * n[1] + 
            n[2] * n[2];

        if (fabsf(1.0f - size_2) > epsilon)
        {
            std::cout << "size_2 = " << size_2 << "\n";
            return false;
        }
    }

    return true;
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
            2,
            2,
            2,
        };

    const uint32_t meshSize[3] =
        {
            dataSize[0] - 5,
            dataSize[1] - 5,
            dataSize[2] - 5,
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

    auto meshA = mcmCreateMeshBuffer();
    auto meshB = mcmCreateMeshBuffer();

    // Generate Mesh with face normals

    auto result = mcmGenerateMesh(meshA, scalarField.data(), dataSize, meshOrigin, meshSize, 0.0f, MCM_FACE_NORMALS);

    if (result != McmResult::MCM_SUCCESS)
    {
        return -1;
    }

    // Generate mesh with vertex normals

    result = mcmGenerateMesh(meshB, scalarField.data(), dataSize, meshOrigin, meshSize, 0.0f, MCM_VERTEX_NORMALS);

    if (result != McmResult::MCM_SUCCESS)
    {
        return -1;
    }

    // test for normalized normals

    const auto* normalsA = mcmGetNormals(meshA);
    const auto* normalsB = mcmGetNormals(meshB);

    const auto normalCountA = mcmCountNormals(meshA);
    const auto normalCountB = mcmCountNormals(meshB);

    if (!testNormals(normalsA, normalCountA))
    {
        return -1;
    }

    if (!testNormals(normalsB, normalCountB))
    {
        return -1;
    }

    mcmDeleteMeshBuffer(meshA);
    mcmDeleteMeshBuffer(meshB);

    return 0;
}
