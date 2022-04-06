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

bool testNormals(const Vector3<float>* normals, size_t count)
{
    const float epsilon = 1e-2f;

    for (size_t i = 0; i != count; ++i)
    {
        const auto& n = normals[i];

        float size_2 = n.x * n.x + n.y * n.y + n.z * n.z;

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
