// mc-mesher TEST file
// Kyle J Burgess

#include "mc_mesher.h"
#include "gradient_noise.h"

#include <ctime>
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

float TestGenerateMesh(McmMeshBuffer* mesh, const float* data, const uint32_t dataSize[3], const uint32_t meshOrigin[3], const uint32_t meshSize[3], McmFlag flags)
{
    constexpr float twoOver256 = 2.0f / 256.0f;
    constexpr float secPerClock = 1.0f / static_cast<float>(CLOCKS_PER_SEC);

    clock_t dt = 0;
    clock_t t;
    McmResult result;

    for (size_t i = 0; i != 256; ++i)
    {
        float isoLevel = -1.0f + twoOver256 * static_cast<float>(i);

        t = clock();
        result = mcmGenerateMesh(mesh, data, dataSize, meshOrigin, meshSize, isoLevel, flags);
        dt += (clock() - t);

        if (result != McmResult::MCM_SUCCESS)
        {
            throw;
        }
    }

    return secPerClock * static_cast<float>(dt);
}

int main()
{
    const uint32_t dataSize[3] =
        {
            128,
            128,
            128,
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

    std::cout << "generating noise...\n";

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

    std::cout << "generating meshes...\n";

    auto mesh = mcmCreateMeshBuffer();

    std::cout << "mcmGenerateMeshVN 256^3 = ";
    auto t = TestGenerateMesh(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, MCM_VERTEX_NORMALS);
    std::cout << t << "s (avg = ";
    std::cout << (t / 256.0f) << "s)\n";

    mcmDeleteMeshBuffer(mesh);
    mesh = mcmCreateMeshBuffer();

    std::cout << "mcmGenerateMeshFN 256^3 => ";
    t = TestGenerateMesh(mesh, scalarField.data(), dataSize, meshOrigin, meshSize, MCM_FACE_NORMALS);
    std::cout << t << "s (avg = ";
    std::cout << (t / 256.0f) << "s)\n";

    mcmDeleteMeshBuffer(mesh);
}
