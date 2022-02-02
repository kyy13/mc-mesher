// MCmesher TEST file
// Kyle J Burgess

#include "MCmesher.h"
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

template<auto fn>
float TestGenerateMesh(McmMeshBuffer* mesh, const float* data, Vector3<uint32_t> dataSize, Vector3<uint32_t> meshOrigin, Vector3<uint32_t> meshSize)
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
        result = fn(mesh, data, dataSize, meshOrigin, meshSize, isoLevel);
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
    const Vector3<uint32_t> dataSize =
        {
            .x = 128,
            .y = 128,
            .z = 128,
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

    std::cout << "generating noise...\n";

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

    std::cout << "generating meshes...\n";

    auto mesh = mcmCreateMeshBuffer();

    std::cout << "mcmGenerateMeshVN 256^3 = ";
    auto t = TestGenerateMesh<mcmGenerateMeshVN>(mesh, scalarField.data(), dataSize, meshOrigin, meshSize);
    std::cout << t << "s (avg = ";
    std::cout << (t / 256.0f) << "s)\n";

    mcmDeleteMeshBuffer(mesh);
    mesh = mcmCreateMeshBuffer();

    std::cout << "GenerateMeshFN 256^3 => ";
    t = TestGenerateMesh<mcmGenerateMeshFN>(mesh, scalarField.data(), dataSize, meshOrigin, meshSize);
    std::cout << t << "s (avg = ";
    std::cout << (t / 256.0f) << "s)\n";

    mcmDeleteMeshBuffer(mesh);
}
