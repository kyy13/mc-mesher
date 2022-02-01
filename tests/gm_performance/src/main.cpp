// MCmesher TEST file
// Kyle J Burgess

#include "MCmesher.h"
#include "gradient_noise.h"

#include <ctime>
#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>

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
    gnd::gradient_noise<float, 3> noise3d(532346);
    std::vector<float> data(dataSize.x * dataSize.y * dataSize.z);
    for (size_t z = 0; z != dataSize.z; ++z)
    {
        for (size_t y = 0; y != dataSize.y; ++y)
        {
            for (size_t x = 0; x != dataSize.x; ++x)
            {
                size_t i = x + dataSize.x * (y + dataSize.y * z);

                data[i] = noise3d({
                    static_cast<float>(x) * 0.05f,
                    static_cast<float>(y) * 0.05f,
                    static_cast<float>(z) * 0.05f,
                });

                if (data[i] < -1.0f) data[i] = -1.0f;
                if (data[i] > 1.0f) data[i] = 1.0f;
            }
        }
    }

    std::cout << "generating meshes...\n";

    auto mesh = CreateMesh();

    size_t n = 0;
    clock_t dt = 0;
    clock_t dt_sum = 0;
    for (float isoLevel = -1.0f; isoLevel <= 1.0f; isoLevel += 0.05f)
    {
        clock_t t = clock();
        GenerateMeshFN(mesh, data.data(), dataSize, meshOrigin, meshSize, isoLevel);
        dt = (clock() - t);
        dt_sum += dt;

        float tf = static_cast<float>(dt) / static_cast<float>(CLOCKS_PER_SEC);
        std::cout
            << "isoLevel=" << isoLevel
            << ", dt=" << tf << "s"
            << ", indices=" << CountIndices(mesh)
            << ", vertices=" << CountVertices(mesh)
            << '\n';

        ++n;
    }

    float tf_sum = static_cast<float>(dt_sum) / static_cast<float>(CLOCKS_PER_SEC);
    std::cout
        << "dt sum=" << tf_sum << "s"
        << ", dt avg=" << (tf_sum / static_cast<float>(n))
        << '\n';

    DeleteMesh(mesh);
}
