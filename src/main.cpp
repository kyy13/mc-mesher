// MCmesher TEST file
// Kyle J Burgess

#include "MCmesher.h"

#include <vector>
#include <iostream>

int main()
{
    const Vector3<uint32_t> dataSize =
        {
            .x = 4,
            .y = 4,
            .z = 4,
        };

    const std::vector<float> data =
        {
            0,0.3207658,0.5359496,0.569116,-0.1141592,0.2016224,0.4672953,0.5943463,-0.1721031,0.09653023,0.3694703,0.5387987,-0.1499517,0.04908073,0.3108177,0.5018625,0.1194656,0.4169789,0.6118618,0.6341059,0.01077445,0.3115193,0.5633928,0.6800475,-0.05944544,0.2063428,0.4798266,0.6543195,-0.07367653,0.1377311,0.4167093,0.6333929,0.2337421,0.4763889,0.6272331,0.6387581,0.1452572,0.3926565,0.5947783,0.6928875,0.04478313,0.2665647,0.498939,0.6666303,-0.03664279,0.1464832,0.3983899,0.6245985,0.3603238,0.5235451,0.5991657,0.586514,0.2785117,0.4397385,0.5525945,0.6071839,0.138537,0.2754115,0.4162852,0.5426846,-0.01500723,0.09798103,0.2650715,0.4565942,
        };

    auto mesh = CreateMesh();

    const Vector3<uint32_t> meshOrigin =
        {
            .x = 0,
            .y = 0,
            .z = 0,
        };

    const Vector3<uint32_t> meshSize =
        {
            .x = 3,
            .y = 3,
            .z = 3,
        };

    GenerateMesh(mesh, data.data(), dataSize, meshOrigin, meshSize, 0.5f, true, true);

    size_t size = CountIndices(mesh);
    const uint32_t* arr = GetIndices(mesh);

    for (size_t i = 0; i != size; ++i)
    {
        std::cout << arr[i] << "\n";
    }

    DeleteMesh(mesh);
}
