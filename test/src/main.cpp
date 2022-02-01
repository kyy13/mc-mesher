// MCmesher TEST file
// Kyle J Burgess

#include "MCmesher.h"

#include <vector>
#include <iostream>
#include <iomanip>
#include <fstream>

int main()
{
    const Vector3<uint32_t> dataSize =
        {
            .x = 256,
            .y = 256,
            .z = 256,
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

    auto mesh = CreateMesh();

    //GenerateMeshVN(mesh, data.data(), dataSize, meshOrigin, meshSize, 0.5f);

    size_t size = CountIndices(mesh);
    const uint32_t* arr = GetIndices(mesh);

    for (size_t i = 0; i != size; ++i)
    {
        std::cout << arr[i] << "\n";
    }

    DeleteMesh(mesh);
}
