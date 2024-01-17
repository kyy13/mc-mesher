// mc-mesher TEST file
// Kyle J Burgess

#include "mc_mesher.h"
#include "gradient_noise.h"
#include "McmGeometry.h"

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

    // Generate mesh with single cube mcmComputeCaseGeometry and mcmComputeCaseIndex

    std::vector<uint32_t> indicesC;
    std::vector<Vector3<float>> verticesC;

    uint32_t mem_w = dataSize[0];
    uint32_t mem_wh = dataSize[0] * dataSize[1];

    for (uint32_t z = meshOrigin[2]; z != meshOrigin[2] + meshSize[2]; ++z)
    {
        for (uint32_t y = meshOrigin[1]; y != meshOrigin[1] + meshSize[1]; ++y)
        {
            for (uint32_t x = meshOrigin[0]; x != meshOrigin[0] + meshSize[0]; ++x)
            {
                uint32_t i = x + dataSize[0] * (y + dataSize[1] * z);

                float* voxel = &scalarField[i];

                const float corners[] =
                    {
                        *(voxel),
                        *(voxel + 1),
                        *(voxel + mem_w),
                        *(voxel + mem_w + 1),
                        *(voxel + mem_wh),
                        *(voxel + mem_wh + 1),
                        *(voxel + mem_wh + mem_w),
                        *(voxel + mem_wh + mem_w + 1),
                    };

                uint8_t caseIndex = mcmComputeCaseIndex(corners, 0.0f);

                Vector3<float> vOut[12];

                uint32_t numVertices = mcmComputeCaseGeometry<float, true>(corners, 0.0f, vOut);

                for (uint32_t m = 0; m != numVertices; ++m)
                {
                    indicesC.push_back(verticesC.size());

                    Vector3<float> v = Vector3<float>(
                            vOut[m].x + static_cast<float>(x),
                            vOut[m].y + static_cast<float>(y),
                            vOut[m].z + static_cast<float>(z));

                    verticesC.push_back(v);
                }
            }
        }
    }

    // compare

    size_t numIndices = mcmCountIndices(meshA);

    if (numIndices != mcmCountIndices(meshB))
    {
        return -1;
    }

    if (numIndices != indicesC.size())
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

    // Compare
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

        auto indexC = indicesC[i];

        if (indexC >= verticesC.size())
        {
            return -1;
        }

        constexpr float epsilon = 0.00001f;

        auto dx = fabsf(verticesA[3*indexA + 0] - verticesB[3*indexB + 0]);
        auto dy = fabsf(verticesA[3*indexA + 1] - verticesB[3*indexB + 1]);
        auto dz = fabsf(verticesA[3*indexA + 2] - verticesB[3*indexB + 2]);

        if (!(dx < epsilon && dy < epsilon && dz < epsilon))
        {
            return -1;
        }

        dx = fabsf(verticesA[3*indexA + 0] - verticesC[indexC].x);
        dy = fabsf(verticesA[3*indexA + 1] - verticesC[indexC].y);
        dz = fabsf(verticesA[3*indexA + 2] - verticesC[indexC].z);

        if (!(dx < epsilon && dy < epsilon && dz < epsilon))
        {
            return -1;
        }
    }

    mcmDeleteMeshBuffer(meshA);
    mcmDeleteMeshBuffer(meshB);

    return 0;
}
