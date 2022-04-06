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

    // Generate mesh with single cube mcmComputeCaseGeometry and mcmComputeCaseIndex

    std::vector<uint32_t> indicesC;
    std::vector<Vector3<float>> verticesC;

    uint32_t mem_w = dataSize.x;
    uint32_t mem_wh = dataSize.x * dataSize.y;

    for (uint32_t z = meshOrigin.z; z != meshOrigin.z + meshSize.z; ++z)
    {
        for (uint32_t y = meshOrigin.y; y != meshOrigin.y + meshSize.y; ++y)
        {
            for (uint32_t x = meshOrigin.x; x != meshOrigin.x + meshSize.x; ++x)
            {
                uint32_t i = x + dataSize.x * (y + dataSize.y * z);

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

                    Vector3<float> v =
                        {
                            .x = vOut[m].x + static_cast<float>(x),
                            .y = vOut[m].y + static_cast<float>(y),
                            .z = vOut[m].z + static_cast<float>(z),
                        };

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

        auto dx = fabsf(verticesA[indexA].x - verticesB[indexB].x);
        auto dy = fabsf(verticesA[indexA].y - verticesB[indexB].y);
        auto dz = fabsf(verticesA[indexA].z - verticesB[indexB].z);

        if (!(dx < epsilon && dy < epsilon && dz < epsilon))
        {
            return -1;
        }

        dx = fabsf(verticesA[indexA].x - verticesC[indexC].x);
        dy = fabsf(verticesA[indexA].y - verticesC[indexC].y);
        dz = fabsf(verticesA[indexA].z - verticesC[indexC].z);

        if (!(dx < epsilon && dy < epsilon && dz < epsilon))
        {
            return -1;
        }
    }

    mcmDeleteMeshBuffer(meshA);
    mcmDeleteMeshBuffer(meshB);

    return 0;
}
