// MCmesher
// Kyle J Burgess

#include "MCmesher.h"
#include "Mesh.h"
#include "Common.h"
#include "LookupTable.h"

#include <unordered_map>

McmResult mcmGenerateMeshFN(
    McmMeshBuffer* mesh,
    const float* data,
    Vector3<uint32_t> dataSize,
    Vector3<uint32_t> meshOrigin,
    Vector3<uint32_t> meshSize,
    float isoLevel)
{
    if (mesh == nullptr)
    {
        return McmResult::MCM_MESH_BUFFER_IS_NULL;
    }

    const Vector3<uint32_t> meshEnd = meshOrigin + meshSize;

    if (meshEnd.x >= dataSize.x)
    {
        return McmResult::MCM_OUT_OF_BOUNDS_X;
    }

    if (meshEnd.y >= dataSize.y)
    {
        return McmResult::MCM_OUT_OF_BOUNDS_Y;
    }

    if (meshEnd.z >= dataSize.z)
    {
        return McmResult::MCM_OUT_OF_BOUNDS_Z;
    }

    float corner[8];

    auto& vertices = mesh->vertices;
    auto& normals = mesh->normals;
    auto& indices = mesh->indices;

    vertices.clear();
    indices.clear();
    normals.clear();

    const uint32_t w = dataSize.x;
    const uint32_t wh = dataSize.x * dataSize.y;

    const float* origin = &data[meshOrigin.z * wh + meshOrigin.y * w + meshOrigin.x];
    const float* voxel = origin;

    const float* px1 = &voxel[meshSize.x];
    const float* py1 = &voxel[meshSize.y * w];
    const float* pz1 = &voxel[meshSize.z * wh];

    const uint32_t pdy = w - meshSize.x; // wrap to next y
    const uint32_t pdz = (dataSize.y - meshSize.y) * w; // wrap to next z

    uint32_t x, y, z = meshOrigin.z;

    const Vector3<float> fieldOrigin =
        {
            static_cast<float>(meshOrigin.x),
            static_cast<float>(meshOrigin.y),
            static_cast<float>(meshOrigin.z),
        };

    Vector3<float> cubeOrigin = fieldOrigin;

    // Memory offset lookup table for cube vertices

    /*
     *     p4 ------ p6
     *    /|        /|
     *   / |       / |    z
     *  p5 ------ p7 |    |
     *  |  p0 ----|- p2   o----> y
     *  | /       | /    /
     *  |/        |/    x
     *  p1 ------ p3
     */

    int memOffsets[8];

    {
        const int mem_w = static_cast<int>(dataSize.x);
        const int mem_wh = static_cast<int>(dataSize.x * dataSize.y);

        memOffsets[0] = 0;
        memOffsets[1] = 1;
        memOffsets[2] = mem_w;
        memOffsets[3] = mem_w + 1;

        memOffsets[4] = mem_wh + memOffsets[0];
        memOffsets[5] = mem_wh + memOffsets[1];
        memOffsets[6] = mem_wh + memOffsets[2];
        memOffsets[7] = mem_wh + memOffsets[3];
    }

    for (; voxel != pz1; voxel += pdz)
    {
        y = meshOrigin.y;
        cubeOrigin.y = fieldOrigin.y;

        for (; voxel != py1; voxel += pdy)
        {
            x = meshOrigin.x;
            cubeOrigin.x = fieldOrigin.x;

            for (; voxel != px1; ++voxel)
            {
                corner[0] = *(voxel);
                corner[1] = *(voxel + 1);
                corner[2] = *(voxel + memOffsets[2]);
                corner[3] = *(voxel + memOffsets[3]);
                corner[4] = *(voxel + memOffsets[4]);
                corner[5] = *(voxel + memOffsets[5]);
                corner[6] = *(voxel + memOffsets[6]);
                corner[7] = *(voxel + memOffsets[7]);

                // Get Case Index
                uint8_t caseIndex = 0;

                if (corner[0] >= isoLevel) caseIndex |= 0x01u;
                if (corner[1] >= isoLevel) caseIndex |= 0x02u;
                if (corner[2] >= isoLevel) caseIndex |= 0x04u;
                if (corner[3] >= isoLevel) caseIndex |= 0x08u;
                if (corner[4] >= isoLevel) caseIndex |= 0x10u;
                if (corner[5] >= isoLevel) caseIndex |= 0x20u;
                if (corner[6] >= isoLevel) caseIndex |= 0x40u;
                if (corner[7] >= isoLevel) caseIndex |= 0x80u;

                // Cell has trivial triangulation
                if (caseIndex == 0x00u || caseIndex == 0xffu)
                {
                    ++x;
                    cubeOrigin.x += 1.0f;
                    continue;
                }

                uint32_t caseIndex12 = caseIndex * 12u;
                uint32_t cellClass16 = LookupTable::RegularCellClass[caseIndex] << 4u;

                uint32_t geometryCounts = LookupTable::RegularCellData[cellClass16];
                uint32_t triangleCount = (geometryCounts & 0x0Fu);
                uint32_t vertCount = triangleCount * 3;

                // Step vertices
                for (uint32_t i = 0; i != vertCount; i += 3)
                {
                    Vector3<float> vbuffer[3];

                    for (uint32_t j = 0; j != 3; ++j)
                    {
                        uint32_t vertexIndex = LookupTable::RegularCellData[cellClass16 + i + j + 1];
                        uint32_t vertexData = LookupTable::RegularVertexData[caseIndex12 + vertexIndex] & 0xFFu;

                        const uint32_t endpointIndex[2] =
                            {
                                vertexData >> 3u,
                                vertexData & 0x07u,
                            };

                        const Vector3<float>& d0 = LookupTable::UnitCube[endpointIndex[0]];
                        const Vector3<float> endpoint = cubeOrigin + d0;
                        const Vector3<float> dEndpoint = LookupTable::UnitCube[endpointIndex[1]] - d0;

                        // Lerp factor between endpoints
                        float k = (isoLevel - corner[endpointIndex[0]]) / (corner[endpointIndex[1]] - corner[endpointIndex[0]]);

                        // Lerp vertices
                        vbuffer[j] = endpoint + k * dEndpoint;
                    }

                    // Calculate triangle segments that are too small to render
                    constexpr float epsilon = 0.0000001f;

                    Vector3 d01 = vbuffer[1] - vbuffer[0];

                    if (fabsf(d01.x) < epsilon && fabsf(d01.y) < epsilon && fabsf(d01.z) < epsilon)
                    {
                        continue;
                    }

                    Vector3 d12 = vbuffer[2] - vbuffer[1];

                    if (fabsf(d12.x) < epsilon && fabsf(d12.y) < epsilon && fabsf(d12.z) < epsilon)
                    {
                        continue;
                    }

                    Vector3 d02 = vbuffer[2] - vbuffer[0];

                    if (fabsf(d02.x) < epsilon && fabsf(d02.y) < epsilon && fabsf(d02.z) < epsilon)
                    {
                        continue;
                    }

                    vertices.push_back(vbuffer[0]);
                    vertices.push_back(vbuffer[1]);
                    vertices.push_back(vbuffer[2]);

                    Vector3<float> triFaceNormal = Vector3<float>::cross(d01, d02);
                    triFaceNormal.normalize();

                    normals.push_back(triFaceNormal);
                    normals.push_back(triFaceNormal);
                    normals.push_back(triFaceNormal);
                }
                ++x;
                cubeOrigin.x += 1.0f;
            }
            px1 += w;
            ++y;
            cubeOrigin.y += 1.0f;
        }
        px1 += pdz;
        py1 += wh;
        ++z;
        cubeOrigin.z += 1.0f;
    }

    // Write Indices
    size_t vertexCount = vertices.size();
    indices.reserve(vertexCount);
    for (uint32_t i = 0; i != vertexCount; ++i)
    {
        indices.push_back(i);
    }

    return McmResult::MCM_SUCCESS;
}
