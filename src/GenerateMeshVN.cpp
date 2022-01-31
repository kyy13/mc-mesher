// MCmesher
// Kyle J Burgess

#include "MCmesher.h"
#include "Mesh.h"

#include "LookupTable.h"
#include <unordered_map>

GenerateMeshResult GenerateMeshVN(
    Mesh* mesh,
    const float* data,
    Vector3<uint32_t> dataSize,
    Vector3<uint32_t> meshOrigin,
    Vector3<uint32_t> meshSize,
    float isoLevel)
{
    if (mesh == nullptr)
    {
        return GenerateMeshResult::ERROR_MESH_IS_NULL;
    }

    const Vector3<uint32_t> meshEnd = meshOrigin + meshSize;

    if (meshEnd.x >= dataSize.x)
    {
        return GenerateMeshResult::ERROR_OUT_OF_BOUNDS_X;
    }

    if (meshEnd.y >= dataSize.y)
    {
        return GenerateMeshResult::ERROR_OUT_OF_BOUNDS_Y;
    }

    if (meshEnd.z >= dataSize.z)
    {
        return GenerateMeshResult::ERROR_OUT_OF_BOUNDS_Z;
    }

    struct VertexCacheEntry
    {
        Vector3<float> p;
        Vector3<float> n;
        uint32_t i;
    };

    std::unordered_map<uint32_t, VertexCacheEntry> vertexCache;

    VertexCacheEntry* vertexCacheEntries[3];

    float corner[8];

    auto& vertices = mesh->vertices;
    auto& normals = mesh->normals;
    auto& indices = mesh->indices;

    uint32_t vertexCount = 0;

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

    const Vector3<uint32_t> maxCubeIndex =
        {
            .x = dataSize.x - 2,
            .y = dataSize.y - 2,
            .z = dataSize.z - 2,
        };

    // Memory offset lookup table for cube and adjacent faces

    /*               p28------ p30
     *              /|        /|
     *             / |       / |
     *            p29------ p31|
     *     p18----|- p4 ----|- p6 ------ p22
     *    /|      | /|      | /|        /|
     *   / |      |/ |      |/ |       / |      z
     *  p19------ p5 ------ p7 ------ p23|      |
     *  |  p16----|- p0 ----|- p2 ----|- p20    o----> y
     *  | /       | /|      | /|      | /      /
     *  |/        |/ |      |/ |      |/      x
     *  p17------ p1 ------ p3 ------ p21
     *            |  p24----|- p26
     *            | /       | /
     *            |/        |/
     *            p25------ p27
     *
     *
     *
     *                  p10------ p11
     *                 /|        /|
     *                / |       / |
     *               p4 ------ p6 |
     *              /|  p8 ---/|- p9
     *             / | /     / | /              z
     *            p5 ------ p7 |/               |
     *           /|  p0 ---/|- p2               o----> y
     *          / | /     / | /                /
     *         p14------ p15|/                x
     *         |  p1 ----|- p3
     *         | /       | /
     *         |/        |/
     *         p12------ p13
     */

    int memOffsets[32];

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

        memOffsets[8] = memOffsets[0] - 1;
        memOffsets[9] = memOffsets[2] - 1;
        memOffsets[10] = memOffsets[4] - 1;
        memOffsets[11] = memOffsets[6] - 1;

        memOffsets[12] = memOffsets[1] + 1;
        memOffsets[13] = memOffsets[3] + 1;
        memOffsets[14] = memOffsets[5] + 1;
        memOffsets[15] = memOffsets[7] + 1;

        memOffsets[16] = memOffsets[0] - mem_w;
        memOffsets[17] = memOffsets[1] - mem_w;
        memOffsets[18] = memOffsets[4] - mem_w;
        memOffsets[19] = memOffsets[5] - mem_w;

        memOffsets[20] = memOffsets[2] + mem_w;
        memOffsets[21] = memOffsets[3] + mem_w;
        memOffsets[22] = memOffsets[6] + mem_w;
        memOffsets[23] = memOffsets[7] + mem_w;

        memOffsets[24] = memOffsets[0] - mem_wh;
        memOffsets[25] = memOffsets[1] - mem_wh;
        memOffsets[26] = memOffsets[2] - mem_wh;
        memOffsets[27] = memOffsets[3] - mem_wh;

        memOffsets[28] = memOffsets[4] + mem_wh;
        memOffsets[29] = memOffsets[5] + mem_wh;
        memOffsets[30] = memOffsets[6] + mem_wh;
        memOffsets[31] = memOffsets[7] + mem_wh;
    }

    // Subset of memory offsets clamped to boundaries throughout the voxel iterator
    // first 8 indices are omitted, because they are always in scope, so range is [8,32)

    /*               p20------ p22
     *              /|        /|
     *             / |       / |
     *            p21------ p23|
     *     p10----|- c4 ----|- c6 ------ p14
     *    /|      | /|      | /|        /|
     *   / |      |/ |      |/ |       / |      z
     *  p11------ c5 ------ c7 ------ p15|      |
     *  |  p8 ----|- c0 ----|- c2 ----|- p12    o----> y
     *  | /       | /|      | /|      | /      /
     *  |/        |/ |      |/ |      |/      x
     *  p9 ------ c1 ------ c3 ------ p13
     *            |  p16----|- p18
     *            | /       | /
     *            |/        |/
     *            p17------ p19
     *
     *
     *
     *                  p2 ------ p3
     *                 /|        /|
     *                / |       / |
     *               c4 ------ c6 |
     *              /|  p0 ---/|- p1
     *             / | /     / | /              z
     *            c5 ------ c7 |/               |
     *           /|  c0 ---/|- c2               o----> y
     *          / | /     / | /                /
     *         p6 ------ p7 |/                x
     *         |  c1 ----|- c3
     *         | /       | /
     *         |/        |/
     *         p4 ------ p5
     */

    int memBoundedOffsets[24];

    for (; voxel != pz1; voxel += pdz)
    {
        y = meshOrigin.y;
        cubeOrigin.y = fieldOrigin.y;

        // Clamp -Z adjacent memory offsets
        if (z == 0)
        {
            memBoundedOffsets[16] = memOffsets[0];
            memBoundedOffsets[17] = memOffsets[1];
            memBoundedOffsets[18] = memOffsets[2];
            memBoundedOffsets[19] = memOffsets[3];
        }
        else
        {
            memBoundedOffsets[16] = memOffsets[24];
            memBoundedOffsets[17] = memOffsets[25];
            memBoundedOffsets[18] = memOffsets[26];
            memBoundedOffsets[19] = memOffsets[27];
        }

        // Clamp +Z adjacent memory offsets
        if (z == maxCubeIndex.z)
        {
            memBoundedOffsets[20] = memOffsets[4];
            memBoundedOffsets[21] = memOffsets[5];
            memBoundedOffsets[22] = memOffsets[6];
            memBoundedOffsets[23] = memOffsets[7];
        }
        else
        {
            memBoundedOffsets[20] = memOffsets[28];
            memBoundedOffsets[21] = memOffsets[29];
            memBoundedOffsets[22] = memOffsets[30];
            memBoundedOffsets[23] = memOffsets[31];
        }

        for (; voxel != py1; voxel += pdy)
        {
            x = meshOrigin.x;
            cubeOrigin.x = fieldOrigin.x;

            // Clamp -Y adjacent memory offsets
            if (y == 0)
            {
                memBoundedOffsets[8] = memOffsets[0];
                memBoundedOffsets[9] = memOffsets[1];
                memBoundedOffsets[10] = memOffsets[4];
                memBoundedOffsets[11] = memOffsets[5];
            }
            else
            {
                memBoundedOffsets[8] = memOffsets[16];
                memBoundedOffsets[9] = memOffsets[17];
                memBoundedOffsets[10] = memOffsets[18];
                memBoundedOffsets[11] = memOffsets[19];
            }

            // Clamp +Y adjacent memory offsets
            if (y == maxCubeIndex.y)
            {
                memBoundedOffsets[12] = memOffsets[2];
                memBoundedOffsets[13] = memOffsets[3];
                memBoundedOffsets[14] = memOffsets[6];
                memBoundedOffsets[15] = memOffsets[7];
            }
            else
            {
                memBoundedOffsets[12] = memOffsets[20];
                memBoundedOffsets[13] = memOffsets[21];
                memBoundedOffsets[14] = memOffsets[22];
                memBoundedOffsets[15] = memOffsets[23];
            }

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

                // Calculate vertex normals
                // Clamp -X adjacent memory offsets
                if (x == 0)
                {
                    memBoundedOffsets[0] = memOffsets[0];
                    memBoundedOffsets[1] = memOffsets[2];
                    memBoundedOffsets[2] = memOffsets[4];
                    memBoundedOffsets[3] = memOffsets[6];
                }
                else
                {
                    memBoundedOffsets[0] = memOffsets[8];
                    memBoundedOffsets[1] = memOffsets[9];
                    memBoundedOffsets[2] = memOffsets[10];
                    memBoundedOffsets[3] = memOffsets[11];
                }

                // Clamp +X adjacent memory offsets
                if (x == maxCubeIndex.x)
                {
                    memBoundedOffsets[4] = memOffsets[1];
                    memBoundedOffsets[5] = memOffsets[3];
                    memBoundedOffsets[6] = memOffsets[5];
                    memBoundedOffsets[7] = memOffsets[7];
                }
                else
                {
                    memBoundedOffsets[4] = memOffsets[12];
                    memBoundedOffsets[5] = memOffsets[13];
                    memBoundedOffsets[6] = memOffsets[14];
                    memBoundedOffsets[7] = memOffsets[15];
                }

                // Setup un-normalized cube normals
                Vector3<float> cubeNormals[8];

                cubeNormals[0] =
                    {
                        .x = *(voxel + memBoundedOffsets[0]) - corner[1],
                        .y = *(voxel + memBoundedOffsets[8]) - corner[2],
                        .z = *(voxel + memBoundedOffsets[16]) - corner[4],
                    };

                cubeNormals[1] =
                    {
                        .x = corner[0] - *(voxel + memBoundedOffsets[4]),
                        .y = *(voxel + memBoundedOffsets[9]) - corner[3],
                        .z = *(voxel + memBoundedOffsets[17]) - corner[5],
                    };

                cubeNormals[2] =
                    {
                        .x = *(voxel + memBoundedOffsets[1]) - corner[3],
                        .y = corner[0] - *(voxel + memBoundedOffsets[12]),
                        .z = *(voxel + memBoundedOffsets[18]) - corner[6],
                    };

                cubeNormals[3] =
                    {
                        .x = corner[2] - *(voxel + memBoundedOffsets[5]),
                        .y = corner[1] - *(voxel + memBoundedOffsets[13]),
                        .z = *(voxel + memBoundedOffsets[19]) - corner[7],
                    };

                cubeNormals[4] =
                    {
                        .x = *(voxel + memBoundedOffsets[2]) - corner[5],
                        .y = *(voxel + memBoundedOffsets[10]) - corner[6],
                        .z = corner[0] - *(voxel + memBoundedOffsets[20]),
                    };

                cubeNormals[5] =
                    {
                        .x = corner[4] - *(voxel + memBoundedOffsets[6]),
                        .y = *(voxel + memBoundedOffsets[11]) - corner[7],
                        .z = corner[1] - *(voxel + memBoundedOffsets[21]),
                    };

                cubeNormals[6] =
                    {
                        .x = *(voxel + memBoundedOffsets[3]) - corner[7],
                        .y = corner[4] - *(voxel + memBoundedOffsets[14]),
                        .z = corner[2] - *(voxel + memBoundedOffsets[22]),
                    };

                cubeNormals[7] =
                    {
                        .x = corner[6] - *(voxel + memBoundedOffsets[7]),
                        .y = corner[5] - *(voxel + memBoundedOffsets[15]),
                        .z = corner[3] - *(voxel + memBoundedOffsets[23]),
                    };

                // Normalize normal vectors

                for (auto& cubeNormal : cubeNormals)
                {
                    cubeNormal.normalize();
                }

                // Step vertices
                for (uint32_t i = 0; i != vertCount; i += 3)
                {
                    for (uint32_t j = 0; j != 3; ++j)
                    {
                        uint32_t vertexIndex = LookupTable::RegularCellData[cellClass16 + i + j + 1];
                        uint32_t vertexData = LookupTable::RegularVertexData[caseIndex12 + vertexIndex] & 0xFFu;

                        /*
                         *      edge key = (c index) * 3 + edge key
                         *
                         *            c4 (e#2)
                         *            |
                         *            |                     z
                         *            |                     |
                         *            c ------- c2 (e#1)    o----> y
                         *           /                     /
                         *          /                     x
                         *         c1 (e#0)
                         */

                        const uint32_t cacheBits = LookupTable::EdgeCacheBits[vertexData];

                        uint32_t cacheKey = voxel - origin;

                        if ((cacheBits & 4u) != 0) cacheKey += 1;
                        if ((cacheBits & 8u) != 0) cacheKey += w;
                        if ((cacheBits & 16u) != 0) cacheKey += wh;

                        cacheKey = 3u * cacheKey + (cacheBits & 0b11u);

                        auto it = vertexCache.find(cacheKey);

                        if (it != vertexCache.end())
                        {
                            vertexCacheEntries[j] = &it->second;
                        }
                        else
                        {
                            auto cacheEntry = &vertexCache[cacheKey];

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
                            cacheEntry->p = endpoint + k * dEndpoint;

                            // Lerp vertex normals
                            const Vector3<float>& normal0 = cubeNormals[endpointIndex[0]];
                            const Vector3<float> dNormal = cubeNormals[endpointIndex[1]] - normal0;

                            cacheEntry->n = normal0 + k * dNormal;
                            cacheEntry->n.normalize();

                            // Cache
                            cacheEntry->i = 0xffffffffu;
                            vertexCacheEntries[j] = cacheEntry;
                        }
                    }

                    // Calculate triangle segments that are too small to render
                    constexpr float epsilon = 0.0000001f;

                    Vector3 d01 = vertexCacheEntries[1]->p - vertexCacheEntries[0]->p;

                    if (fabsf(d01.x) < epsilon && fabsf(d01.y) < epsilon && fabsf(d01.z) < epsilon)
                    {
                        continue;
                    }

                    Vector3 d12 = vertexCacheEntries[2]->p - vertexCacheEntries[1]->p;

                    if (fabsf(d12.x) < epsilon && fabsf(d12.y) < epsilon && fabsf(d12.z) < epsilon)
                    {
                        continue;
                    }

                    Vector3 d02 = vertexCacheEntries[2]->p - vertexCacheEntries[0]->p;

                    if (fabsf(d02.x) < epsilon && fabsf(d02.y) < epsilon && fabsf(d02.z) < epsilon)
                    {
                        continue;
                    }

                    for (auto entry : vertexCacheEntries)
                    {
                        if (entry->i == 0xffffffffu)
                        {
                            vertices.push_back(entry->p);
                            normals.push_back(entry->n);
                            indices.push_back(vertexCount);
                            entry->i = vertexCount;
                            ++vertexCount;
                        }
                        else
                        {
                            indices.push_back(entry->i);
                        }
                    }
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

    return GenerateMeshResult::SUCCESS;
}
