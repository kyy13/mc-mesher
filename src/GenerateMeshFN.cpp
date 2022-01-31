// MCmesher
// Kyle J Burgess

#include "MCmesher.h"
#include "Mesh.h"

#include "LookupTable.h"
#include <unordered_map>

GenerateMeshResult GenerateMeshFN(
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

                            // Cache
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

                    vertices.push_back(vertexCacheEntries[0]->p);
                    vertices.push_back(vertexCacheEntries[1]->p);
                    vertices.push_back(vertexCacheEntries[2]->p);

                    Vector3<float> triFaceNormal = Vector3<float>::cross(d01, d02);
                    triFaceNormal.normalize();

                    normals.push_back(triFaceNormal);
                    normals.push_back(triFaceNormal);
                    normals.push_back(triFaceNormal);

                    indices.push_back(vertexCount);
                    ++vertexCount;
                    indices.push_back(vertexCount);
                    ++vertexCount;
                    indices.push_back(vertexCount);
                    ++vertexCount;
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
