// MCmesher
// Kyle J Burgess

#include "Common.h"
#include "LookupTable.h"

uint8_t mcmComputeCaseIndex(const float corners[8], float isoLevel)
{
    uint8_t caseIndex = 0u;

    if (corners[0] >= isoLevel) caseIndex |= 0x01u;
    if (corners[1] >= isoLevel) caseIndex |= 0x02u;
    if (corners[2] >= isoLevel) caseIndex |= 0x04u;
    if (corners[3] >= isoLevel) caseIndex |= 0x08u;
    if (corners[4] >= isoLevel) caseIndex |= 0x10u;
    if (corners[5] >= isoLevel) caseIndex |= 0x20u;
    if (corners[6] >= isoLevel) caseIndex |= 0x40u;
    if (corners[7] >= isoLevel) caseIndex |= 0x80u;

    return caseIndex;
}

// Lookup table for mcmComputeEdgeCacheKey
// Indexed by vertexData (RegularVertexData & 0xFFu) where the first byte if the first corner number [0,8],
// and the second byte is the second corner number [0,8].
// Each element has the following bits in logical order: edgeNumber (2 bits) -> dx(1 bit) -> dy(1 bit) -> dz(1 bit)
// 0xff is an impossible case (diagonal corner connection)
// dx, dy, dz is 0 if the edge lies on the current vertex, 1 if the edge lies on the next vertex (positive direction on respective axis)

const uint32_t McmEdgeCacheLookup[] =
    {
        0xff, 0x00, 0x01, 0xff, 0x02, 0xff, 0xff, 0xff,
        0x00, 0xff, 0xff, 0x05, 0xff, 0x06, 0xff, 0xff,
        0x01, 0xff, 0xff, 0x08, 0xff, 0xff, 0x0a, 0xff,
        0xff, 0x05, 0x08, 0xff, 0xff, 0xff, 0xff, 0x0e,
        0x02, 0xff, 0xff, 0xff, 0xff, 0x10, 0x11, 0xff,
        0xff, 0x06, 0xff, 0xff, 0x10, 0xff, 0xff, 0x15,
        0xff, 0xff, 0x0a, 0xff, 0x11, 0xff, 0xff, 0x18,
        0xff, 0xff, 0xff, 0x0e, 0xff, 0x15, 0x18, 0xff,
    };

/*
 *      edge key = (vertex#) * 3 + (edge# [0,2])
 *
 *            vz (e#2)
 *            |
 *            |                     z
 *            |                     |
 *            v ------- vy (e#1)    o----> y
 *           /                     /
 *          /                     x
 *         vx (e#0)
 */

uint32_t mcmComputeEdgeCacheKey(uint32_t memPos, uint32_t memDy, uint32_t memDz, uint32_t vertexData)
{
    const uint32_t cacheBits = McmEdgeCacheLookup[vertexData];

    // Shift memory to cube that contains edge
    if ((cacheBits & 4u ) != 0) memPos += 1;
    if ((cacheBits & 8u ) != 0) memPos += memDy;
    if ((cacheBits & 16u) != 0) memPos += memDz;

    // key = (3 * cube#) + (edge# [0,2])
    return 3u * memPos + (cacheBits & 0b11u);
}

uint32_t mcmComputeCaseGeometry(const float corners[8], float isoLevel, Vector3<float> vertices[12])
{
    uint8_t caseIndex = mcmComputeCaseIndex(corners, isoLevel);

    if (caseIndex == 0x00u || caseIndex == 0xffu)
    {
        return 0;
    }

    const uint32_t caseIndex12 = caseIndex * 12u;
    const uint32_t cellClass16 = LookupTable::RegularCellClass[caseIndex] << 4u;

    const uint32_t geometryCounts = LookupTable::RegularCellData[cellClass16];
    const uint32_t triangleCount = (geometryCounts & 0x0Fu);
    const uint32_t vertCount = triangleCount * 3;

    // Step vertices
    uint32_t outVertexCount = 0;

    for (uint32_t i = 0; i != vertCount; i += 3)
    {
        Vector3<float> triangleVertices[3];

        for (uint32_t j = 0; j != 3; ++j)
        {
            const uint32_t vertexIndex = LookupTable::RegularCellData[cellClass16 + i + j + 1];
            const uint32_t vertexData = LookupTable::RegularVertexData[caseIndex12 + vertexIndex] & 0xFFu;

            const uint32_t endpointIndex[2] =
                {
                    vertexData >> 3u,
                    vertexData & 0x07u,
                };

            const Vector3<float>& endpoint = LookupTable::UnitCube[endpointIndex[0]];
            const Vector3<float> dEndpoint = LookupTable::UnitCube[endpointIndex[1]] - endpoint;

            // Lerp factor between endpoints
            const float k = (isoLevel - corners[endpointIndex[0]]) / (corners[endpointIndex[1]] - corners[endpointIndex[0]]);

            // Lerp vertices
            triangleVertices[j] = endpoint + k * dEndpoint;
        }

        // Calculate triangle segments that are too small to render
        constexpr float epsilon = 0.0000001f;

        Vector3<float> d01 = triangleVertices[1] - triangleVertices[0];

        if (fabsf(d01.x) < epsilon && fabsf(d01.y) < epsilon && fabsf(d01.z) < epsilon)
        {
            continue;
        }

        Vector3<float> d12 = triangleVertices[2] - triangleVertices[1];

        if (fabsf(d12.x) < epsilon && fabsf(d12.y) < epsilon && fabsf(d12.z) < epsilon)
        {
            continue;
        }

        Vector3<float> d02 = triangleVertices[2] - triangleVertices[0];

        if (fabsf(d02.x) < epsilon && fabsf(d02.y) < epsilon && fabsf(d02.z) < epsilon)
        {
            continue;
        }

        // Set vertices
        vertices[outVertexCount] = triangleVertices[0];
        ++outVertexCount;
        vertices[outVertexCount] = triangleVertices[1];
        ++outVertexCount;
        vertices[outVertexCount] = triangleVertices[2];
        ++outVertexCount;
    }

    return outVertexCount;
}

bool mcmRayIntersectTriangle(Vector3<float> rayPos, Vector3<float> rayDir, const Vector3<float> triangle[3], Vector3<float>& pIntersect)
{
    constexpr float epsilon = 0.0000001f;

    const Vector3<float> pvec =
        {
            .x = rayDir.y * (triangle[2].z - triangle[0].z) - rayDir.z * (triangle[2].y - triangle[0].y),
            .y = rayDir.z * (triangle[2].x - triangle[0].x) - rayDir.x * (triangle[2].z - triangle[0].z),
            .z = rayDir.x * (triangle[2].y - triangle[0].y) - rayDir.y * (triangle[2].x - triangle[0].x),
        };

    float det =
        (triangle[1].x - triangle[0].x) * pvec.x +
        (triangle[1].y - triangle[0].y) * pvec.y +
        (triangle[1].z - triangle[0].z) * pvec.z;

    // always returns false if triangle is back-facing
    if (det < epsilon)
    {
        return false;
    }

    // inverse determinant
    det = 1.0f / det;

    const float u = det * (
        (rayPos.x - triangle[0].x) * pvec.x +
        (rayPos.y - triangle[0].y) * pvec.y +
        (rayPos.z - triangle[0].z) * pvec.z);

    if (u < 0.0f || u > 1.0f)
    {
        return false;
    }

    const Vector3<float> qvec =
        {
            .x = (rayPos.y - triangle[0].y) * (triangle[1].z - triangle[0].z) - (rayPos.z - triangle[0].z) * (triangle[1].y - triangle[0].y),
            .y = (rayPos.z - triangle[0].z) * (triangle[1].x - triangle[0].x) - (rayPos.x - triangle[0].x) * (triangle[1].z - triangle[0].z),
            .z = (rayPos.x - triangle[0].x) * (triangle[1].y - triangle[0].y) - (rayPos.y - triangle[0].y) * (triangle[1].x - triangle[0].x),
        };

    const float v = det * (rayDir.x * qvec.x + rayDir.y * qvec.y + rayDir.z * qvec.z);

    if (v < 0.0f || u + v > 1.0f)
    {
        return false;
    }

    const float t = det * (
        (triangle[2].x - triangle[0].x) * qvec.x +
        (triangle[2].y - triangle[0].y) * qvec.y +
        (triangle[2].z - triangle[0].z) * qvec.z);

    if (t < epsilon)
    {
        return false;
    }

    pIntersect =
        {
            .x = rayPos.x + t * rayDir.x,
            .y = rayPos.y + t * rayDir.y,
            .z = rayPos.z + t * rayDir.z,
        };

    return true;
}
