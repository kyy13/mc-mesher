// mc-mesher
// Kyle J Burgess

#ifndef MCM_GEOMETRY_H
#define MCM_GEOMETRY_H

#include "McmVector.h"
#include "McmLookupTable.h"

#include <cstdint>

// Determine if ray intersects triangle
bool     mcmRayIntersectTriangle(
    Vector3<float>                  rayPos,
    Vector3<float>                  rayDir,
    const Vector3<float>            triangle[3],
    float                           pIntersect[3]);

// Determine if ray intersects AABB
bool     mcmRayIntersectAABB(
    const Vector3<float>&           minB,
    const Vector3<float>&           maxB,
    const Vector3<float>&           rayPos,
    const Vector3<float>&           rayDir,
    float                           pIntersect[3]);

// Generate a Case Index for a cube
template<class T>
uint8_t mcmComputeCaseIndex(
    const T corners[8],
    T isoLevel)
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

// Compute single cube geometry
template<class T, bool EDGE_LERP>
uint32_t mcmComputeCaseGeometry(
    const T corners[8],
    T isoLevel,
    Vector3<float> vertices[12])
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
            float k;

            if constexpr (EDGE_LERP)
            {
                k = (isoLevel - static_cast<float>(corners[endpointIndex[0]])) /
                    static_cast<float>(corners[endpointIndex[1]] - corners[endpointIndex[0]]);
            }
            else
            {
                k = 0.5f;
            }

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



#endif
