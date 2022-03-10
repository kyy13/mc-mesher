// mc-mesher
// Kyle J Burgess

#include "McmCommon.h"
#include "McmLookupTable.h"

#include <cstring>

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

bool mcmRayIntersectAABB(const Vector3<float>& minB, const Vector3<float>& maxB, const Vector3<float>& rayPos, const Vector3<float>& rayDir, Vector3<float>& pIntersect)
{
    constexpr int RIGHT = 0;
    constexpr int LEFT = 1;
    constexpr int MIDDLE = 2;

    bool inside = true;
    uint8_t quadrant[3];
    size_t whichPlane = 0;
    float maxT[3];
    float candidatePlane[3];

    // x
    if (rayPos.x < minB.x)
    {
        quadrant[0] = LEFT;
        candidatePlane[0] = minB.x;
        inside = false;
    }
    else if (rayPos.x > maxB.x)
    {
        quadrant[0] = RIGHT;
        candidatePlane[0] = maxB.x;
        inside = false;
    }
    else
    {
        quadrant[0] = MIDDLE;
    }

    // y
    if (rayPos.y < minB.y)
    {
        quadrant[1] = LEFT;
        candidatePlane[1] = minB.y;
        inside = false;
    }
    else if (rayPos.y > maxB.y)
    {
        quadrant[1] = RIGHT;
        candidatePlane[1] = maxB.y;
        inside = false;
    }
    else
    {
        quadrant[1] = MIDDLE;
    }

    // z
    if (rayPos.z < minB.z)
    {
        quadrant[2] = LEFT;
        candidatePlane[2] = minB.z;
        inside = false;
    }
    else if (rayPos.z > maxB.z)
    {
        quadrant[2] = RIGHT;
        candidatePlane[2] = maxB.z;
        inside = false;
    }
    else
    {
        quadrant[2] = MIDDLE;
    }

    // inside AABB
    if (inside)
    {
        pIntersect = rayPos;
        return true;
    }

    // x
    if (quadrant[0] != MIDDLE && rayDir.x != 0.0f)
    {
        maxT[0] = (candidatePlane[0] - rayPos.x) / rayDir.x;
    }
    else
    {
        maxT[0] = -1.0f;
    }

    // y
    if (quadrant[1] != MIDDLE && rayDir.y != 0.0f)
    {
        maxT[1] = (candidatePlane[1] - rayPos.y) / rayDir.y;
    }
    else
    {
        maxT[1] = -1.0f;
    }

    // z
    if (quadrant[2] != MIDDLE && rayDir.z != 0.0f)
    {
        maxT[2] = (candidatePlane[2] - rayPos.z) / rayDir.z;
    }
    else
    {
        maxT[2] = -1.0f;
    }

    // largest t
    for (size_t i = 1; i < 3; ++i)
    {
        if (maxT[whichPlane] < maxT[i])
        {
            whichPlane = i;
        }
    }

    if (maxT[whichPlane] < 0.0f)
    {
        return false;
    }

    // x
    if (whichPlane != 0)
    {
        pIntersect.x = rayPos.x + maxT[whichPlane] * rayDir.x;

        if (pIntersect.x < minB.x || pIntersect.x > maxB.x)
        {
            return false;
        }
    }
    else
    {
        pIntersect.x = candidatePlane[0];
    }

    // y
    if (whichPlane != 1)
    {
        pIntersect.y = rayPos.y + maxT[whichPlane] * rayDir.y;

        if (pIntersect.y < minB.y || pIntersect.y > maxB.y)
        {
            return false;
        }
    }
    else
    {
        pIntersect.y = candidatePlane[1];
    }

    // z
    if (whichPlane != 2)
    {
        pIntersect.z = rayPos.z + maxT[whichPlane] * rayDir.z;

        if (pIntersect.z < minB.z || pIntersect.z > maxB.z)
        {
            return false;
        }
    }
    else
    {
        pIntersect.z = candidatePlane[2];
    }

    return true;
}
