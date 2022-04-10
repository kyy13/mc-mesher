// mc-mesher
// Kyle J Burgess

#ifndef MCM_MESH_INTERSECT_SEGMENT_H
#define MCM_MESH_INTERSECT_SEGMENT_H

#include "mc_mesher.h"
#include "McmGeometry.h"
#include "McmLookupTable.h"

#include <cmath>
#include <limits>

template<class T, bool EDGE_LERP>
McmResult _mcmMeshIntersectSegment(const T* data, Vector3<uint32_t> dataSize, T isoLevel, Vector3<float> segPos, Vector3<float> segDir, float& d)
{
    constexpr float epsilon = 1e-7f;

    const uint32_t mem_w = dataSize.x;
    const uint32_t mem_wh = dataSize.x * dataSize.y;

    // March ray to AABB if outside entire data set
    const Vector3<float> minB =
        {
            0.0f,
            0.0f,
            0.0f,
        };

    const Vector3<float> maxB =
        {
            static_cast<float>(dataSize.x - 1),
            static_cast<float>(dataSize.y - 1),
            static_cast<float>(dataSize.z - 1),
        };

    Vector3<float> pIntersect;
    if (!mcmRayIntersectAABB(minB, maxB, segPos, segDir, pIntersect))
    {
        return MCM_NO_INTERSECTION;
    }

    Vector3<float> delta = segPos - pIntersect;
    d -= delta.magnitude();

    if (d < 0.0f)
    {
        return MCM_NO_INTERSECTION;
    }

    segPos = pIntersect;

    /*     p3 ------ p7
     *    /|        /|
     *   / |       / |      z
     *  p2 ------ p6 |      |
     *  |  p0 ----|- p4     o----> y
     *  | /       | /      /
     *  |/        |/      x
     *  p1 ------ p5
     */

    // Find the bounding cube that contains the point
    Vector3<float> min =
        {
            floorf(segPos.x),
            floorf(segPos.y),
            floorf(segPos.z),
        };

    Vector3<float> max =
        {
            ceilf(segPos.x),
            ceilf(segPos.y),
            ceilf(segPos.z),
        };

    // If the point lies directly on the face, then
    // we want the cube in the same direction as the axis of movement
    if (max.x == min.x)
    {
        if (segDir.x >= 0.0f)
            max.x += 1.0f;
        else
            min.x -= 1.0f;
    }

    if (max.y == min.y)
    {
        if (segDir.y >= 0.0f)
            max.y += 1.0f;
        else
            min.y -= 1.0f;
    }

    if (max.z == min.z)
    {
        if (segDir.z >= 0.0f)
            max.z += 1.0f;
        else
            min.z -= 1.0f;
    }

    // Convert to unsigned, correcting small errors in boundary collision from mcmRayIntersectAABB
    Vector3<uint32_t> umin =
        {
            .x = (min.x < 0.0f) ? 0 : static_cast<uint32_t>(min.x),
            .y = (min.y < 0.0f) ? 0 : static_cast<uint32_t>(min.y),
            .z = (min.z < 0.0f) ? 0 : static_cast<uint32_t>(min.z),
        };

    if (min.x > static_cast<float>(dataSize.x - 2))
    { umin.x = dataSize.x - 2; }
    if (min.y > static_cast<float>(dataSize.y - 2))
    { umin.y = dataSize.y - 2; }
    if (min.z > static_cast<float>(dataSize.z - 2))
    { umin.z = dataSize.z - 2; }

    // Get the origin of the cube, so we can determine the contents of the cube
    uint32_t voxelIndex = umin.x + mem_w * umin.y + mem_wh * umin.z;
    const T* voxel = &data[voxelIndex];

    // Get case index
    const T corners[8] =
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

    uint8_t caseIndex = mcmComputeCaseIndex(corners, isoLevel);

    // Voxel is partially filled
    if (caseIndex != 0x00 && caseIndex != 0xff)
    {
        Vector3<float> vertices[12];

        uint32_t vertexCount = mcmComputeCaseGeometry<T, EDGE_LERP>(corners, isoLevel, vertices);

        for (uint32_t i = 0; i != vertexCount; i += 3)
        {
            vertices[i    ] = vertices[i    ] + min;
            vertices[i + 1] = vertices[i + 1] + min;
            vertices[i + 2] = vertices[i + 2] + min;

            if (mcmRayIntersectTriangle(segPos, segDir, &vertices[i], pIntersect))
            {
                delta = segPos - pIntersect;
                d -= delta.magnitude();

                return (d >= 0.0f)
                    ? MCM_SUCCESS
                    : MCM_NO_INTERSECTION;
            }
        }
    }

    // Voxel is filled || unfilled || partially filled, but the ray missed all the triangles

    // If there is no movement, then we are not going to hit anything in an unfilled box
    if (segDir.x == 0.0f && segDir.y == 0.0f && segDir.z == 0.0f)
    {
        return MCM_NO_INTERSECTION;
    }

    // Step along ray to find next cube
    float t[3];

    const auto pos = reinterpret_cast<const float*>(&segPos);
    const auto dir = reinterpret_cast<const float*>(&segDir);
    const auto pMin = reinterpret_cast<const float*>(&min);
    const auto pMax = reinterpret_cast<const float*>(&max);

    // Test parameter t to face
    for (uint32_t i = 0; i != 3; ++i)
    {
        if (dir[i] > 0.0f)
        {
            t[i] = (pMax[i] - pos[i]) / dir[i];
        }

        else if (dir[i] < 0.0f)
        {
            t[i] = (pMin[i] - pos[i]) / dir[i];
        }
        else
        {
            t[i] = std::numeric_limits<float>::max();
        }
    }

    // Find smallest magnitude of t
    float tMin = t[0];
    if (fabsf(t[1]) < fabsf(tMin)) tMin = t[1];
    if (fabsf(t[2]) < fabsf(tMin)) tMin = t[2];

    delta = segPos;

    // Query next cubes
    tMin += epsilon;
    segPos.x += tMin * segDir.x;
    segPos.y += tMin * segDir.y;
    segPos.z += tMin * segDir.z;

    delta = delta - segPos;
    d -= delta.magnitude();

    if (d < 0.0f)
    {
        return MCM_NO_INTERSECTION;
    }

    return _mcmMeshIntersectSegment<T, EDGE_LERP>(data, dataSize, isoLevel, segPos, segDir, d);
}

template<class T, bool EDGE_LERP>
McmResult mcmMeshIntersectSegment(const T* data, Vector3<uint32_t> dataSize, T isoLevel, Vector3<float> segPos, Vector3<float> segEnd, Vector3<float>& pIntersect)
{
    Vector3 delta = segEnd - segPos;

    float dTotal = delta.magnitude();
    float d = dTotal;

    McmResult r = _mcmMeshIntersectSegment(data, dataSize, isoLevel, segPos, delta, d);

    if (r == MCM_SUCCESS)
    {
        if (d >= 0.0f)
        {
            delta.normalize();
            pIntersect = segPos + (dTotal - d) * delta;
            return MCM_SUCCESS;
        }

        return MCM_NO_INTERSECTION;
    }

    return r;
}

#endif
