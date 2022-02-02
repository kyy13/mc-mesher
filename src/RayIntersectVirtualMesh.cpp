// MCmesher
// Kyle J Burgess

#include "MCmesher.h"
#include "Common.h"
#include "LookupTable.h"

#include <cmath>
#include <limits>

McmResult mcmRayIntersectVirtualMesh(const float* data, Vector3<uint32_t> dataSize, float isoLevel, Vector3<float> rayPos, Vector3<float> rayDir, Vector3<float>& pIntersect)
{
    const uint32_t mem_w = dataSize.x;
    const uint32_t mem_wh = dataSize.x * dataSize.y;

    /*     p3 ------ p7
     *    / |       /|
     *   /  |      / |      z
     *  p2 ------ p6 |      |
     *  |  p0 ----|- p4     o----> y
     *  | /       | /      /
     *  |/        |/      x
     *  p1 ------ p5
     */

    // Find the bounding cube that contains the point
    Vector3<float> min =
        {
            floorf(rayPos.x),
            floorf(rayPos.y),
            floorf(rayPos.z),
        };

    Vector3<float> max =
        {
            ceilf(rayPos.x),
            ceilf(rayPos.y),
            ceilf(rayPos.z),
        };

    // If the point lies directly on the face, then
    // we want the cube in the same direction as the axis of movement
    if (max.x == min.x)
    {
        if (rayDir.x >= 0.0f)
            max.x += 1.0f;
        else
            min.x -= 1.0f;
    }

    if (max.y == min.y)
    {
        if (rayDir.y >= 0.0f)
            max.y += 1.0f;
        else
            min.y -= 1.0f;
    }

    if (max.z == min.z)
    {
        if (rayDir.z >= 0.0f)
            max.z += 1.0f;
        else
            min.z -= 1.0f;
    }

    // Check if out of bounds

    // TODO: this should really move the ray to the mesh start
    //       and only exit if the ray is both outside and moving away...
    if (min.x < 0.0f || min.y < 0.0f || min.z < 0.0f)
    {
        return McmResult::MCM_NO_INTERSECTION;
    }

    const Vector3<uint32_t> umin =
        {
            .x = static_cast<uint32_t>(min.x),
            .y = static_cast<uint32_t>(min.y),
            .z = static_cast<uint32_t>(min.z),
        };

    if ((umin.x >= dataSize.x - 2) ||
        (umin.y >= dataSize.y - 2) ||
        (umin.z >= dataSize.z - 2))
    {
        return McmResult::MCM_NO_INTERSECTION;
    }

    // Get the origin of the cube, so we can determine the contents of the cube
    uint32_t voxelIndex = umin.x + mem_w * umin.y + mem_wh * umin.z;
    const float* voxel = &data[voxelIndex];

    // Get case index
    const float corners[8] =
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

    // Voxel is filled
    if (caseIndex == 0xff)
    {
        pIntersect = rayPos;
        return MCM_SUCCESS;
    }

    // Voxel is partially filled
    if (caseIndex != 0x00)
    {
        Vector3<float> vertices[12];

        uint32_t vertexCount = mcmComputeCaseGeometry(corners, isoLevel, vertices);

        for (uint32_t i = 0; i != vertexCount; i += 3)
        {
            vertices[i    ] = vertices[i    ] + min;
            vertices[i + 1] = vertices[i + 1] + min;
            vertices[i + 2] = vertices[i + 2] + min;

            if (mcmRayIntersectTriangle(rayPos, rayDir, &vertices[i], pIntersect))
            {
                return MCM_SUCCESS;
            }
        }
    }

    // Voxel is unfilled || partially filled, but the ray missed all the triangles

    // If there is no movement, then we are not going to hit anything in an unfilled box
    if (rayDir.x == 0.0f && rayDir.y == 0.0f && rayDir.z == 0.0f)
    {
        return MCM_NO_INTERSECTION;
    }

    // Step along ray to find next cube
    float t[3];

    const auto pos = reinterpret_cast<const float*>(&rayPos);
    const auto dir = reinterpret_cast<const float*>(&rayDir);
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

    // Query next cubes
    constexpr float epsilon = 1e-7f;

    tMin += epsilon;
    rayPos.x += tMin * rayDir.x;
    rayPos.y += tMin * rayDir.y;
    rayPos.z += tMin * rayDir.z;

    return mcmRayIntersectVirtualMesh(data, dataSize, isoLevel, rayPos, rayDir, pIntersect);
}
