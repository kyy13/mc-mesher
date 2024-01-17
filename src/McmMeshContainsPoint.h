// mc-mesher
// Kyle J Burgess

#ifndef MCM_MESH_CONTINS_POINT
#define MCM_MESH_CONTINS_POINT

#include "mc_mesher.h"
#include "McmGeometry.h"
#include "McmLookupTable.h"
#include "McmMeshIntersectSegment.h"

#include <cmath>
#include <limits>

template<class T, bool EDGE_LERP>
McmResult mcmMeshContainsPoint(const T* data, const uint32_t _dataSize[3], T isoLevel, const float _point[3])
{
    Vector3<uint32_t> dataSize(_dataSize);
    Vector3<float> point(_point);

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

    // Outside AABB
    if (point.x < minB.x ||
        point.y < minB.y ||
        point.z < minB.z ||
        point.x > maxB.x ||
        point.y > maxB.y ||
        point.z > maxB.z)
    {
        return MCM_FAILURE;
    }

    Vector3<uint32_t> umin = Vector3<uint32_t>(
            static_cast<uint32_t>(point.x),
            static_cast<uint32_t>(point.y),
            static_cast<uint32_t>(point.z));

    if (umin.x == dataSize.x - 1)
    { --umin.z; }
    if (umin.y == dataSize.y - 1)
    { --umin.y; }
    if (umin.z == dataSize.z - 1)
    { --umin.x; }

    const uint32_t mem_w = dataSize.x;
    const uint32_t mem_wh = dataSize.x * dataSize.y;

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

    if (caseIndex == 0x00)
    {
        return MCM_FAILURE;
    }

    if (caseIndex == 0xff)
    {
        return MCM_SUCCESS;
    }

    // Voxel is partially filled

    // Pick a corner that is < isoLevel (outside) and draw
    // a segment to it.

    Vector3<float> origin = Vector3<float>(
            static_cast<float>(umin.x),
            static_cast<float>(umin.y),
            static_cast<float>(umin.z));

    float pOutside[3];

    for (uint32_t i = 0; i != 8; ++i)
    {
        if (corners[i] < isoLevel)
        {
            pOutside[0] = origin.x + LookupTable::UnitCube[i].x;
            pOutside[1] = origin.y + LookupTable::UnitCube[i].y;
            pOutside[2] = origin.z + LookupTable::UnitCube[i].z;
            break;
        }
    }

    // If the point intersects surface to reach pOutside, then it is inside the surface
    float pIntersect[3];
    return mcmMeshIntersectSegment<T, EDGE_LERP>(data, _dataSize, isoLevel, _point, pOutside, pIntersect);
}

#endif
