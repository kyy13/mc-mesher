// mc-mesher
// Kyle J Burgess

#ifndef MCM_COMMON_H
#define MCM_COMMON_H

#include "McmVector.h"
#include <cstdint>

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
uint32_t mcmComputeCaseGeometry(
    const float                     corners[8],
    float                           isoLevel,
    Vector3<float>                  vertices[12]);

// Determine if ray intersects triangle
bool     mcmRayIntersectTriangle(
    Vector3<float>                  rayPos,
    Vector3<float>                  rayDir,
    const Vector3<float>            triangle[3],
    Vector3<float>&                 pIntersect);

// Determine if ray intersects AABB
bool     mcmRayIntersectAABB(
    const Vector3<float>&           minB,
    const Vector3<float>&           maxB,
    const Vector3<float>&           rayPos,
    const Vector3<float>&           rayDir,
    Vector3<float>&                 pIntersect);

#endif
