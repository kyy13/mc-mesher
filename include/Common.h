// MCmesher
// Kyle J Burgess

#ifndef MCM_COMMON_H
#define MCM_COMMON_H

#include "Vector.h"
#include <cstdint>

// Generate a Case Index for a cube
uint8_t mcmComputeCaseIndex(
    const float                     corners[8],
    float                           isoLevel);

// Compute a cache key for an edge
uint32_t mcmComputeEdgeCacheKey(
    uint32_t                        memPos,
    uint32_t                        memDy,
    uint32_t                        memDz,
    uint32_t                        vertexData);

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
