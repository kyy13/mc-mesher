// mc-mesher
// Kyle J Burgess

#include "McmGeometry.h"
#include <cstring>

bool mcmRayIntersectTriangle(Vector3<float> rayPos, Vector3<float> rayDir, const Vector3<float> triangle[3], float pIntersect[3])
{
    constexpr float epsilon = 0.0000001f;

    const Vector3<float> pvec = Vector3<float>(
            rayDir.y * (triangle[2].z - triangle[0].z) - rayDir.z * (triangle[2].y - triangle[0].y),
            rayDir.z * (triangle[2].x - triangle[0].x) - rayDir.x * (triangle[2].z - triangle[0].z),
            rayDir.x * (triangle[2].y - triangle[0].y) - rayDir.y * (triangle[2].x - triangle[0].x));

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

    const Vector3<float> qvec = Vector3<float>(
            (rayPos.y - triangle[0].y) * (triangle[1].z - triangle[0].z) - (rayPos.z - triangle[0].z) * (triangle[1].y - triangle[0].y),
            (rayPos.z - triangle[0].z) * (triangle[1].x - triangle[0].x) - (rayPos.x - triangle[0].x) * (triangle[1].z - triangle[0].z),
            (rayPos.x - triangle[0].x) * (triangle[1].y - triangle[0].y) - (rayPos.y - triangle[0].y) * (triangle[1].x - triangle[0].x));

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

    pIntersect[0] = rayPos.x + t * rayDir.x;
    pIntersect[1] = rayPos.y + t * rayDir.y;
    pIntersect[2] = rayPos.z + t * rayDir.z;

    return true;
}

bool mcmRayIntersectAABB(const Vector3<float>& minB, const Vector3<float>& maxB, const Vector3<float>& rayPos, const Vector3<float>& rayDir, float pIntersect[3])
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
        memcpy(pIntersect, &rayPos, sizeof(rayPos));
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
        pIntersect[0] = rayPos.x + maxT[whichPlane] * rayDir.x;

        if (pIntersect[0] < minB.x || pIntersect[0] > maxB.x)
        {
            return false;
        }
    }
    else
    {
        pIntersect[0] = candidatePlane[0];
    }

    // y
    if (whichPlane != 1)
    {
        pIntersect[1] = rayPos.y + maxT[whichPlane] * rayDir.y;

        if (pIntersect[1] < minB.y || pIntersect[1] > maxB.y)
        {
            return false;
        }
    }
    else
    {
        pIntersect[1] = candidatePlane[1];
    }

    // z
    if (whichPlane != 2)
    {
        pIntersect[2] = rayPos.z + maxT[whichPlane] * rayDir.z;

        if (pIntersect[2] < minB.z || pIntersect[2] > maxB.z)
        {
            return false;
        }
    }
    else
    {
        pIntersect[2] = candidatePlane[2];
    }

    return true;
}
