// MCmesher
// Kyle J Burgess

#ifndef MC_MESHER_VECTOR_H
#define MC_MESHER_VECTOR_H

#include <cmath>

struct Vector2
{
    float x;
    float y;
};

struct Vector3
{
    float x;
    float y;
    float z;

    constexpr Vector3 operator+(const Vector3& v) const
    {
        return {
            .x = x + v.x,
            .y = y + v.y,
            .z = z + v.z,
        };
    }

    constexpr Vector3 operator-(const Vector3& v) const
    {
        return {
            .x = x - v.x,
            .y = y - v.y,
            .z = z - v.z,
        };
    }

    constexpr Vector3 operator*(float k) const
    {
        return {
            .x = k * x,
            .y = k * y,
            .z = k * z,
        };
    }

    static constexpr Vector3 cross(const Vector3& a, const Vector3& b)
    {
        return {
            .x = a.y * b.z - a.z * b.y,
            .y = b.x * a.z - a.x * b.z,
            .z = a.x * b.y - a.y * b.x,
        };
    }

    void normalize();
};

constexpr Vector3 operator*(float k, const Vector3& v)
{
    return {
        .x = k * v.x,
        .y = k * v.y,
        .z = k * v.z,
    };
}

#endif
