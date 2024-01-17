// mc-mesher
// Kyle J Burgess

#ifndef MCM_VECTOR_H
#define MCM_VECTOR_H

#include <cmath>
#include <cstring>

struct Vector2
{
    float x;
    float y;
};

template<class T>
struct Vector3
{
    using type = T;

    T x;
    T y;
    T z;

    Vector3()
    {}

    Vector3(T _x, T _y, T _z) : x(_x), y(_y), z(_z)
    {}

    Vector3(const T ptr[3])
    {
        memcpy(&x, ptr, 3 * sizeof(T));
    }

    constexpr Vector3 operator+(const Vector3& v) const
    {
        return Vector3(x + v.x, y + v.y, z + v.z);
    }

    constexpr Vector3 operator-(const Vector3& v) const
    {
        return Vector3(x - v.x, y - v.y, z - v.z);
    }

    constexpr Vector3 operator*(float k) const
    {
        return Vector3(k * x, k * y, k * z);
    }

    static constexpr Vector3 cross(const Vector3& a, const Vector3& b)
    {
        return Vector3(
            a.y * b.z - a.z * b.y,
            b.x * a.z - a.x * b.z,
            a.x * b.y - a.y * b.x);
    }

    void normalize()
    {
        static_assert(std::is_floating_point<T>::value, "cannot normalize an integral vector type!");

        T oneOverLen;

        if constexpr (sizeof(T) == 4u)
        {
            oneOverLen = 1.0f / sqrtf(x * x + y * y + z * z);
        }
        else
        {
            oneOverLen = 1.0 / sqrt(x * x + y * y + z * z);
        }

        x *= oneOverLen;
        y *= oneOverLen;
        z *= oneOverLen;
    }

    [[nodiscard]]
    float magnitude() const
    {
        return sqrtf(x * x + y * y + z * z);
    }
};

template<class T>
constexpr Vector3<T> operator*(T k, const Vector3<T>& v)
{
    return Vector3(
        k * v.x,
        k * v.y,
        k * v.z);
}

#endif
