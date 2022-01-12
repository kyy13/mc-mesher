// MCmesher
// Kyle J Burgess

#include "Vector.h"

void Vector3::normalize()
{
    float oneOverLen = 1.0f / sqrtf(x * x + y * y + z * z);

    x *= oneOverLen;
    y *= oneOverLen;
    z *= oneOverLen;
}
