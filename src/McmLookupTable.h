// mc-mesher
// Kyle J Burgess

#ifndef MCM_LOOKUP_TABLE_H
#define MCM_LOOKUP_TABLE_H

#include <McmVector.h>
#include <cstdint>

namespace LookupTable
{
    extern const Vector3<float> UnitCube[8];

    extern const uint32_t RegularCellClass[256];

    extern const uint32_t RegularCellData[256];

    extern const uint32_t RegularVertexData[3072];

    extern const uint32_t McmEdgeCacheLookup[64];
}

#endif
