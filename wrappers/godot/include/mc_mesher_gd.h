// mc-mesher
// Kyle J Burgess

#ifndef MCM_GODOT_WRAPPER
#define MCM_GODOT_WRAPPER

#include "mc_mesher.h"
#include "Godot.hpp"
#include "Ref.hpp"
#include "ArrayMesh.hpp"
#include "PoolArrays.hpp"

namespace godot
{
    class McmMeshBuffer : public Object
    {
        GODOT_CLASS(McmMeshBuffer, Object);
    public:

        McmMeshBuffer();

        ~McmMeshBuffer();

        McmResult generateMesh(
            Ref<ArrayMesh>                arrayMesh,      // Godot array mesh
            Ref<PoolRealArray>            data,           // 3D field of scalar floating-point values as a contiguous array
            Ref<PoolIntArray>             dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
            Ref<PoolIntArray>             meshOrigin,     // Origin of the mesh to generate (in cubes) within the 3D field
            Ref<PoolIntArray>             meshSize,       // Size of the mesh to generate (in cubes) within the 3D field
            float                         isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
            bool                          vertexNormals); // Set to true to use vertex normals instead of face normals

    // Godot method for binding class methods
    static void _register_methods();

    protected:
        McmMeshBuffer* m_buffer;
    };
}

#endif
