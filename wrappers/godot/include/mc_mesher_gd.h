// mc-mesher
// Kyle J Burgess

#ifndef MCM_GODOT_WRAPPER
#define MCM_GODOT_WRAPPER

#include "mc_mesher.h"
#include "godot.hpp"
#include "object.hpp"
#include "ref.hpp"
#include "array_mesh.hpp"

namespace godot
{
    void register_mcmesher_types();

    void unregister_mcmesher_types();

    class McmMeshBuffer : public Object
    {
        GDCLASS(McmMeshBuffer, Object);
    public:

        McmMeshBuffer();

        ~McmMeshBuffer();

        McmResult generateMesh(
            Ref<ArrayMesh>                arrayMesh,      // Godot array mesh
            Ref<PoolFloat32Array>         data,           // 3D field of scalar floating-point values as a contiguous array
            Vector3i                      dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
            Vector3i                      meshOrigin,     // Origin of the mesh to generate (in cubes) within the 3D field
            Vector3i                      meshSize,       // Size of the mesh to generate (in cubes) within the 3D field
            float                         isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
            bool                          vertexNormals); // Set to true to use vertex normals instead of face normals

    protected:

        // Godot method for binding class methods
        static void _bind_methods();

        McmMeshBuffer* m_buffer;
    };
}

#endif
