// mc-mesher
// Kyle J Burgess

#ifndef MCM_GODOT_WRAPPER
#define MCM_GODOT_WRAPPER

#include "mc_mesher.h"
#include "Godot.hpp"
#include "Ref.hpp"
#include "ArrayMesh.hpp"
#include "PoolArrays.hpp"
#include "Node.hpp"

// Godot required methods
extern "C"
{
    void GDN_EXPORT godot_gdnative_init(godot_gdnative_init_options *o);

    void GDN_EXPORT godot_gdnative_terminate(godot_gdnative_terminate_options *o);

    void GDN_EXPORT godot_gdnative_singleton();

    void GDN_EXPORT godot_nativescript_init(void *handle);
}

namespace godot
{
    class McmMeshBuffer : public Node
    {
        GODOT_CLASS(McmMeshBuffer, Node);
    public:

        // Constructor (can't use Godot objects here, see _init)
        McmMeshBuffer();

        // Destructor
        ~McmMeshBuffer();

        // Generate a marching cubes mesh with face normals, and store the results in an McmMeshBuffer
        McmResult generateMesh(
            Ref<ArrayMesh>                arrayMesh,      // Godot array mesh
            PoolRealArray                 data,           // 3D field of scalar floating-point values as a contiguous array
            PoolIntArray                  dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
            PoolIntArray                  meshOrigin,     // Origin of the mesh to generate (in cubes) within the 3D field
            PoolIntArray                  meshSize,       // Size of the mesh to generate (in cubes) within the 3D field
            float                         isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
            bool                          vertexNormals); // Set to true to use vertex normals instead of face normals

        // Intersect a scalar field with a ray (gives the same results as mesh-ray intersection except faster, and the mesh does not need to be generated)
        // If an intersection occurs, then mcmRayIntersectMesh returns MCM_SUCCESS, and the point of intersection is set,
        // otherwise, mcmRayIntersectMesh returns MCM_NO_INTERSECTION
        McmResult rayIntersectVirtualMesh(
            PoolRealArray                 data,           // 3D field of scalar floating-point values as a contiguous array
            PoolIntArray                  dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
            float                         isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
            PoolRealArray                 rayPos,         // Starting point of the ray
            PoolRealArray                 rayDir,         // Direction of the ray (does not need to be normalized)
            PoolRealArray                 pIntersect);    // The point of intersection if an intersection occurred

        // Godot method for binding class methods
        static void _register_methods();

        // Godot init method (can use Godot objects here)
        void _init();

        // Godot update method
        void _process(float delta);

    protected:

        ::McmMeshBuffer* m_buffer;

        static void flipIndices(uint32_t* arr, uint32_t n);
    };
}

#endif
