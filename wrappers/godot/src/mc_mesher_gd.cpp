// mc-mesher
// Kyle J Burgess

#include "mc_mesher_gd.h"
#include "defs.hpp"

void GDN_EXPORT godot_gdnative_init(godot_gdnative_init_options *o)
{
    godot::Godot::gdnative_init(o);
}

void GDN_EXPORT godot_gdnative_terminate(godot_gdnative_terminate_options *o)
{
    godot::Godot::gdnative_terminate(o);
}

void GDN_EXPORT godot_nativescript_init(void *handle)
{
    godot::Godot::nativescript_init(handle);
    godot::register_class<godot::McmMeshBuffer>();
}

namespace godot
{
    McmMeshBuffer::McmMeshBuffer()
    {
        m_buffer = mcmCreateMeshBuffer();
    }

    McmMeshBuffer::~McmMeshBuffer()
    {
        mcmDeleteMeshBuffer(m_buffer);
    }

    McmResult McmMeshBuffer::generateMesh(
        Ref<ArrayMesh>                arrayMesh,      // Godot array mesh
        PoolRealArray                 data,           // 3D field of scalar floating-point values as a contiguous array
        PoolIntArray                  dataSize,       // Size of 3D field x, y, and z axis (in vertices) where field array length is x * y * z
        PoolIntArray                  meshOrigin,     // Origin of the mesh to generate (in cubes) within the 3D field
        PoolIntArray                  meshSize,       // Size of the mesh to generate (in cubes) within the 3D field
        float                         isoLevel,       // The ISO level for the surface (under ISO = inside the volume, over ISO = outside the volume)
        bool                          vertexNormals)  // Set to true to use vertex normals instead of face normals
    {
        McmResult result = MCM_SUCCESS;

        const ::Vector3<uint32_t> _dataSize =
            {
                static_cast<uint32_t>(dataSize[0]),
                static_cast<uint32_t>(dataSize[1]),
                static_cast<uint32_t>(dataSize[2]),
            };

        const ::Vector3<uint32_t> _meshOrigin =
            {
                static_cast<uint32_t>(meshOrigin[0]),
                static_cast<uint32_t>(meshOrigin[1]),
                static_cast<uint32_t>(meshOrigin[2]),
            };

        const ::Vector3<uint32_t> _meshSize =
            {
                static_cast<uint32_t>(meshSize[0]),
                static_cast<uint32_t>(meshSize[1]),
                static_cast<uint32_t>(meshSize[2]),
            };

        if (vertexNormals)
        {
            result = mcmGenerateMeshVN(m_buffer, data.read().ptr(), _dataSize, _meshOrigin, _meshSize, isoLevel);
        }
        else
        {
            result = mcmGenerateMeshFN(m_buffer, data.read().ptr(), _dataSize, _meshOrigin, _meshSize, isoLevel);
        }

        if (result != MCM_SUCCESS)
        {
            return result;
        }

        Array arrays;

        arrays.resize(ArrayMesh::ARRAY_MAX);

        // Write Vertices

        PoolVector3Array vertices;

        uint32_t vertexCount = mcmCountVertices(m_buffer);

        vertices.resize(vertexCount);

        mcmCopyVertices(m_buffer, reinterpret_cast<::Vector3<float>*>(vertices.write().ptr()));

        arrays[ArrayMesh::ARRAY_VERTEX] = vertices;

        // Write Normals

        PoolVector3Array normals;

        uint32_t normalCount = mcmCountNormals(m_buffer);

        normals.resize(normalCount);

        mcmCopyNormals(m_buffer, reinterpret_cast<::Vector3<float>*>(normals.write().ptr()));

        arrays[ArrayMesh::ARRAY_NORMAL] = normals;

        // Write Indices

        PoolIntArray indices;

        uint32_t indexCount = mcmCountIndices(m_buffer);

        indices.resize(indexCount);

        mcmCopyIndices(m_buffer, reinterpret_cast<uint32_t*>(indices.write().ptr()));

        arrays[ArrayMesh::ARRAY_INDEX] = indices;

        // Assign to ArrayMesh

        arrayMesh->add_surface_from_arrays(Mesh::PRIMITIVE_TRIANGLES, arrays);

        return result;
    }

    void McmMeshBuffer::_register_methods()
    {
        register_method("_process", &McmMeshBuffer::_process);
        register_method("generateMesh", &McmMeshBuffer::generateMesh);
    }

    void McmMeshBuffer::_init()
    {}

    void McmMeshBuffer::_process(float delta)
    {}
}
