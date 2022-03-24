// mc-mesher
// Kyle J Burgess

#include "mc_mesher_gd.h"
#include "defs.hpp"
#include "class_db.hpp"

namespace godot
{
    void register_mcmesher_types()
    {
        ClassDB::register_class<McmMeshBuffer>();
    }

    void unregister_mcmesher_types()
    {}

    McmMeshBuffer::McmMeshBuffer()
    {

    }

    McmMeshBuffer::~McmMeshBuffer()
    {

    }

    void McmMeshBuffer::_bind_methods()
    {
        //register_method("_process", &McmMeshBuffer::_process);
    }
}

extern "C"
{
    GDNativeBool GDN_EXPORT mcmesher_library_init(const GDNativeInterface* p_interface, const GDNativeExtensionClassLibraryPtr p_library, GDNativeInitialization* r_initialization)
    {
        godot::GDExtensionBinding::InitObject init_obj(p_interface, p_library, r_initialization);

        init_obj.register_scene_initializer(godot::register_mcmesher_types);

        init_obj.register_scene_terminator(godot::unregister_mcmesher_types);

        return init_obj.init();
    }
}
