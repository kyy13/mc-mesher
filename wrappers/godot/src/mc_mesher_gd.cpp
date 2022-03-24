// mc-mesher
// Kyle J Burgess

#include "mc_mesher_gd.h"
#include "defs.hpp"

extern "C" void GDN_EXPORT godot_gdnative_init(godot_gdnative_init_options *o)
{
    godot::Godot::gdnative_init(o);
}

extern "C" void GDN_EXPORT godot_gdnative_terminate(godot_gdnative_terminate_options *o)
{
    godot::Godot::gdnative_terminate(o);
}

extern "C" void GDN_EXPORT godot_nativescript_init(void *handle)
{
    godot::Godot::nativescript_init(handle);
    godot::register_class<godot::McmMeshBuffer>();
}

namespace godot
{
    McmMeshBuffer::McmMeshBuffer()
    {

    }

    McmMeshBuffer::~McmMeshBuffer()
    {

    }

    void McmMeshBuffer::_register_methods()
    {
        //register_method("_process", &McmMeshBuffer::_process);
    }
}
