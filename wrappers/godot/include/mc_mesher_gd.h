// mc-mesher
// Kyle J Burgess

#ifndef MCM_GODOT_WRAPPER
#define MCM_GODOT_WRAPPER

#include "godot.hpp"
#include "object.hpp"

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

    protected:

        // Godot method for binding class methods
        static void _bind_methods();
    };
}

#endif
