// mc-mesher
// Kyle J Burgess

#ifndef MCM_MESH_H
#define MCM_MESH_H

#include <vector>

struct McmMeshBuffer
{
    std::vector<Vector3<float>> vertices;
    std::vector<Vector3<float>> normals;
    std::vector<uint32_t> indices;
};

#endif
