// MCmesher
// Kyle J Burgess

#ifndef MESH_H
#define MESH_H

#include <vector>

struct McmMeshBuffer
{
    std::vector<Vector3<float>> vertices;
    std::vector<Vector3<float>> normals;
    std::vector<uint32_t> indices;
};

#endif
