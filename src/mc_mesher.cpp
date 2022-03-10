// mc-mesher
// Kyle J Burgess

#include "mc_mesher.h"
#include "McmMeshBuffer.h"
#include "McmLookupTable.h"

#include <stdexcept>
#include <vector>
#include <cstring>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <cassert>

McmMeshBuffer* mcmCreateMeshBuffer()
{
    return new McmMeshBuffer();
}

void mcmDeleteMeshBuffer(McmMeshBuffer* mesh)
{
    delete mesh;
}

uint32_t mcmCountVertices(const McmMeshBuffer* mesh)
{
    return mesh->vertices.size();
}

const Vector3<float>* mcmGetVertices(const McmMeshBuffer* mesh)
{
    return mesh->vertices.data();
}

void mcmCopyVertices(const McmMeshBuffer* mesh, Vector3<float>* dst)
{
    memcpy(dst, mesh->vertices.data(), mesh->vertices.size() * sizeof(Vector3<float>));
}

uint32_t mcmCountNormals(const McmMeshBuffer* mesh)
{
    return mesh->normals.size();
}

const Vector3<float>* mcmGetNormals(const McmMeshBuffer* mesh)
{
    return mesh->normals.data();
}

void mcmCopyNormals(const McmMeshBuffer* mesh, Vector3<float>* dst)
{
    memcpy(dst, mesh->normals.data(), mesh->normals.size() * sizeof(Vector3<float>));
}

uint32_t mcmCountIndices(const McmMeshBuffer* mesh)
{
    return mesh->indices.size();
}

const uint32_t* mcmGetIndices(const McmMeshBuffer* mesh)
{
    return mesh->indices.data();
}

void mcmCopyIndices(const McmMeshBuffer* mesh, uint32_t* dst)
{
    memcpy(dst, mesh->indices.data(), mesh->indices.size() * sizeof(uint32_t));
}
