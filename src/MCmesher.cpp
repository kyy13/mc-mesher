// MCmesher
// Kyle J Burgess

#include "MCmesher.h"
#include "Mesh.h"
#include "LookupTable.h"

#include <stdexcept>
#include <vector>
#include <cstring>
#include <cmath>
#include <iostream>
#include <unordered_map>
#include <cassert>

Mesh* CreateMesh()
{
    return new Mesh();
}

void DeleteMesh(Mesh* mesh)
{
    delete mesh;
}

uint32_t CountVertices(const Mesh* mesh)
{
    return mesh->vertices.size();
}

const Vector3<float>* GetVertices(const Mesh* mesh)
{
    return mesh->vertices.data();
}

void CopyVertices(const Mesh* mesh, Vector3<float>* dst)
{
    memcpy(dst, mesh->vertices.data(), mesh->vertices.size() * sizeof(Vector3<float>));
}

uint32_t CountNormals(const Mesh* mesh)
{
    return mesh->normals.size();
}

const Vector3<float>* GetNormals(const Mesh* mesh)
{
    return mesh->normals.data();
}

void CopyNormals(const Mesh* mesh, Vector3<float>* dst)
{
    memcpy(dst, mesh->normals.data(), mesh->normals.size() * sizeof(Vector3<float>));
}

uint32_t CountIndices(const Mesh* mesh)
{
    return mesh->indices.size();
}

const uint32_t* GetIndices(const Mesh* mesh)
{
    return mesh->indices.data();
}

void CopyIndices(const Mesh* mesh, uint32_t* dst)
{
    memcpy(dst, mesh->indices.data(), mesh->indices.size() * sizeof(uint32_t));
}
