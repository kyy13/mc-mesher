// mc-mesher
// Kyle J Burgess

#include "mc_mesher.h"
#include "McmMeshBuffer.h"
#include "McmLookupTable.h"
#include "McmMeshContainsPoint.h"
#include "McmMeshIntersectRay.h"
#include "McmMeshIntersectSegment.h"
#include "McmGenerateMeshFN.h"
#include "McmGenerateMeshVN.h"
#include "McmVector.h"

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

const float* mcmGetVertices(const McmMeshBuffer* mesh)
{
    return reinterpret_cast<const float*>(mesh->vertices.data());
}

void mcmCopyVertices(const McmMeshBuffer* mesh, Vector3<float>* dst)
{
    memcpy(dst, mesh->vertices.data(), mesh->vertices.size() * sizeof(Vector3<float>));
}

uint32_t mcmCountNormals(const McmMeshBuffer* mesh)
{
    return mesh->normals.size();
}

const float* mcmGetNormals(const McmMeshBuffer* mesh)
{
    return reinterpret_cast<const float*>(mesh->normals.data());
}

void mcmCopyNormals(const McmMeshBuffer* mesh, float* dst)
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

McmResult mcmGenerateMesh(McmMeshBuffer* meshBuffer, const float* data, const uint32_t dataSize[3], const uint32_t meshOrigin[3], const uint32_t meshSize[3], float isoLevel, McmFlags flags)
{
    // Pick and generate template functions from flags
    if ((flags & MCM_FACE_NORMALS) == 0)
    {
        // Vertex Normals
        if ((flags & MCM_EDGE_CENTER) == 0)
        {
            // Edge Lerp
            if ((flags & MCM_WINDING_RHCS_CCW) == 0)
            {
                // Right-Handed CW
                return mcmGenerateMeshVN<float, true, true>(meshBuffer, data, dataSize, meshOrigin, meshSize, isoLevel);
            }
            else
            {
                // Right-Handed CCW
                return mcmGenerateMeshVN<float, false, true>(meshBuffer, data, dataSize, meshOrigin, meshSize, isoLevel);
            }
        }
        else
        {
            // Edge Center
            if ((flags & MCM_WINDING_RHCS_CCW) == 0)
            {
                // Right-Handed CW
                return mcmGenerateMeshVN<float, true, false>(meshBuffer, data, dataSize, meshOrigin, meshSize, isoLevel);
            }
            else
            {
                // Right-Handed CCW
                return mcmGenerateMeshVN<float, false, false>(meshBuffer, data, dataSize, meshOrigin, meshSize, isoLevel);
            }
        }
    }
    else
    {
        // Face Normals
        if ((flags & MCM_EDGE_CENTER) == 0)
        {
            // Edge Lerp
            if ((flags & MCM_WINDING_RHCS_CCW) == 0)
            {
                // Right-Handed CW
                return mcmGenerateMeshFN<float, true, true>(meshBuffer, data, dataSize, meshOrigin, meshSize, isoLevel);
            }
            else
            {
                // Right-Handed CCW
                return mcmGenerateMeshFN<float, false, true>(meshBuffer, data, dataSize, meshOrigin, meshSize, isoLevel);
            }
        }
        else
        {
            // Edge Center
            if ((flags & MCM_WINDING_RHCS_CCW) == 0)
            {
                // Right-Handed CW
                return mcmGenerateMeshFN<float, true, false>(meshBuffer, data, dataSize, meshOrigin, meshSize, isoLevel);
            }
            else
            {
                // Right-Handed CCW
                return mcmGenerateMeshFN<float, false, false>(meshBuffer, data, dataSize, meshOrigin, meshSize, isoLevel);
            }
        }
    }
}

McmResult mcmMeshContainsPoint(const float* data, const uint32_t dataSize[3], float isoLevel, const float point[3], McmFlags flags)
{
    if ((flags & MCM_EDGE_CENTER) == 0)
    {
        // Edge Lerp
        return mcmMeshContainsPoint<float, true>(data, dataSize, isoLevel, point);
    }
    else
    {
        // Edge Center
        return mcmMeshContainsPoint<float, false>(data, dataSize, isoLevel, point);
    }
}

McmResult mcmMeshIntersectRay(const float* data, const uint32_t dataSize[3], float isoLevel, const float rayPos[3], const float rayDir[3], McmFlags flags, float pIntersect[3])
{
    if ((flags & MCM_EDGE_CENTER) == 0)
    {
        // Edge Lerp
        return mcmMeshIntersectRay<float, true>(data, dataSize, isoLevel, rayPos, rayDir, pIntersect);
    }
    else
    {
        // Edge Center
        return mcmMeshIntersectRay<float, false>(data, dataSize, isoLevel, rayPos, rayDir, pIntersect);
    }
}

McmResult mcmMeshIntersectSegment(const float* data, const uint32_t dataSize[3], float isoLevel, const float segPos[3], const float segEnd[3], McmFlags flags, float pIntersect[3])
{
    if ((flags & MCM_EDGE_CENTER) == 0)
    {
        // Edge Lerp
        return mcmMeshIntersectSegment<float, true>(data, dataSize, isoLevel, segPos, segEnd, pIntersect);
    }
    else
    {
        // Edge Center
        return mcmMeshIntersectSegment<float, false>(data, dataSize, isoLevel, segPos, segEnd, pIntersect);
    }
}
