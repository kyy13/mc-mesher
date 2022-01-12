// MCmesher
// Kyle J Burgess

#include "MarchingCubes.h"
#include "LookupTable.h"

#include <stdexcept>
#include <vector>
#include <cstring>
#include <cmath>
#include <iostream>

ScalarField::ScalarField()
    : width(0)
    , height(0)
    , depth(0)
{}

ScalarField::ScalarField(uint32_t _width, uint32_t _height, uint32_t _depth)
    : width(_width)
    , height(_height)
    , depth(_depth)
{}

CubeSlice::CubeSlice()
    : x0(0)
    , y0(0)
    , z0(0)
    , width(0)
    , height(0)
    , depth(0)
{}

CubeSlice::CubeSlice(uint32_t _x0, uint32_t _y0, uint32_t _z0, uint32_t _width, uint32_t _height, uint32_t _depth)
    : x0(_x0)
    , y0(_y0)
    , z0(_z0)
    , width(_width)
    , height(_height)
    , depth(_depth)
{}

struct Mesh
{
    std::vector<Vector3> vertices;
    std::vector<Vector3> faceNormals;
    std::vector<Vector3> vertexNormals;
    std::vector<uint32_t> indices;
};

template<bool ComputeFaceNormals, bool ComputeVertexNormals>
void GenerateMesh(Mesh* mesh, const float* data, const ScalarField& field, const CubeSlice& slice, float isoLevel)
{
    Vector3 triVertices[3];
    Vector3 triNormals[3];
    Vector3 cubeNormals[8];

    Vector3 triFaceNormal = {};

    float corner[8];

    if (mesh == nullptr)
    {
        throw std::runtime_error("called GenerateMesh() on null mesh");
    }

    auto& vertices = mesh->vertices;
    auto& faceNormals = mesh->faceNormals;
    auto& vertexNormals = mesh->vertexNormals;
    auto& indices = mesh->indices;

    uint32_t x0 = slice.x0;
    uint32_t y0 = slice.y0;
    uint32_t z0 = slice.z0;

    uint32_t x1 = x0 + slice.width;

    if ((x1 >= field.width) || (x0 >= x1))
    {
        throw std::runtime_error("slice x-axis out of bounds of data field");
    }

    uint32_t y1 = y0 + slice.height;

    if ((y1 >= field.height) || (y0 >= y1))
    {
        throw std::runtime_error("slice y-axis out of bounds of data field");
    }

    uint32_t z1 = z0 + slice.depth;

    if ((z1 >= field.depth) || (z0 >= z1))
    {
        throw std::runtime_error("slice z-axis out of bounds of data field");
    }

    uint32_t vertexCount = 0;

    vertices.clear();
    indices.clear();
    faceNormals.clear();
    vertexNormals.clear();

    uint32_t w = field.width;
    uint32_t wh = field.width * field.height;

    const float* voxel = &data[z0 * wh + y0 * w + x0];

    const float* px1 = &voxel[slice.width];
    const float* py1 = &voxel[slice.height * w];
    const float* pz1 = &voxel[slice.depth * wh];

    uint32_t pdy = w - slice.width; // wrap to next y
    uint32_t pdz = (field.height - slice.height) * w; // wrap to next z

    uint32_t x, y, z = z0;

    Vector3 fieldOrigin = {static_cast<float>(x0), static_cast<float>(y0), static_cast<float>(z0)};
    Vector3 cubeOrigin = fieldOrigin;

    // Field max cube x, y, z
    uint32_t fieldCubeX1, fieldCubeY1, fieldCubeZ1;

    if constexpr (ComputeVertexNormals)
    {
        fieldCubeX1 = field.width - 2;
        fieldCubeY1 = field.height - 2;
        fieldCubeZ1 = field.depth - 2;
    }

    // Memory offset lookup table for cube and adjacent faces

    /*               p28------ p30
     *              /|        /|
     *             / |       / |
     *            p29------ p31|
     *     p18----|- p4 ----|- p6 ------ p22
     *    /|      | /|      | /|        /|
     *   / |      |/ |      |/ |       / |      z
     *  p19------ p5 ------ p7 ------ p23|      |
     *  |  p16----|- p0 ----|- p2 ----|- p20    o----> y
     *  | /       | /|      | /|      | /      /
     *  |/        |/ |      |/ |      |/      x
     *  p17------ p1 ------ p3 ------ p21
     *            |  p24----|- p26
     *            | /       | /
     *            |/        |/
     *            p25------ p27
     *
     *
     *
     *                  p10------ p11
     *                 /|        /|
     *                / |       / |
     *               p4 ------ p6 |
     *              /|  p8 ---/|- p9
     *             / | /     / | /              z
     *            p5 ------ p7 |/               |
     *           /|  p0 ---/|- p2               o----> y
     *          / | /     / | /                /
     *         p14------ p15|/                x
     *         |  p1 ----|- p3
     *         | /       | /
     *         |/        |/
     *         p12------ p13
     */

    int memOffsets[32];

    {
        int mem_w = static_cast<int>(field.width);
        int mem_wh = static_cast<int>(field.width * field.height);

        memOffsets[0] = 0;
        memOffsets[1] = 1;
        memOffsets[2] = mem_w;
        memOffsets[3] = mem_w + 1;

        memOffsets[4] = mem_wh + memOffsets[0];
        memOffsets[5] = mem_wh + memOffsets[1];
        memOffsets[6] = mem_wh + memOffsets[2];
        memOffsets[7] = mem_wh + memOffsets[3];

        // Only need these for vertex normals
        if constexpr (ComputeVertexNormals)
        {
            memOffsets[8] = memOffsets[0] - 1;
            memOffsets[9] = memOffsets[2] - 1;
            memOffsets[10] = memOffsets[4] - 1;
            memOffsets[11] = memOffsets[6] - 1;

            memOffsets[12] = memOffsets[1] + 1;
            memOffsets[13] = memOffsets[3] + 1;
            memOffsets[14] = memOffsets[5] + 1;
            memOffsets[15] = memOffsets[7] + 1;

            memOffsets[16] = memOffsets[0] - mem_w;
            memOffsets[17] = memOffsets[1] - mem_w;
            memOffsets[18] = memOffsets[4] - mem_w;
            memOffsets[19] = memOffsets[5] - mem_w;

            memOffsets[20] = memOffsets[2] + mem_w;
            memOffsets[21] = memOffsets[3] + mem_w;
            memOffsets[22] = memOffsets[6] + mem_w;
            memOffsets[23] = memOffsets[7] + mem_w;

            memOffsets[24] = memOffsets[0] - mem_wh;
            memOffsets[25] = memOffsets[1] - mem_wh;
            memOffsets[26] = memOffsets[2] - mem_wh;
            memOffsets[27] = memOffsets[3] - mem_wh;

            memOffsets[28] = memOffsets[4] + mem_wh;
            memOffsets[29] = memOffsets[5] + mem_wh;
            memOffsets[30] = memOffsets[6] + mem_wh;
            memOffsets[31] = memOffsets[7] + mem_wh;
        }
    }

    // Subset of memory offsets clamped to boundaries throughout the voxel iterator
    // first 8 indices are omitted, because they are always in scope, so range is [8,32)

    /*               p20------ p22
     *              /|        /|
     *             / |       / |
     *            p21------ p23|
     *     p10----|- c4 ----|- c6 ------ p14
     *    /|      | /|      | /|        /|
     *   / |      |/ |      |/ |       / |      z
     *  p11------ c5 ------ c7 ------ p15|      |
     *  |  p8 ----|- c0 ----|- c2 ----|- p12    o----> y
     *  | /       | /|      | /|      | /      /
     *  |/        |/ |      |/ |      |/      x
     *  p9 ------ c1 ------ c3 ------ p13
     *            |  p16----|- p18
     *            | /       | /
     *            |/        |/
     *            p17------ p19
     *
     *
     *
     *                  p2 ------ p3
     *                 /|        /|
     *                / |       / |
     *               c4 ------ c6 |
     *              /|  p0 ---/|- p1
     *             / | /     / | /              z
     *            c5 ------ c7 |/               |
     *           /|  c0 ---/|- c2               o----> y
     *          / | /     / | /                /
     *         p6 ------ p7 |/                x
     *         |  c1 ----|- c3
     *         | /       | /
     *         |/        |/
     *         p4 ------ p5
     */

    int memBoundedOffsets[24];

    for (; voxel != pz1; voxel += pdz)
    {
        y = y0;
        cubeOrigin.y = fieldOrigin.y;

        if constexpr (ComputeVertexNormals)
        {
            // Clamp -Z adjacent memory offsets
            if (z == 0)
            {
                memBoundedOffsets[16] = memOffsets[0];
                memBoundedOffsets[17] = memOffsets[1];
                memBoundedOffsets[18] = memOffsets[2];
                memBoundedOffsets[19] = memOffsets[3];
            }
            else
            {
                memBoundedOffsets[16] = memOffsets[24];
                memBoundedOffsets[17] = memOffsets[25];
                memBoundedOffsets[18] = memOffsets[26];
                memBoundedOffsets[19] = memOffsets[27];
            }

            // Clamp +Z adjacent memory offsets
            if (z == fieldCubeZ1)
            {
                memBoundedOffsets[20] = memOffsets[4];
                memBoundedOffsets[21] = memOffsets[5];
                memBoundedOffsets[22] = memOffsets[6];
                memBoundedOffsets[23] = memOffsets[7];
            }
            else
            {
                memBoundedOffsets[20] = memOffsets[28];
                memBoundedOffsets[21] = memOffsets[29];
                memBoundedOffsets[22] = memOffsets[30];
                memBoundedOffsets[23] = memOffsets[31];
            }
        }

        for (; voxel != py1; voxel += pdy)
        {
            x = x0;
            cubeOrigin.x = fieldOrigin.x;

            if constexpr (ComputeVertexNormals)
            {
                // Clamp -Y adjacent memory offsets
                if (y == 0)
                {
                    memBoundedOffsets[8] = memOffsets[0];
                    memBoundedOffsets[9] = memOffsets[1];
                    memBoundedOffsets[10] = memOffsets[4];
                    memBoundedOffsets[11] = memOffsets[5];
                }
                else
                {
                    memBoundedOffsets[8] = memOffsets[16];
                    memBoundedOffsets[9] = memOffsets[17];
                    memBoundedOffsets[10] = memOffsets[18];
                    memBoundedOffsets[11] = memOffsets[19];
                }

                // Clamp +Y adjacent memory offsets
                if (y == fieldCubeY1)
                {
                    memBoundedOffsets[12] = memOffsets[2];
                    memBoundedOffsets[13] = memOffsets[3];
                    memBoundedOffsets[14] = memOffsets[6];
                    memBoundedOffsets[15] = memOffsets[7];
                }
                else
                {
                    memBoundedOffsets[12] = memOffsets[20];
                    memBoundedOffsets[13] = memOffsets[21];
                    memBoundedOffsets[14] = memOffsets[22];
                    memBoundedOffsets[15] = memOffsets[23];
                }
            }

            for (; voxel != px1; ++voxel)
            {
                corner[0] = *(voxel);
                corner[1] = *(voxel + 1);
                corner[2] = *(voxel + memOffsets[2]);
                corner[3] = *(voxel + memOffsets[3]);
                corner[4] = *(voxel + memOffsets[4]);
                corner[5] = *(voxel + memOffsets[5]);
                corner[6] = *(voxel + memOffsets[6]);
                corner[7] = *(voxel + memOffsets[7]);

                // Get Case Index
                uint8_t caseIndex = 0;

                if (corner[0] >= isoLevel) caseIndex |= 0x01u;
                if (corner[1] >= isoLevel) caseIndex |= 0x02u;
                if (corner[2] >= isoLevel) caseIndex |= 0x04u;
                if (corner[3] >= isoLevel) caseIndex |= 0x08u;
                if (corner[4] >= isoLevel) caseIndex |= 0x10u;
                if (corner[5] >= isoLevel) caseIndex |= 0x20u;
                if (corner[6] >= isoLevel) caseIndex |= 0x40u;
                if (corner[7] >= isoLevel) caseIndex |= 0x80u;

                // Cell has trivial triangulation
                if (caseIndex == 0x00u || caseIndex == 0xffu)
                {
                    ++x;
                    cubeOrigin.x += 1.0f;
                    continue;
                }

                uint32_t caseIndex12 = caseIndex * 12u;
                uint32_t cellClass16 = LookupTable::RegularCellClass[caseIndex] << 4u;

                uint32_t geometryCounts = LookupTable::RegularCellData[cellClass16];
                uint32_t triangleCount = (geometryCounts & 0x0Fu);
                uint32_t vertCount = triangleCount * 3;

                // Calculate vertex normals
                if constexpr (ComputeVertexNormals)
                {
                    // Clamp -X adjacent memory offsets
                    if (x == 0)
                    {
                        memBoundedOffsets[0] = memOffsets[0];
                        memBoundedOffsets[1] = memOffsets[2];
                        memBoundedOffsets[2] = memOffsets[4];
                        memBoundedOffsets[3] = memOffsets[6];
                    }
                    else
                    {
                        memBoundedOffsets[0] = memOffsets[8];
                        memBoundedOffsets[1] = memOffsets[9];
                        memBoundedOffsets[2] = memOffsets[10];
                        memBoundedOffsets[3] = memOffsets[11];
                    }

                    // Clamp +X adjacent memory offsets
                    if (x == fieldCubeX1)
                    {
                        memBoundedOffsets[4] = memOffsets[1];
                        memBoundedOffsets[5] = memOffsets[3];
                        memBoundedOffsets[6] = memOffsets[5];
                        memBoundedOffsets[7] = memOffsets[7];
                    }
                    else
                    {
                        memBoundedOffsets[4] = memOffsets[12];
                        memBoundedOffsets[5] = memOffsets[13];
                        memBoundedOffsets[6] = memOffsets[14];
                        memBoundedOffsets[7] = memOffsets[15];
                    }

                    // Setup un-normalized cube normals
                    cubeNormals[0] =
                        {
                            .x = *(voxel + memBoundedOffsets[0]) - corner[1],
                            .y = *(voxel + memBoundedOffsets[8]) - corner[2],
                            .z = *(voxel + memBoundedOffsets[16]) - corner[4],
                        };

                    cubeNormals[1] =
                        {
                            .x = corner[0] - *(voxel + memBoundedOffsets[4]),
                            .y = *(voxel + memBoundedOffsets[9]) - corner[3],
                            .z = *(voxel + memBoundedOffsets[17]) - corner[5],
                        };

                    cubeNormals[2] =
                        {
                            .x = *(voxel + memBoundedOffsets[1]) - corner[3],
                            .y = corner[0] - *(voxel + memBoundedOffsets[12]),
                            .z = *(voxel + memBoundedOffsets[18]) - corner[6],
                        };

                    cubeNormals[3] =
                        {
                            .x = corner[2] - *(voxel + memBoundedOffsets[5]),
                            .y = corner[1] - *(voxel + memBoundedOffsets[13]),
                            .z = *(voxel + memBoundedOffsets[19]) - corner[7],
                        };

                    cubeNormals[4] =
                        {
                            .x = *(voxel + memBoundedOffsets[2]) - corner[5],
                            .y = *(voxel + memBoundedOffsets[10]) - corner[6],
                            .z = corner[0] - *(voxel + memBoundedOffsets[20]),
                        };

                    cubeNormals[5] =
                        {
                            .x = corner[4] - *(voxel + memBoundedOffsets[6]),
                            .y = *(voxel + memBoundedOffsets[11]) - corner[7],
                            .z = corner[1] - *(voxel + memBoundedOffsets[21]),
                        };

                    cubeNormals[6] =
                        {
                            .x = *(voxel + memBoundedOffsets[3]) - corner[7],
                            .y = corner[4] - *(voxel + memBoundedOffsets[14]),
                            .z = corner[2] - *(voxel + memBoundedOffsets[22]),
                        };

                    cubeNormals[7] =
                        {
                            .x = corner[6] - *(voxel + memBoundedOffsets[7]),
                            .y = corner[5] - *(voxel + memBoundedOffsets[15]),
                            .z = corner[3] - *(voxel + memBoundedOffsets[23]),
                        };

                    // Normalize normal vectors

                    for (auto& cubeNormal : cubeNormals)
                    {
                        cubeNormal.normalize();
                    }
                }

                // Step vertices
                for (uint32_t i = 0; i != vertCount; i += 3)
                {
                    for (uint32_t j = 0; j != 3; ++j)
                    {
                        uint32_t vertexIndex = LookupTable::RegularCellData[cellClass16 + i + j + 1];

                        uint32_t vertexData = LookupTable::RegularVertexData[caseIndex12 + vertexIndex] & 0xFFu;

                        uint32_t endpointIndex0 = (vertexData >> 4u);
                        uint32_t endpointIndex1 = (vertexData & 0x0F);

                        const Vector3& d0 = LookupTable::UnitCube[endpointIndex0];

                        Vector3 endpoint = cubeOrigin + d0;

                        Vector3 dEndpoint = LookupTable::UnitCube[endpointIndex1] - d0;

                        Vector3& normal0 = cubeNormals[endpointIndex0];

                        Vector3 dNormal = cubeNormals[endpointIndex1] - normal0;

                        // Lerp factor between endpoints
                        float k = (isoLevel - corner[endpointIndex0]) / (corner[endpointIndex1] - corner[endpointIndex0]);

                        // Lerp vertices
                        triVertices[j] = endpoint + k * dEndpoint;

                        // Lerp vertex normals
                        if constexpr (ComputeVertexNormals)
                        {

                            triNormals[j] = normal0 + k * dNormal;
                            triNormals[j].normalize();
                        }
                    }

                    // Calculate triangle segments that are too small to render
                    constexpr float epsilon = 0.0000001f;

                    Vector3 d01 = triVertices[1] - triVertices[0];

                    if (fabsf(d01.x) < epsilon && fabsf(d01.y) < epsilon && fabsf(d01.z) < epsilon)
                    {
                        ++x;
                        cubeOrigin.x += 1.0f;
                        continue;
                    }

                    Vector3 d12 = triVertices[2] - triVertices[1];

                    if (fabsf(d12.x) < epsilon && fabsf(d12.y) < epsilon && fabsf(d12.z) < epsilon)
                    {
                        ++x;
                        cubeOrigin.x += 1.0f;
                        continue;
                    }

                    Vector3 d02 = triVertices[2] - triVertices[0];

                    if (fabsf(d02.x) < epsilon && fabsf(d02.y) < epsilon && fabsf(d02.z) < epsilon)
                    {
                        ++x;
                        cubeOrigin.x += 1.0f;
                        continue;
                    }

                    // Set vertices

                    vertices.push_back(triVertices[0]);
                    vertices.push_back(triVertices[1]);
                    vertices.push_back(triVertices[2]);

                    // Set normals

                    if constexpr (ComputeVertexNormals)
                    {
                        vertexNormals.push_back(triNormals[0]);
                        vertexNormals.push_back(triNormals[1]);
                        vertexNormals.push_back(triNormals[2]);
                    }
                    else if constexpr (ComputeFaceNormals)
                    {
                        triFaceNormal = Vector3::cross(d01, d02);
                        triFaceNormal.normalize();

                        faceNormals.push_back(triFaceNormal);
                        faceNormals.push_back(triFaceNormal);
                        faceNormals.push_back(triFaceNormal);
                    }

                    // Set indices

                    indices.push_back(vertexCount);
                    ++vertexCount;
                    indices.push_back(vertexCount);
                    ++vertexCount;
                    indices.push_back(vertexCount);
                    ++vertexCount;
                }
                ++x;
                cubeOrigin.x += 1.0f;
            }
            px1 += w;
            ++y;
            cubeOrigin.y += 1.0f;
        }
        px1 += pdz;
        py1 += wh;
        ++z;
        cubeOrigin.z += 1.0f;
    }
}

void GenerateMesh(Mesh* mesh, const float* data, const ScalarField& field, const CubeSlice& slice, float isoLevel, bool computeFaceNormals, bool computeVertexNormals)
{
    if (computeFaceNormals)
    {
        if (computeVertexNormals)
        {
            GenerateMesh<true, true>(mesh, data, field, slice, isoLevel);
        }
        else
        {
            GenerateMesh<true, false>(mesh, data, field, slice, isoLevel);
        }
    }
    else
    {
        if (computeVertexNormals)
        {
            GenerateMesh<false, true>(mesh, data, field, slice, isoLevel);
        }
        else
        {
            GenerateMesh<false, false>(mesh, data, field, slice, isoLevel);
        }
    }
}

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

const Vector3* GetVertices(const Mesh* mesh)
{
    return mesh->vertices.data();
}

void CopyVertices(const Mesh* mesh, Vector3* dst)
{
    memcpy(dst, mesh->vertices.data(), mesh->vertices.size() * sizeof(Vector3));
}

uint32_t CountFaceNormals(const Mesh* mesh)
{
    return mesh->faceNormals.size();
}

const Vector3* GetFaceNormals(const Mesh* mesh)
{
    return mesh->faceNormals.data();
}

void CopyFaceNormals(const Mesh* mesh, Vector3* dst)
{
    memcpy(dst, mesh->faceNormals.data(), mesh->faceNormals.size() * sizeof(Vector3));
}

uint32_t CountVertexNormals(const Mesh* mesh)
{
    return mesh->vertexNormals.size();
}

const Vector3* GetVertexNormals(const Mesh* mesh)
{
    return mesh->vertexNormals.data();
}

void CopyVertexNormals(const Mesh* mesh, Vector3* dst)
{
    memcpy(dst, mesh->vertexNormals.data(), mesh->vertexNormals.size() * sizeof(Vector3));
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
