#pragma once

struct VertexC4ubV3f
{
	unsigned char R, G, B, A;
	float X, Y, Z;
};

struct Mesh
{
	int nVertices = 0;
	VertexC4ubV3f *vertices = nullptr;
	int nTriangles = 0;
	int *triangles = nullptr;
};

struct MeshChunks
{
	VertexC4ubV3f* vertices = 0;
	int *triangles = 0;
	int *vertices_chunk_sizes = 0;
    int *triangles_chunk_sizes = 0;
	int n_chunks = 0;
};

void clearMeshChunks(MeshChunks &mesh_chunks);