#include "mesh.h"

void clearMeshChunks(MeshChunks &mesh_chunks)
{
	if (mesh_chunks.triangles != nullptr)
	{
		delete[]mesh_chunks.triangles;
		mesh_chunks.triangles = nullptr;
	}

	if (mesh_chunks.vertices != nullptr)
	{
		delete[]mesh_chunks.vertices;
		mesh_chunks.vertices = nullptr;
	}

	if (mesh_chunks.triangles_chunk_sizes != nullptr)
	{
		delete[]mesh_chunks.triangles_chunk_sizes;
		mesh_chunks.triangles_chunk_sizes = nullptr; 
	}

	if (mesh_chunks.vertices_chunk_sizes != nullptr)
	{
		delete[]mesh_chunks.vertices_chunk_sizes;
		mesh_chunks.vertices_chunk_sizes = nullptr;
	}
}