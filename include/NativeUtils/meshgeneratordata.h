#pragma once

#include <vector>
#include "triangleindexes.h"

struct MeshGeneratorData
{
	std::vector<std::vector<int>> vertices_map;
	std::vector<int> vertex_depth_map_pos, vertex_depth_map_index;
	std::vector<std::vector<TriangleIndexes>> triangle_indexes;
	std::vector<std::vector<TriangleIndexes>> partial_indexes;
	std::vector<std::vector<unsigned char>> edges;
	std::vector<int> mc_chunk_index;
	std::vector<int> mc_vertices_map;
};