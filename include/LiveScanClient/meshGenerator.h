#pragma once

#include "utils.h"
#include <vector>

struct TriangleIndexes
{
	int ind[3];
};

class MeshGenerator
{
public:
	MeshGenerator();

	static std::vector<TriangleIndexes> generateTriangles(UINT16 *depth_image, std::vector<int> &depth_to_vertices_map, 
		int ndepth_frame_width, int ndepth_frame_height);
	static std::vector<TriangleIndexes> generateTrianglesGradients(UINT16 *depth_image, std::vector<int> &depth_to_vertices_map,
		int ndepth_frame_width, int ndepth_frame_height);
private:
	static bool checkTriangleConstraints(UINT16 *depth_ptr1, UINT16 *depth_ptr2, UINT16 *depth_ptr3);

};