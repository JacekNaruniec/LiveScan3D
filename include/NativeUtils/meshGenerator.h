#pragma once

#include <vector>

struct TriangleIndexes
{
	int ind[3];
	int map_ind[3];
	TriangleIndexes(int i1, int i2, int i3, int map_ind_1 = -1, int map_ind_2 = -1, int map_ind_3 = -1):
		ind{ i1, i2, i3 }, map_ind{ map_ind_1, map_ind_2, map_ind_3 } 
		{ }
	TriangleIndexes() : ind{ -1, -1, -1 }, map_ind { -1, -1, -1 }
	{}
};



#ifndef UINT16
typedef unsigned short      UINT16, *PUINT16;
#endif

class MeshGenerator
{
public:
	MeshGenerator();

	static void generateTrianglesGradients(UINT16 *depth_image, std::vector<TriangleIndexes> &indexes, int ndepth_frame_width, int ndepth_frame_height);
	//static void generateTrianglesGradients(UINT16 *depth_image, std::vector<int> &depth_to_vertices_map,
	//	std::vector<TriangleIndexes> &indexes, int ndepth_frame_width, int ndepth_frame_height);
	static int getNTrianglesPassingConditions(UINT16 *initialPos, int w);

private:
	static bool checkTriangleConstraints(UINT16 *depth_ptr1, UINT16 *depth_ptr2, UINT16 *depth_ptr3);
	static void generateTrianglesGradientsRegion(UINT16 *depth_image, std::vector<TriangleIndexes> &indexes, int ndepth_frame_width, int ndepth_frame_height,
		int minX, int minY, int maxX, int maxY);
	//static void generateTrianglesGradientsRegion(UINT16 *depth_image, std::vector<int> &depth_to_vertices_map, std::vector<TriangleIndexes> &indexes, int ndepth_frame_width, int ndepth_frame_height,
	//	int minX, int minY, int maxX, int maxY);

};