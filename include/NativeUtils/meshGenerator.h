#pragma once

#include <vector>

struct TriangleIndexes
{
	int ind[3];
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