#pragma once

#include <vector>
#include "mesh.h"
#include "cameraparameters.h"

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

class MeshGenerator
{
public:
	MeshGenerator();

	static void generateTrianglesGradients(unsigned short *depth_image, std::vector<TriangleIndexes> &indexes, int ndepth_frame_width, int ndepth_frame_height);
	static int getNTrianglesPassingConditions(unsigned short  *initialPos, int w);
	
	void setBounds(float minX, float maxX, float minY, float maxY, float minZ, float maxZ);
	void generateMeshFromDepthMaps(int n_maps, unsigned char* depth_maps, unsigned char *depth_colors, int *widths, int *heights,
		float *intr_params, float *wtransform_params, Mesh *out_mesh);
private:
	static bool checkTriangleConstraints(unsigned short  *depth_ptr1, unsigned short  *depth_ptr2, unsigned short  *depth_ptr3);
	static void generateTrianglesGradientsRegion(unsigned short  *depth_image, std::vector<TriangleIndexes> &indexes, int ndepth_frame_width, int ndepth_frame_height,
		int minX, int minY, int maxX, int maxY);

	std::vector<float> bounds; 
};
