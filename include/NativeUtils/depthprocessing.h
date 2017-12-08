#pragma once

#include <stdio.h>
#include <vector>
#include "meshGenerator.h"

#include "icp.h"

#if defined(DEPTH_PROCESSING_DLL_EXPORTS) // inside DLL
#   define DEPTH_PROCESSING_API   __declspec(dllexport)
#else // outside DLL
#   define DEPTH_PROCESSING_API   __declspec(dllimport)
#endif  

//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }

struct VertexC4ubV3f
{
	unsigned char R, G, B, A;
	float X, Y, Z;
};

struct PointProjection
{
	int x, y;
	unsigned short d;
	int index;
};

struct Mesh
{
	int nVertices;
	VertexC4ubV3f *vertices;
	int nTriangles;
	int *triangles;
};

struct Connection
{
	int point1_index; 
	int point2_index; 
	unsigned short depth1, depth2; 
};

struct WorldTransformation
{
	std::vector<float> t;
	std::vector<std::vector<float>> R;
	WorldTransformation() {}

	WorldTransformation(float *p)
	{
		t.resize(3);
		memcpy(t.data(), p, 3 * sizeof(float));
		R = std::vector<std::vector<float>>(3, std::vector<float>(3));
		for (int i = 0; i < 3; i++)
			memcpy(R[i].data(), p + 3 + 3 * i, 3 * sizeof(float));
	}

	void inv()
	{
		std::vector<std::vector<float>> Rt(3, std::vector<float>(3));
		for (int i = 0; i < 3; i++)
		{
			t[i] = -t[i];
			for (int j = 0; j < 3; j++)
				Rt[i][j] = R[j][i];
		}
		R = Rt;
	}
};

struct VerticesWithDepthColorMaps
{
	std::vector<Point3f> vertices;
	std::vector<unsigned char> colors;
	std::vector<int> depth_to_vertices_map;
	std::vector<int> vertices_to_depth_map;
	std::vector<bool> point_assigned;
	std::vector<unsigned short> depth_map;
	std::vector<unsigned char> confidence_map;
	std::vector<unsigned char> colors_map;
};

struct IntrinsicCameraParameters
{
	float cx, cy;        // principal points
	float fx, fy;        // focal lengths
	float r2, r4, r6;    // camera radial distortion parameters (second, fourth and sixth order)
	IntrinsicCameraParameters() {}
	IntrinsicCameraParameters(float *p) :
		cx(p[0]), cy(p[1]), fx(p[2]), fy(p[3]), r2(p[4]), r4(p[5]), r6(p[6]) {}
};

void RotatePoint(Point3f &point, std::vector<std::vector<float>> &R);
void writeDepthImage(std::vector<unsigned short> &depth_image, int w, int h, std::string filename);
std::vector<TriangleIndexes> generateTrianglesForStiches(std::vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *heights, int *widths, std::vector<WorldTransformation> &world_transforms,
	std::vector<IntrinsicCameraParameters> &intrinsic_params);


extern "C" DEPTH_PROCESSING_API void __stdcall generateVerticesFromDepthMap(unsigned char* depth_maps,
	unsigned char *depth_colors, int *widths, int *heights, float *intr_params, float *wtransform_params, Mesh *out_mesh,
	float minX, float minY, float minZ, float maxX, float maxY, float maxZ, int depth_map_index);


extern "C" DEPTH_PROCESSING_API void __stdcall generateMeshFromDepthMaps(int n_maps, unsigned char* depth_maps,
	unsigned char *depth_colors, int *widths, int *heights, float *intr_params, float *wtransform_params, Mesh *out_mesh, bool bcolor_transfer,
	float minX, float minY, float minZ, float maxX, float maxY, float maxZ, bool bgenerate_triangles);
extern "C" DEPTH_PROCESSING_API void __stdcall depthMapAndColorSetRadialCorrection(int n_maps, unsigned char* depth_maps, unsigned char *depth_colors, int *widths, int *heights, float *intr_params);
extern "C" DEPTH_PROCESSING_API void __stdcall deleteMesh(Mesh*);
