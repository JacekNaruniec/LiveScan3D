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

/*
struct MeshChunks
{
	VertexC4ubV3f *vertices;
	int *triangles;
	int *vertices_chunk_sizes;
	int *triangles_chunk_sizes;
};

extern "C" DEPTH_PROCESSING_API void __stdcall formMeshChunks(Mesh &mesh, MeshChunks *mesh_chunks);
*/

extern "C" DEPTH_PROCESSING_API void __stdcall generateVerticesFromDepthMap(__int64 *mesh_generator_handle, unsigned char* depth_maps,
	unsigned char *depth_colors, int *widths, int *heights, float *intr_params, float *wtransform_params, Mesh *out_mesh,
	float minX, float minY, float minZ, float maxX, float maxY, float maxZ, int depth_map_index);
extern "C" DEPTH_PROCESSING_API void __stdcall generateMeshFromDepthMaps(__int64 *mesh_generator_handle, int n_maps, unsigned char* depth_maps,
	unsigned char *depth_colors, int *widths, int *heights, float *intr_params, float *wtransform_params, Mesh *out_mesh, bool bcolor_transfer,
	float minX, float minY, float minZ, float maxX, float maxY, float maxZ, bool bgenerate_triangles);
extern "C" DEPTH_PROCESSING_API void __stdcall depthMapAndColorSetRadialCorrection(int n_maps, unsigned char* depth_maps, unsigned char *depth_colors, int *widths, int *heights, float *intr_params);

extern "C" DEPTH_PROCESSING_API void __stdcall createMeshChunks(__int64 *mesh_generator_handle, Mesh *mesh, MeshChunks *mesh_chunks);

extern "C" DEPTH_PROCESSING_API void __stdcall deleteMeshChunks(MeshChunks*);

extern "C" DEPTH_PROCESSING_API void __stdcall deleteMesh(Mesh*);
extern "C" DEPTH_PROCESSING_API Mesh* __stdcall createMesh();

extern "C" DEPTH_PROCESSING_API __int64* __stdcall createMeshGenerator();
extern "C" DEPTH_PROCESSING_API void __stdcall deleteMeshGenerator(__int64* ptr);


void storeAllFramesInformation(std::string filename, int n_maps, unsigned char* depth_maps,
	unsigned char *depth_colors, int *widths, int *heights, float *intr_params, float *wtransform_params);