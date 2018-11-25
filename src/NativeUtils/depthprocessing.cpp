#include "depthprocessing.h"
#include "meshGenerator.h"
#include <thread>
#include <unordered_map>
#include <algorithm>
#include <deque>
#include "colorcorrection.h"
#include "pgm.h"
#include <mutex>
#include "simpletimer.h"
#include "icp.h"
#include "cameraparameters.h"
#include "depthmaputils.h"

#include "simpleimage.h"


using namespace std;

void depthMapAndColorRadialCorrection(unsigned short *depth_map, unsigned char *colors, int w, int h, IntrinsicCameraParameters &intrinsic_params)
{
	vector<unsigned short> map_copy(w*h, 0);
	vector<unsigned char> colors_copy(w*h * 3, 0);

	float r2 = intrinsic_params.r2;
	float r4 = intrinsic_params.r4;
	float r6 = intrinsic_params.r6;

	for (int y = 0; y < h; y++)
		for (int x = 0; x<w; x++)
		{
			if (depth_map[x + y*w] == 0)
				continue;
			float u = (x - intrinsic_params.cx) / intrinsic_params.fx;
			float v = (y - intrinsic_params.cy) / intrinsic_params.fy;
			float r = u * u + v * v;
			float d = 1 - r2 * r - r4 * r * r - r6 * r * r * r;

			int x_corr = (int)(u * d * intrinsic_params.fx + intrinsic_params.cx);
			int y_corr = (int)(v * d * intrinsic_params.fy + intrinsic_params.cy);

			if (x_corr >= 0 && y_corr >= 0 && x_corr < w && y_corr < h)
			{
				map_copy[x_corr + y_corr * w] = depth_map[x + y*w];
				memcpy(colors_copy.data() + (x_corr + y_corr*w) * 3, colors + (x + y*w) * 3, 3);
			}
		}

	// closing holes
	const int shifts[] = { -w - 1, -w, -w + 1, -1, 1, w - 1, w, w + 1 };
	const int n_shifts = 8;
	for (int y = 1; y < h - 1; y++)
		for (int x = 1; x < w - 1; x++)
		{
			int pos = x + y*w;
			int val = map_copy[pos];

			if (val == 0)
			{
				int n = 0;
				int sum = 0;
				int sum_cR = 0, sum_cG = 0, sum_cB = 0;
				int prev_val = -1;
				for (int i = 0; i < n_shifts; i++)
					if ((map_copy[pos + shifts[i]] > 0) && (prev_val == -1 || abs(map_copy[pos + shifts[i]] - prev_val) < 30))
					{
						prev_val = map_copy[pos + shifts[i]];
						n++;
						sum += map_copy[pos + shifts[i]];
						sum_cR += colors_copy[(pos + shifts[i]) * 3];
						sum_cG += colors_copy[(pos + shifts[i]) * 3 + 1];
						sum_cB += colors_copy[(pos + shifts[i]) * 3 + 2];
					}

				if (n > 4)
				{
					map_copy[pos] = sum / n;
					colors_copy[pos * 3] = sum_cR / n;
					colors_copy[pos * 3 + 1] = sum_cG / n;
					colors_copy[pos * 3 + 2] = sum_cB / n;
				}
			}
		}

	memcpy(depth_map, map_copy.data(), w*h * sizeof(depth_map[0]));
	memcpy(colors, colors_copy.data(), w*h * 3 * sizeof(colors[0]));
}

DEPTH_PROCESSING_API void __stdcall generateMeshFromDepthMaps(__int64 *mesh_generator_handle, int n_maps, unsigned char* depth_maps,
	unsigned char *depth_colors, int *widths, int *heights, float *intr_params, float *wtransform_params, Mesh *out_mesh, bool bcolor_transfer,
	float minX, float minY, float minZ, float maxX, float maxY, float maxZ, bool bgenerate_triangles)
{
	MeshGenerator* mesh_generator = (MeshGenerator*)mesh_generator_handle;
	mesh_generator->setBounds(minX, maxX, minY, maxY, minZ, maxZ);

	mesh_generator->generateMeshFromDepthMaps(n_maps, depth_maps, depth_colors, widths, heights,
		intr_params, wtransform_params, out_mesh, bcolor_transfer, bgenerate_triangles, false);
}

DEPTH_PROCESSING_API void __stdcall createMeshChunks(__int64 *mesh_generator_handle, Mesh *mesh, MeshChunks *mesh_chunks)
{
	MeshGenerator* mesh_generator = (MeshGenerator*)mesh_generator_handle;
	mesh_generator->formMeshChunks(*mesh, *mesh_chunks);
}


void storeAllFramesInformation(string filename, int n_maps, unsigned char* depth_maps,
	unsigned char *depth_colors, int *widths, int *heights, float *intr_params, float *wtransform_params)
{
	FILE *f = fopen(filename.c_str(), "wb");

	fwrite(&n_maps, sizeof(n_maps), 1, f);
	if (n_maps > 0)
	{
		fwrite(widths, sizeof(widths[0]), n_maps, f);
		fwrite(heights, sizeof(heights[0]), n_maps, f);
	}

	int pos_d = 0, pos_c = 0;
	for (int i = 0; i < n_maps; i++)
	{
		fwrite(depth_maps + pos_d, 1, widths[i] * heights[i] * 2, f);
		fwrite(depth_colors + pos_c, 1, widths[i] * heights[i] * 3, f);
		pos_d += widths[i] * heights[i] * 2;
		pos_c += widths[i] * heights[i] * 3;
	}

	fwrite(intr_params, sizeof(intr_params[0]), 7 * n_maps, f);
	fwrite(wtransform_params, sizeof(wtransform_params[0]), 12 * n_maps, f);

	fclose(f);
}

extern "C" DEPTH_PROCESSING_API void __stdcall generateVerticesFromDepthMap(__int64 *mesh_generator_handle, unsigned char* depth_maps,
	unsigned char *depth_colors, int *widths, int *heights, float *intr_params, float *wtransform_params, Mesh *out_mesh,
	float minX, float minY, float minZ, float maxX, float maxY, float maxZ, int depth_map_index)
{
	// move pointers to the desired map parameters
	for (int i = 0; i < depth_map_index; i++)
	{
		int shift = widths[i] * heights[i];
		depth_maps += shift * 2;
		depth_colors += shift * 3;
	}

	widths += depth_map_index;
	heights += depth_map_index;
	intr_params += depth_map_index * 7;
	wtransform_params += depth_map_index * 12;
	
	generateMeshFromDepthMaps(mesh_generator_handle, 1, depth_maps, depth_colors, widths, heights, intr_params,
		wtransform_params, out_mesh, false, minX, minY, minZ, maxX, maxY, maxZ, false);
}


extern "C" DEPTH_PROCESSING_API void __stdcall depthMapAndColorSetRadialCorrection(int n_maps, unsigned char* depth_maps, unsigned char *depth_colors, int *widths, int *heights, float *intr_params)
{
	vector<IntrinsicCameraParameters> intrinsic_params(n_maps);

	for (int i = 0; i < n_maps; i++)
		intrinsic_params[i] = IntrinsicCameraParameters(intr_params + i * 7);

	int depth_pos = 0;
	int color_pos = 0;
	vector<thread> threads;

	for (int i = 0; i < n_maps; i++)
	{
		int n_pixels = widths[i] * heights[i];
		threads.push_back(thread(depthMapAndColorRadialCorrection, (unsigned short*)(depth_maps + depth_pos), depth_colors + color_pos, widths[i], heights[i], intrinsic_params[i]));
		depth_pos += n_pixels * 2;
		color_pos += n_pixels * 3;
	}

	for (int i = 0; i < n_maps; i++)
		threads[i].join();
}


extern "C" DEPTH_PROCESSING_API Mesh* __stdcall createMesh()
{
	Mesh *newMesh = new Mesh();
	newMesh->nTriangles = 0;
	newMesh->nVertices = 0;
	newMesh->triangles = NULL;
	newMesh->vertices = NULL;
	return newMesh;
}

extern "C" DEPTH_PROCESSING_API void __stdcall deleteMeshChunks(MeshChunks* mesh_chunks)
{
	if (mesh_chunks->triangles != nullptr)
		delete[]mesh_chunks->triangles;
	if (mesh_chunks->triangles_chunk_sizes != nullptr)
		delete[]mesh_chunks->triangles_chunk_sizes;
	if (mesh_chunks->vertices != nullptr)
		delete[]mesh_chunks->vertices;
	if (mesh_chunks->vertices_chunk_sizes != nullptr)
		delete[]mesh_chunks->vertices_chunk_sizes;
}


extern "C" DEPTH_PROCESSING_API void __stdcall deleteMesh(Mesh* mesh)
{
	if (mesh->triangles != 0)
	{
		delete[]mesh->triangles;
		mesh->nTriangles = 0;
	}

	if (mesh->vertices != 0)
	{
		delete[]mesh->vertices;
		mesh->nVertices = 0;
	}
}

extern "C" DEPTH_PROCESSING_API __int64* __stdcall createMeshGenerator()
{
	return (__int64*)(new MeshGenerator());	
}

extern "C" DEPTH_PROCESSING_API void __stdcall deleteMeshGenerator(__int64* ptr)
{
	if (ptr != nullptr)
		delete ptr; 
}