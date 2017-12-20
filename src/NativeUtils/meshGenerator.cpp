#include "meshGenerator.h"

using namespace std;

#include <chrono>
#include <algorithm>
#include <thread>

MeshGenerator::MeshGenerator()
{
	bounds = { -100.0f, 100.0f, -100.0f, 100.0f, -100.0f, 100.0f };
}

bool MeshGenerator::checkTriangleConstraints(unsigned short *depth_ptr1, unsigned short *depth_ptr2, unsigned short *depth_ptr3)
{
	unsigned short vals[3] = { *depth_ptr1, *depth_ptr2, *depth_ptr3 };
	unsigned short *ptrs[3] = { depth_ptr1, depth_ptr2, depth_ptr3 };

	const int pairs_1[] = { 0, 1, 2 };
	const int pairs_2[] = { 1, 2, 0 };

	if (vals[0] == 0 || vals[1] == 0 || vals[2] == 0)
		return false;

	// linear threshold set, that for the depth 1000 (1m) threshold is 10 (1 cm), at the depth 12000 (12m) threshold is 40 (4 cm)
	const int depth_thr = (int)((vals[0] + vals[1] + vals[2]) / 3.0 * 0.00272 * 2.0 + 7.273);
	//const int depth_thr = 10;

	for (int tr = 0; tr < 3; tr++)
	{
		int ind1 = pairs_1[tr];
		int ind2 = pairs_2[tr];
		int val1 = vals[ind1];
		int val2 = vals[ind2];
		// absolute depth condition
		if (abs(val1 - val2) < depth_thr)
			continue;

		// forward line depth linearity condition
		__int64 shift = ptrs[ind2] - ptrs[ind1];
		int val_forward = *(shift + ptrs[ind2]);

		if (val_forward != 0)
		{
			int gradient_forward = val_forward - val2;
			if (abs(val2 - val1 - gradient_forward * 0.3) < depth_thr)
				continue;
		}

		// backward line depth linearity condition
		int val_backward = *(ptrs[ind1] - shift);
		if (val_backward != 0)
		{
			int gradient_backward = val1 - val_backward;
			if (abs(val2 - val1 - gradient_backward * 0.3) < depth_thr)
				continue;
		}
		return false;
	}

	return true;
}

int MeshGenerator::getNTrianglesPassingConditions(unsigned short *initialPos, int w)
{
	const int pixel_shifts[] = { -1, -w - 1, -w };
	int n = 0;

	if (checkTriangleConstraints(initialPos, initialPos + pixel_shifts[0], initialPos + pixel_shifts[2])) n++;
	if (checkTriangleConstraints(initialPos + pixel_shifts[2], initialPos + pixel_shifts[0], initialPos + pixel_shifts[1])) n++;
	if (checkTriangleConstraints(initialPos, initialPos + pixel_shifts[0], initialPos + pixel_shifts[1])) n++;
	if (checkTriangleConstraints(initialPos, initialPos + pixel_shifts[1], initialPos + pixel_shifts[2])) n++;
	return n;
}

// Function made for parallelization of generateTrianglesGradients
void MeshGenerator::generateTrianglesGradientsRegion(unsigned short *depth_image, vector<TriangleIndexes> &indexes, int ndepth_frame_width, int ndepth_frame_height,
	int minX, int minY, int maxX, int maxY)
{
	int n_depth_pixels = (maxX - minX) * (maxY - minY);
	indexes.resize(n_depth_pixels * 2);
	int w = ndepth_frame_width;
	int h = ndepth_frame_height;
	minX = std::max(minX, 1);
	minY = std::max(minY, 2);
	maxX = std::min(maxX, ndepth_frame_width - 2);
	maxY = std::min(maxY, ndepth_frame_height - 2);

	int n_triangles = 0;

	const int pixel_shifts[] = { -w, -w + 1, 1 };
	const int n_pixel_shifts = 4;


	const int triangles_shifts[] = { pixel_shifts[2], pixel_shifts[0], 0,
		pixel_shifts[2], pixel_shifts[1], pixel_shifts[0],
		0,				  pixel_shifts[1], pixel_shifts[0],
		0,				  pixel_shifts[2] , pixel_shifts[1] };

	for (int y = minY; y < maxY; y++)
	{
		unsigned short *depth_row = depth_image + y * ndepth_frame_width;
		int pos_y = y * ndepth_frame_width;

		for (int x = minX; x < maxX; x++)
		{
			if (depth_row[x] == 0)
				continue;

			bool tr[4] = { false, false, false, false };
			tr[0] = checkTriangleConstraints(depth_row + x, depth_row + x + pixel_shifts[0], depth_row + x + pixel_shifts[2]);
			tr[1] = checkTriangleConstraints(depth_row + x + pixel_shifts[2], depth_row + x + pixel_shifts[0], depth_row + x + pixel_shifts[1]);

			if (!tr[0] && !tr[1])
			{
				tr[2] = checkTriangleConstraints(depth_row + x, depth_row + x + pixel_shifts[0], depth_row + x + pixel_shifts[1]);
				tr[3] = checkTriangleConstraints(depth_row + x, depth_row + x + pixel_shifts[1], depth_row + x + pixel_shifts[2]);
			}

			for (int i = 0; i<4; i++)
				if (tr[i])
				{
					int map1 = pos_y + x + triangles_shifts[i * 3];
					int map2 = pos_y + x + triangles_shifts[i * 3 + 1];
					int map3 = pos_y + x + triangles_shifts[i * 3 + 2];

					if (depth_image[map1] == 0 || depth_image[map2] == 0 || depth_image[map3] == 0)
						continue;

					indexes[n_triangles].ind[0] = map1;
					indexes[n_triangles].ind[1] = map2;
					indexes[n_triangles].ind[2] = map3;
					n_triangles++;
				}

		}
	}

	indexes.resize(n_triangles);
}


void MeshGenerator::generateTrianglesGradients(unsigned short *depth_image, vector<TriangleIndexes> &indexes, int ndepth_frame_width, int ndepth_frame_height)
{
	int n_threads = 4;
	vector<vector<TriangleIndexes>> partial_indexes(8);
	vector<thread> threads;

	int pos_y = 0;
	int step = ndepth_frame_height / n_threads + 1;
	for (int i = 0; i < n_threads; i++)
	{
		int size_y = min(step, ndepth_frame_height - pos_y);

		threads.push_back(thread(generateTrianglesGradientsRegion, depth_image, std::ref(partial_indexes[i]),
			ndepth_frame_width, ndepth_frame_height, 0, pos_y, ndepth_frame_width, pos_y + size_y));
		pos_y += size_y;
	}

	size_t n_indexes = 0;
	for (int i = 0; i < n_threads; i++)
	{
		threads[i].join();
		n_indexes += partial_indexes[i].size();
	}

	indexes.resize(n_indexes);
	size_t pos = 0;
	for (int i = 0; i < n_threads; i++)
	{
		if (partial_indexes[i].size() == 0)
			continue;

		memcpy(indexes.data() + pos, partial_indexes[i].data(), partial_indexes[i].size() * sizeof(partial_indexes[i][0]));
		pos += partial_indexes[i].size();
	}
}

void MeshGenerator::setBounds(float minX, float maxX, float minY, float maxY, float minZ, float maxZ)
{
	bounds = { minX, maxX, minY, maxY, minZ, maxZ };
}
void MeshGenerator::generateMapConfidence(vector<unsigned short> &depth_map, vector<unsigned char> &confidence_map, int w, int h, int et_limit,
	int depth_threshold)
{
	confidence_map.resize(w*h);
	std::fill(confidence_map.begin(), confidence_map.end(), et_limit);

	int *pos_x = new int[w*h];
	int *pos_y = new int[w*h];
	int *new_pos_x = new int[w*h];
	int *new_pos_y = new int[w*h];

	int shift_x[] = { -1, 0 ,  1, -1, 1, -1, 0, 1 };
	int shift_y[] = { -1, -1, -1,  0, 0,  1, 1, 1 };

	int pos_size = 0;
	int new_pos_size = 0;

	vector<bool> marked(w*h, false);

	for (int y = 0; y < h; y++)
	{
		confidence_map[y * w] = 0;
		confidence_map[y * w + w - 1] = 0;
	}

	for (int x = 0; x < w; x++)
	{
		confidence_map[x] = 0;
		confidence_map[x + (h - 1)*w] = 0;
	}

	for (int y = 1; y < h - 1; y++)
		for (int x = 1; x < w - 1; x++)
		{
			int pos = y*w + x;
			if (depth_map[pos] == 0)
			{
				confidence_map[pos] = 0;
				continue;
			}

			bool requirement_met = false;
			for (int shift = 0; shift < 8; shift++)
			{
				// check if any neighbour is a wall, if so -> mark it
				int new_pos = x + shift_x[shift] + (y + shift_x[shift])*w;
				if (abs(depth_map[pos] - depth_map[new_pos]) > depth_threshold || depth_map[new_pos] == 0)
				{
					requirement_met = true;
					break;
				}
			}
			if (requirement_met)
			{
				pos_x[pos_size] = x;
				pos_y[pos_size] = y;
				confidence_map[pos] = 1;
				marked[pos] = true;
				pos_size++;
			}
		}


	int max_et = 1;
	while (pos_size != 0 && max_et != et_limit)
	{
		for (int el = 0; el < pos_size; el++)
		{
			bool requirement_met = false;
			int el_pos = pos_x[el] + pos_y[el] * w;
			int x = pos_x[el];
			int y = pos_y[el];
			int depth = depth_map[el_pos];
			for (int shift = 0; shift < 8; shift++)
			{
				int new_x = x + shift_x[shift];
				int new_y = y + shift_y[shift];

				if (new_x <= 0 || new_y <= 0 || new_x >= w || new_y >= h)
					continue;
				int new_pos = new_x + new_y * w;

				// find point with confidence == 0
				if (abs(depth - depth_map[new_pos]) < depth_threshold && confidence_map[new_pos] == et_limit
					&& depth_map[new_pos] != 0)
				{
					confidence_map[new_x + new_y * w] = max_et + 1;
					new_pos_x[new_pos_size] = new_x;
					new_pos_y[new_pos_size] = new_y;
					new_pos_size++;
					marked[new_x + new_y * w] = true;
				}
			}
		}

		memcpy(pos_x, new_pos_x, new_pos_size * sizeof(new_pos_x[0]));
		memcpy(pos_y, new_pos_y, new_pos_size * sizeof(new_pos_y[0]));
		pos_size = new_pos_size;
		new_pos_size = 0;
		max_et++;
	}

	delete[]pos_x;
	delete[]pos_y;
	delete[]new_pos_x;
	delete[]new_pos_y;
}

void MeshGenerator::clearAllMapsConvexHull()
{
}

void MeshGenerator::updateColorCorrectionCoefficients()
{
}

void MeshGenerator::removeOverlappingRegions()
{
}

void MeshGenerator::generateTriangles()
{
}

void MeshGenerator::reprojectionCorrection()
{
}

void MeshGenerator::formMesh()
{
}


void MeshGenerator::generateDepthMapsConfidence()
{
	size_t n_maps = current_depth_maps.size();

	const int et_limit = 20;
	int depth_threshold = 20;
	depth_map_confidence.resize(n_maps);
	vector<thread> threads;

	for (int i = 0; i < n_maps; i++)
		threads.push_back(thread(&MeshGenerator::generateMapConfidence, this, std::ref(current_depth_maps[i]), std::ref(depth_map_confidence[i]), dm_widths[i], dm_heights[i], et_limit, depth_threshold));
	
	for (int i = 0; i < threads.size(); i++)
		threads[i].join();
}

void MeshGenerator::initializeVariables(int n_maps, unsigned char* depth_maps, float *intr_params, float *extr_params,
	 int *widths, int *heights)
{
	current_depth_maps.resize(n_maps);
	dm_widths.resize(n_maps);
	dm_heights.resize(n_maps);
	int depth_map_pos = 0;
	intrinsic_params.resize(n_maps);
	extrinsic_params.resize(n_maps);
	for (int i = 0; i < n_maps; i++)
	{
		intrinsic_params[i] = IntrinsicCameraParameters(intr_params + i * 7);
		extrinsic_params[i] = ExtrinsicCameraParameters(extr_params + i*(9 + 3));
		extrinsic_params[i].inv();
		dm_widths[i] = widths[i];
		dm_heights[i] = heights[i];
		current_depth_maps[i].resize(widths[i] * heights[i]);
		memcpy(current_depth_maps[i].data(), depth_maps + depth_map_pos, widths[i] * heights[i] * 2);
		depth_map_pos += widths[i] * heights[i] * 2;
	}

}

void MeshGenerator::generateMapCorrespondences(int base_map, int dest_map)
{

}

void MeshGenerator::generateAllMapsCorrespondences()
{
	int n_maps = current_depth_maps.size(); 
	map_correspondences.resize(n_maps);


	for (int base_map = 0; base_map < n_maps; base_map++)
	{
		
		totalRBaseToOverlay = multiply3x3Matrices(overlay_wt.R, base_wt_inv.R);

		int w = dm_widths[base_map];
		int h = dm_heights[base_map];
		unsigned short *depth_map = current_depth_maps[base_map].data();
		
		for (int y = 0; y < h; h++)
			for (int x = 0; x < w; x++)
			{
				int pos = x + y*w;
				unsigned short d = depth_map[pos];
				if (d == 0)
					continue; 



				for (int dest_map = 0; dest_map < n_maps; dest_map++)
				{
					if (base_map == dest_map)
						continue;

				}
			}
	}
}

void MeshGenerator::generateMeshFromDepthMaps(int n_maps, unsigned char* depth_maps, unsigned char *depth_colors, int *widths, int *heights,
	float *intr_params, float *extr_params, Mesh *out_mesh, bool bcolor_transfer, bool bgenerate_triangles)
{
	initializeVariables(n_maps, depth_maps, intr_params, extr_params, widths, heights);
	generateDepthMapsConfidence();
	generateAllMapsCorrespondences();
	updateColorCorrectionCoefficients();
	clearAllMapsConvexHull();
	removeOverlappingRegions();
	generateTriangles();
	reprojectionCorrection();
	formMesh();
}