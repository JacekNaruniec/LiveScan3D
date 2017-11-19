#include "meshGenerator.h"

using namespace std;

#include <chrono>
#include <algorithm>
#include <thread>

MeshGenerator::MeshGenerator()
{

}

bool MeshGenerator::checkTriangleConstraints(UINT16 *depth_ptr1, UINT16 *depth_ptr2, UINT16 *depth_ptr3)
{
	UINT16 vals[3] = { *depth_ptr1, *depth_ptr2, *depth_ptr3 };
	UINT16 *ptrs[3] = { depth_ptr1, depth_ptr2, depth_ptr3 };

	const int pairs_1[] = { 0, 1, 2 };
	const int pairs_2[] = { 1, 2, 0 };

	if (vals[0] == 0 || vals[1] == 0 || vals[2] == 0)
		return false;

	// linear threshold set, that for the depth 1000 (1m) threshold is 10 (1 cm), at the depth 12000 (12m) threshold is 40 (4 cm)
	const int depth_thr = (int)((vals[0] + vals[1] + vals[2]) / 3.0 * 0.00272 + 7.273);
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


int MeshGenerator::getNTrianglesPassingConditions(UINT16 *initialPos, int w)
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
/*void MeshGenerator::generateTrianglesGradientsRegion(UINT16 *depth_image, vector<int> &depth_to_vertices_map, vector<TriangleIndexes> &indexes, int ndepth_frame_width, int ndepth_frame_height,
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
		UINT16 *depth_row = depth_image + y * ndepth_frame_width;
		int *map_row = depth_to_vertices_map.data() + y * ndepth_frame_width;

		for (int x = minX; x < maxX; x++)
		{
			if (map_row[x] == -1)
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
					int map1 = map_row[x + triangles_shifts[i * 3]];
					int map2 = map_row[x + triangles_shifts[i * 3 + 1]];
					int map3 = map_row[x + triangles_shifts[i * 3 + 2]];

					if (map1 == -1 || map2 == -1 || map3 == -1)
						continue;

					indexes[n_triangles].ind[0] = map1;
					indexes[n_triangles].ind[1] = map2;
					indexes[n_triangles].ind[2] = map3;
					n_triangles++;
				}

		}
	}

	indexes.resize(n_triangles);
}*/


// Function made for parallelization of generateTrianglesGradients
void MeshGenerator::generateTrianglesGradientsRegion(UINT16 *depth_image, vector<TriangleIndexes> &indexes, int ndepth_frame_width, int ndepth_frame_height,
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
		0,				  pixel_shifts[2] , pixel_shifts[1]};
		
	for (int y = minY; y < maxY; y++)
	{
		UINT16 *depth_row = depth_image + y * ndepth_frame_width;
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


void MeshGenerator::generateTrianglesGradients(UINT16 *depth_image, vector<TriangleIndexes> &indexes, int ndepth_frame_width, int ndepth_frame_height)
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


/*
void MeshGenerator::generateTrianglesGradients(UINT16 *depth_image, vector<int> &depth_to_vertices_map, vector<TriangleIndexes> &indexes, int ndepth_frame_width, int ndepth_frame_height)
{
	int n_threads = 4;
	vector<vector<TriangleIndexes>> partial_indexes(8);
	vector<thread> threads;

	int pos_y = 0;
	int step = ndepth_frame_height / n_threads + 1;
	for (int i = 0; i < n_threads; i++)
	{
		int size_y = min(step, ndepth_frame_height - pos_y);

		threads.push_back(thread(generateTrianglesGradientsRegion, depth_image, std::ref(depth_to_vertices_map), std::ref(partial_indexes[i]),
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
*/