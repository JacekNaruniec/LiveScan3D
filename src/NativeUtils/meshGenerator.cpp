#include "meshGenerator.h"

using namespace std;

#include <chrono>

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

	// threshold set, that for the depth 1000 (1m) threshold is 10 (1 cm), at the depth 12000 (12m) threshold is 40 (4 cm)
	const int depth_thr = (int)((vals[0] + vals[1] + vals[2]) / 3.0 * 0.00272 + 7.273);

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
			if (abs(val2 - val1 - gradient_forward) < depth_thr)
				continue;
		}

		// backward line depth linearity condition
		int val_backward = *(ptrs[ind1] - shift);
		if (val_backward != 0)
		{
			int gradient_backward = val1 - val_backward;
			if (abs(val2 - val1 - gradient_backward) < depth_thr)
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


vector<TriangleIndexes> MeshGenerator::generateTrianglesGradients(UINT16 *depth_image, vector<int> &depth_to_vertices_map, int ndepth_frame_width, int ndepth_frame_height)
{
	int n_depth_pixels = ndepth_frame_height * ndepth_frame_width;
	vector<TriangleIndexes> indexes(n_depth_pixels * 2);
	int w = ndepth_frame_width;
	int h = ndepth_frame_height;

	int n_triangles = 0;

	const int pixel_shifts[] = { -w, -w + 1, 1 };
	const int n_pixel_shifts = 4;

	const int triangles_shifts[] = { 0,				  pixel_shifts[0], pixel_shifts[2],
									 pixel_shifts[2], pixel_shifts[0], pixel_shifts[1],
									 0,				  pixel_shifts[0], pixel_shifts[1],
									 0,				  pixel_shifts[1], pixel_shifts[2]};	

	for (int y = 2; y < ndepth_frame_height - 2; y++)
	{
		UINT16 *depth_row = depth_image + y * ndepth_frame_width;
		int *map_row = depth_to_vertices_map.data() + y * ndepth_frame_width;
		for (int x = 1; x < ndepth_frame_width - 2; x++)
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

	return indexes;
}
