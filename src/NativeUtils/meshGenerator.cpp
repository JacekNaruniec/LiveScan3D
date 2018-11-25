#include "meshGenerator.h"
#include <chrono>
#include <algorithm>
#include <thread>
#include "SimpleImage.h"
#include "point3f.h"
#include "depthmaputils.h"

using namespace std;

//#define SAVE_INTERMEDIATE_IMAGES

#include "simpletimer.h"

// thresholds set according to Wasenmuller: "Comparison of Kinect v1 and v2 Depth Images in Terms of Accuracy and Precision"

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

	const unsigned short depth_thr = getDepthThreshold((unsigned short)((vals[0] + vals[1] + vals[2]) / 3.0));
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
void MeshGenerator::generateTrianglesGradientsRegion(unsigned short *depth_image, vector<TriangleIndexes> *indexes, 
	int ndepth_frame_width, int ndepth_frame_height, int minX, int minY, int maxX, int maxY, int *out_n_triangles)
{
	int n_depth_pixels = (maxX - minX) * (maxY - minY);

	int w = ndepth_frame_width;
	int h = ndepth_frame_height;
	minX = max(minX, 1);
	minY = max(minY, 2);
	maxX = min(maxX, ndepth_frame_width - 2);
	maxY = min(maxY, ndepth_frame_height - 2);

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
					
					(*indexes)[n_triangles].ind[0] = map1;
					(*indexes)[n_triangles].ind[1] = map2;
					(*indexes)[n_triangles].ind[2] = map3;

					if (depth_image[map1] == 0 || depth_image[map2] == 0 || depth_image[map3] == 0)
						continue;

					n_triangles++;
				}

		}
	}
	*out_n_triangles = n_triangles;
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

inline unsigned short MeshGenerator::getDepthThreshold(unsigned short depth)
{
	return(unsigned short)(max(20, depth * 0.03));
}


bool MeshGenerator::checkDepthPointConstraints(int current_map_index, int x, int y)
{
	int w = dm_widths[current_map_index];
	int pos = x + y * w;
	size_t n_maps = current_depth_maps.size(); 
	unsigned char confidence = depth_map_confidence[current_map_index][x + y * w];

	for (int i = 0; i < n_maps; i++)
	{
		if (i == current_map_index)
			continue; 

		int dst_d = map_correspondences[current_map_index][i].dst_d[pos];

		if (dst_d == 0)
			continue; 

		int dst_x = map_correspondences[current_map_index][i].x[pos];
		int dst_y = map_correspondences[current_map_index][i].y[pos];
		int d_mapped = map_correspondences[current_map_index][i].org_d_mapped[pos];

		if (current_depth_maps[i][dst_x + dst_y * dm_widths[i]] == 0)
			continue; 

		// check optimality
		int dest_confidence = depth_map_confidence[i][dst_y * w + dst_x];

		unsigned short depth_threshold = getDepthThreshold(dst_d);

		bool points_close = abs(dst_d - d_mapped) < 20;


		if (dst_d > d_mapped + depth_threshold || dst_d == 0)
			return false;

		if (dest_confidence < 3)
			continue;

		if (points_close && confidence < dest_confidence)
			return false;
	}

	return true; 
}


void MeshGenerator::applyAllColorCorrections()
{
	size_t n_tranforms = color_correction_params.size();
	const int n_threads = 4;
	vector<thread> threads(n_threads);

	for (size_t transf_ind = 0; transf_ind < n_tranforms; transf_ind++)
	{
		int target_map_index = color_correction_params[transf_ind].map_index;
		vector<RGB> &colors = current_color_maps[target_map_index];

		size_t step = colors.size() / n_threads + 1;
		size_t pos = 0;
		for (int i = 0; i < n_threads; i++)
		{
			size_t n_elems = min(step, colors.size() - pos);
			threads[i] = thread(applyColorCorrectionRGB, colors.data() + pos, n_elems, color_correction_params[transf_ind]);
			pos += n_elems; 
		}
		for (int i = 0; i < n_threads; i++)
			threads[i].join(); 
	}
}


void MeshGenerator::clearAllMapsByOverlapAndFOV()
{
	size_t n_maps = current_depth_maps.size();

	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	{
		int w = dm_widths[current_map_index];
		int h = dm_heights[current_map_index];

		for (int x = 0; x < w; x++)
			for (int y = 0; y < h; y++)
			{
				int pos = x + y *w;

				if (current_depth_maps[current_map_index][pos] == 0)
					continue; 

				bool pointValid = checkDepthPointConstraints(current_map_index, x, y);

				if (!pointValid)
					current_depth_maps[current_map_index][pos] = 0;
			}
	}
}

ColorCorrectionParams MeshGenerator::getColorCorrectionTransform(int index1, int index2)
{
	ColorCorrectionParams params; 
	int n_mapped_pixels = map_correspondences[index1][index2].n_mapped_pixels;
	vector<unsigned char> pixels1(3 * n_mapped_pixels), pixels2(3 * n_mapped_pixels); 
	int w = dm_widths[index1];
	int h = dm_heights[index2];
	
	//writeDepthImageHSV(current_depth_maps[index1], w, h, "test//map1.png");
	//writeDepthImageHSV(current_depth_maps[index2], w, h, "test//map2.png");
	/*SimpleImage im; 
	im.create(w, h, 3, (unsigned char*)current_color_maps[index1].data());
	im.writeToFile("test//map1_colors.png");
	im.create(w, h, 3, (unsigned char*)current_color_maps[index2].data());
	im.writeToFile("test//map2_colors.png");
	*/


	int n_added = 0;
	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
			{
			int index1_pos = x + y * w; 
			if (current_depth_maps[index1][index1_pos] == 0 || map_correspondences[index1][index2].org_d_mapped[index1_pos] == 0)
				continue; 

			pixels1[n_added]     = current_color_maps[index1][index1_pos].R;
			pixels1[n_added + 1] = current_color_maps[index1][index1_pos].G;
			pixels1[n_added + 2] = current_color_maps[index1][index1_pos].B;
			int index2_pos = map_correspondences[index1][index2].x[index1_pos] + map_correspondences[index1][index2].y[index1_pos] * w;
			unsigned short mapped_d = map_correspondences[index1][index2].org_d_mapped[index1_pos];
			unsigned short dest_d = current_depth_maps[index2][index2_pos];
			unsigned short thr = getDepthThreshold(mapped_d);
			
			if (dest_d == 0 || mapped_d - dest_d > thr)
				continue;

			int x2 = index2_pos % w;
			int y2 = index2_pos / w;

			pixels2[n_added] = current_color_maps[index2][index2_pos].R;
			pixels2[n_added + 1] = current_color_maps[index2][index2_pos].G;
			pixels2[n_added + 2] = current_color_maps[index2][index2_pos].B;
			
			if ((pixels2[n_added] == 0 && pixels2[n_added + 1] == 0 && pixels2[n_added + 2] == 0) ||
				(pixels1[n_added] == 0 && pixels1[n_added + 1] == 0 && pixels1[n_added + 2] == 0))
				continue; 

			n_added += 3;
		}
	pixels1.resize(n_added);
	pixels2.resize(n_added);
	params = getColorCorrectionTransformForPoints(pixels1, pixels2);
	params.base_map_index = index1;
	params.map_index = index2;

	return params; 
}

void MeshGenerator::formVerticesChunksOnly(Mesh &mesh, MeshChunks &mesh_chunks)
{
	const int chunk_size_limit = 65000;
	clearMeshChunks(mesh_chunks);
	int n_vertices = mesh.nVertices;

	mesh_chunks.vertices = new VertexC4ubV3f[n_vertices];
	memcpy(mesh_chunks.vertices, mesh.vertices, n_vertices * sizeof(mesh.vertices[0]));
	int n_chunks = n_vertices / chunk_size_limit + 1;
	int curr_vertex = 0;
	mesh_chunks.vertices_chunk_sizes = new int[n_chunks]();
	mesh_chunks.triangles_chunk_sizes = new int[n_chunks]();
	int n = 0;

	while (curr_vertex < n_vertices)
	{
		int n_to_add = min(chunk_size_limit, n_vertices - curr_vertex);
		mesh_chunks.vertices_chunk_sizes[n] = n_to_add;
		curr_vertex += n_to_add;
		n++;
	}
	mesh_chunks.n_chunks = n_chunks;
}

void MeshGenerator::formMeshChunks(Mesh &mesh, MeshChunks &mesh_chunks)
{
	const int chunk_size_limit = 65000;

	int n_vertices = mesh.nVertices;
	int n_triangles = mesh.nTriangles;

	if (n_triangles == 0 && n_vertices != 0)
	{
		formVerticesChunksOnly(mesh, mesh_chunks);
		return;
	} 

	clearMeshChunks(mesh_chunks);

	if (n_triangles == 0 && n_vertices == 0)
		return; 

	int max_n_chunks = (n_triangles * 3) / chunk_size_limit + 2;

	mesh_chunks.triangles_chunk_sizes = new int[max_n_chunks]();
	mesh_chunks.vertices_chunk_sizes = new int[max_n_chunks]();


	resizeIfNeeded(n_vertices * 2, n_vertices, data.mc_chunk_index);
	resizeIfNeeded(n_vertices * 2, n_vertices, data.mc_vertices_map);

	std::fill(data.mc_chunk_index.begin(), data.mc_chunk_index.end(), -1);

	vector<int> &chunk_index = data.mc_chunk_index;
	vector<int> &vertices_map = data.mc_vertices_map;
	VertexC4ubV3f* new_vertices = new VertexC4ubV3f[n_triangles * 3];
	int* new_triangles = new int[n_triangles * 3];
	int triangles_chunk_start = 0;
	int current_chunk_index = 0;
	int current_vertex = 0;
	int vertices_in_current_chunk = 0;
	int *triangles = mesh.triangles;
	VertexC4ubV3f *vertices = mesh.vertices;

	for (int t = 0; t < n_triangles * 3; t++)
	{
		int val = triangles[t];
		if (chunk_index[val] != current_chunk_index)
		{
			new_vertices[current_vertex] = vertices[val];
			vertices_map[val] = vertices_in_current_chunk;
			chunk_index[val] = current_chunk_index;
			current_vertex++;
			new_triangles[t] = vertices_in_current_chunk;
			vertices_in_current_chunk++;
		}
		else
			new_triangles[t] = vertices_map[val];

		if (vertices_in_current_chunk >= chunk_size_limit && (((t + 1) % 3) == 0))
		{
			mesh_chunks.triangles_chunk_sizes[current_chunk_index] = (t - triangles_chunk_start + 1) / 3;
			mesh_chunks.vertices_chunk_sizes[current_chunk_index] = vertices_in_current_chunk;
			current_chunk_index++;
			vertices_in_current_chunk = 0;
			triangles_chunk_start = t;
		}
	}

	if (vertices_in_current_chunk != 0)
	{
		mesh_chunks.vertices_chunk_sizes[current_chunk_index] = vertices_in_current_chunk;
		mesh_chunks.triangles_chunk_sizes[current_chunk_index] = ((n_triangles * 3 - triangles_chunk_start) / 3);
		current_chunk_index++;
	}

	mesh_chunks.n_chunks = current_chunk_index;
	mesh_chunks.vertices = new_vertices;
	mesh_chunks.triangles = new_triangles; 


}



void MeshGenerator::updateColorCorrectionCoefficients()
{
	color_correction_params.clear();
	size_t n_maps = current_depth_maps.size();
	vector<vector<int>> coverage(n_maps, vector<int>(n_maps, 0));

	vector<bool> colors_assigned(n_maps, false);
	int coverage_threshold = 1000;

	for (int map1_index = 0; map1_index < n_maps; map1_index++)
	{
		for (int map2_index = map1_index + 1; map2_index < n_maps; map2_index++)
		{
			if (map1_index == map2_index)
				continue;

			coverage[map1_index][map2_index] = map_correspondences[map1_index][map2_index].n_mapped_pixels;
			coverage[map2_index][map1_index] = coverage[map1_index][map2_index];
		}
	}

	bool no_more_to_assign = false;
	while (!no_more_to_assign)
	{
		no_more_to_assign = true;
		int max_val = 0;
		int max_val_index_1;
		int max_val_index_2;

		// find connection with already assigned camera (map1_index have to be assigned, the second one - not)
		for (int map1_index = 0; map1_index < n_maps; map1_index++)
			for (int map2_index = 0; map2_index < n_maps; map2_index++)
			{
				if (map1_index == map2_index || colors_assigned[map2_index] || !colors_assigned[map1_index])
					continue;

				if (coverage[map1_index][map2_index] > max_val)
				{
					max_val = coverage[map1_index][map2_index];
					max_val_index_1 = map1_index;
					max_val_index_2 = map2_index;
				}
			}

		// if didn't found any pair with already connected, find any other suitable pair
		if (max_val == 0)
		{
			for (int map1_index = 0; map1_index < n_maps; map1_index++)
				for (int map2_index = map1_index + 1; map2_index < n_maps; map2_index++)
					if (coverage[map1_index][map2_index] > max_val && !colors_assigned[map1_index] && !colors_assigned[map2_index])
					{
						max_val = coverage[map1_index][map2_index];
						max_val_index_1 = map1_index;
						max_val_index_2 = map2_index;
					}
		}
		
		if (max_val > coverage_threshold)
		{
			ColorCorrectionParams transform;
			
			transform = getColorCorrectionTransform(max_val_index_1, max_val_index_2);

			colors_assigned[max_val_index_1] = true;
			colors_assigned[max_val_index_2] = true;

			color_correction_params.push_back(transform);
			no_more_to_assign = false;
		}
	}
}

bool MeshGenerator::findEdgeConnection(int x, int y, int first_depth_map_index, int sec_depth_map_index,
	unsigned char *sec_edge_map, Connection &out_c)
{
	int w = dm_widths[first_depth_map_index];
	int h = dm_heights[first_depth_map_index];
	int pos = x + y *w;
	unsigned short src_d = current_depth_maps[first_depth_map_index][pos];
	int out_x, out_y;
	unsigned short out_d;
	const int n_shifts = 9;
	const int shifts_x[] = { 0, -1,  0,  1, 1, 1, 0, -1, -1 };
	const int shifts_y[] = { 0, -1, -1, -1, 0, 1, 1,  1,  0 };

	const int depth_threshold = getDepthThreshold(src_d);

	vector<unsigned short> &sec_depth_map = current_depth_maps[sec_depth_map_index];

	auto &correspodences = map_correspondences[first_depth_map_index][sec_depth_map_index];

	out_x = correspodences.x[pos];
	out_y = correspodences.y[pos];
	out_d = correspodences.org_d_mapped[pos];

	if (out_x < 1 || out_y < 1 || out_x >= dm_widths[sec_depth_map_index] - 1 || out_y >= dm_heights[sec_depth_map_index] - 1)
		return false;

	for (int shift = 0; shift < n_shifts; shift++)
	{
		int cur_x = out_x + shifts_x[shift];
		int cur_y = out_y + shifts_y[shift];

		if (abs(out_d - sec_depth_map[cur_x + cur_y * w]) < depth_threshold && sec_edge_map[cur_x + cur_y * w] == 255)
		{
			out_c.point1_index = x + y * w;
			out_c.point2_index = cur_x + cur_y * w;
			out_c.depth1 = out_d;
			out_c.depth2 = sec_depth_map[cur_x + cur_y * w];
			return true;
		}
	}

	return false;
}

void MeshGenerator::createTrianglesForSegment(int seed_x, int seed_y, int w, int h, unsigned short *first_depth_map, unsigned short *second_depth_map,
	unsigned char *first_edge_map, unsigned char *second_edge_map, Connection &c_1, vector<unsigned char> &points_visited, int depth_map_index1, int depth_map_index2,
	TriangleIndexes *indexes, int &n_triangles)
{
	const int n_shifts = 8;
	const int shifts_x[] = { -1,  0,  1, 1, 1, 0, -1, -1 };
	const int shifts_y[] = { -1, -1, -1, 0, 1, 1,  1,  0 };
	int depth_treshold;
	int n_pos = 1;
	int n_new_pos = 0;
	vector<Connection> connections, new_connections;
	connections.reserve(w);
	new_connections.reserve(w);
	connections.push_back(c_1);
	n_triangles = 0;

	while (n_pos != 0)
	{

		for (int n = 0; n < n_pos; n++)
		{
			if (points_visited[connections[n].point1_index])
				continue;

			points_visited[connections[n].point1_index] = true;
			int depth_val = first_depth_map[connections[n].point1_index];
			int pos_x = connections[n].point1_index % w;
			int pos_y = connections[n].point1_index / w;

			depth_treshold = getDepthThreshold(depth_val);

			for (int shift = 0; shift < n_shifts; shift++)
			{
				int new_x = pos_x + shifts_x[shift];
				int new_y = pos_y + shifts_y[shift];
				if (new_x < 0 || new_y < 0 || new_x >= w || new_y >= h || first_edge_map[new_x + new_y * w] == 0 ||
					points_visited[new_x + new_y * w])
					continue;

				int dist = abs(depth_val - first_depth_map[new_x + new_y * w]);
				if (dist > depth_treshold)
					continue;

				Connection c_2;

				if (!findEdgeConnection(new_x, new_y, depth_map_index1, depth_map_index2, second_edge_map, c_2))
					continue;
				c_1 = connections[n];

				bool tr[4] = { false, false, false, false };
				tr[0] = (abs(c_1.depth1 - c_2.depth2) < depth_treshold) && (abs(c_1.depth1 - c_2.depth1) < depth_treshold);
				tr[1] = (abs(c_1.depth1 - c_2.depth2) < depth_treshold) && (abs(c_1.depth1 - c_1.depth2) < depth_treshold);

				if (!tr[0] && !tr[1])
				{
					tr[2] = (abs(c_1.depth1 - c_2.depth1) < depth_treshold) && (abs(c_2.depth1 - c_1.depth2) < depth_treshold);
					tr[3] = (abs(c_2.depth1 - c_1.depth2) < depth_treshold) && (abs(c_2.depth2 - c_1.depth2) < depth_treshold);
				}


				if (tr[0]) indexes[n_triangles++].set(c_1.point1_index, c_2.point2_index, c_2.point1_index,
					depth_map_index1, depth_map_index2, depth_map_index1);
				if (tr[1]) indexes[n_triangles++].set(c_1.point1_index, c_2.point2_index, c_1.point2_index,
					depth_map_index1, depth_map_index2, depth_map_index2);

				if (tr[2]) indexes[n_triangles++].set(c_1.point1_index, c_2.point1_index, c_1.point2_index,
					depth_map_index1, depth_map_index1, depth_map_index2);

				if (tr[3]) indexes[n_triangles++].set(c_2.point1_index, c_2.point2_index, c_1.point2_index,
					depth_map_index1, depth_map_index2, depth_map_index2);

				new_connections.push_back(c_2);
			}
		}
		connections = new_connections;
		new_connections.clear();
		n_pos = (int)connections.size();
	}
}



void MeshGenerator::generateTrianglesForStiches(vector<TriangleIndexes> &triangles)
{

	int n_maps = (int)current_depth_maps.size();
	vector<int> n_edges(n_maps);
	int n_total_edges = 0; 

	resizeIfNeeded(n_maps, n_maps, data.edges, true);
	vector<thread> threads(n_maps);
	for (int cur_map = 0; cur_map < n_maps; cur_map++)
	{
		resizeIfNeeded(dm_widths[cur_map] * dm_heights[cur_map], dm_widths[cur_map] * dm_heights[cur_map], data.edges[cur_map], true);
		
		threads[cur_map] = thread(getEdges, dm_widths[cur_map], dm_heights[cur_map], 
			current_depth_maps[cur_map].data(), std::ref(n_edges[cur_map]), std::ref(data.edges[cur_map]));
	}
	for (int cur_map = 0; cur_map < n_maps; cur_map++)
	{
		threads[cur_map].join();
		n_total_edges += n_edges[cur_map];
	}

	resizeIfNeeded(n_total_edges * 8, n_total_edges * 4, triangles);

	vector<Connection> connections;
	int n_triangles = 0, n_total_triangles = 0;

	for (int first_map = 0; first_map < n_maps; first_map++)
	{
		unsigned short *first_depth_map = current_depth_maps[first_map].data();
		unsigned char *first_edge_map = data.edges[first_map].data();
		int w = dm_widths[first_map];
		int h = dm_heights[first_map];

		vector<unsigned char> points_visited(w*h, 0);

		for (int second_map = 0; second_map < n_maps; second_map++)
		{
			if (first_map == second_map)
				continue;

			unsigned short *second_depth_map = current_depth_maps[second_map].data();
			unsigned char *second_edge_map = data.edges[second_map].data();

			for (int y = 0; y < h; y++)
				for (int x = 0; x < w; x++)
				{
					if (points_visited[x + y*w] || first_edge_map[x + y*w] != 255)
						continue;

					Connection connection;
					
					if (!findEdgeConnection(x, y, first_map, second_map, second_edge_map, connection))
						continue;

					createTrianglesForSegment(x, y, w, h, first_depth_map, second_depth_map,
						first_edge_map, second_edge_map, connection, points_visited, first_map, second_map,
						triangles.data() + n_total_triangles, n_triangles);
					n_total_triangles += n_triangles;
				}
		}
	}

	triangles.resize(n_total_triangles);
}

void MeshGenerator::setVertex(VertexC4ubV3f &v,  const int x, const int y, const unsigned short depth, ExtrinsicCameraParameters &ep, IntrinsicCameraParameters &ip)
{
	Point3f p;
	p.Z = depth / 1000.0f;
	p.X = ((x - ip.cx) / ip.fx) * p.Z + ep.t[0];
	p.Y = ((ip.cy - y) / ip.fy) * p.Z + ep.t[1];
	p.Z += ep.t[2];

	p.rotate(ep.R);
	v.X = p.X;
	v.Y = p.Y;
	v.Z = p.Z; 
}

void MeshGenerator::pointProjection(const Point3f p, int &x, int &y, unsigned short &d, ExtrinsicCameraParameters &ep, IntrinsicCameraParameters &ip)
{
	Point3f tmp = p;
	tmp.rotate(ep.R);
	tmp.X += ep.t[0];
	tmp.Y += ep.t[1];
	tmp.Z += ep.t[2];

	x = static_cast<int>((tmp.X * ip.fx) / tmp.Z + ip.cx + 0.5);
	y = static_cast<int>(ip.cy - (tmp.Y * ip.fy) / tmp.Z + 0.5);
	d = static_cast<unsigned short>(min(max(0, (int)(tmp.Z * 1000.0f)), 65535));
}

void MeshGenerator::setVerticesThreadFunction(int min_vertex, int max_vertex, 
	const vector<int> &vertex_depth_map_pos, const vector<int> &vertex_depth_map_index,
	vector<int> dm_widths, vector<int> dm_heights, VertexC4ubV3f *vertices,
	vector<IntrinsicCameraParameters> ip, vector<ExtrinsicCameraParameters> ep_inv)
{
	for (int v = min_vertex; v < max_vertex; v++)
	{
		int pos = vertex_depth_map_pos[v];
		int map = vertex_depth_map_index[v];
		int x = pos % dm_widths[map];
		int y = pos / dm_widths[map];
		RGB &c = current_color_maps[map][pos];
		unsigned short d = current_depth_maps[map][pos];
		VertexC4ubV3f &vertex = vertices[v];
		setVertex(vertex, x, y, d, ep_inv[map], ip[map]);
		vertex.A = 255;
		vertex.R = c.R;
		vertex.G = c.G;
		vertex.B = c.B;
	}
}

void MeshGenerator::formMesh(Mesh *mesh, std::vector<std::vector<TriangleIndexes>> &triangle_indexes)
{
	size_t n_sets = triangle_indexes.size(); 

	for (int i=0; i<3; i++)
		std::fill(data.vertices_map[i].begin(), data.vertices_map[i].end(), -1);

	int n_triangles_inds = 0;
	int total_vertices_indexes = 0;
	vector<ExtrinsicCameraParameters> ep_inv = extrinsic_params; 
	vector<IntrinsicCameraParameters> ip = intrinsic_params;
	for (int i = 0; i < ep_inv.size(); i++)
		ep_inv[i].inv(); 

	int n_total_triangles = 0;
	for (int curr_set_index = 0; curr_set_index < n_sets; curr_set_index++)
		n_total_triangles += (int)triangle_indexes[curr_set_index].size();

	if (mesh->nTriangles * 2 < n_total_triangles)
	{
		if (mesh->nTriangles != 0)
			delete[]mesh->triangles;

		mesh->triangles = new int[n_total_triangles * 3 * 2];
	}

	mesh->nTriangles = (int)n_total_triangles;


	resizeIfNeeded((int)(n_total_triangles * 3 * 2.0), (int)(n_total_triangles * 3), data.vertex_depth_map_pos);
	resizeIfNeeded((int)(n_total_triangles * 3 * 2.0), (int)(n_total_triangles * 3), data.vertex_depth_map_index);

	for (int curr_set_index = 0; curr_set_index < n_sets; curr_set_index++)
	{
		// get the number of unique vertices
		auto &current_indexes = triangle_indexes[curr_set_index];

		size_t n_cur_triangles = current_indexes.size();
		for (int curr_triangle_index = 0; curr_triangle_index < n_cur_triangles; curr_triangle_index++)
		{
			for (int i = 0; i < 3; i++)
			{
				int v_ind = current_indexes[curr_triangle_index].ind[i];
				int map_ind;
				if (curr_set_index < n_sets - 1)  
					map_ind = curr_set_index;
				else   // stiches
					map_ind= current_indexes[curr_triangle_index].map_ind[i];

				int vertex_mapped_ind = data.vertices_map[map_ind][v_ind];

				if (vertex_mapped_ind == -1)
				{
					int x = v_ind % dm_widths[map_ind];
					int y = v_ind / dm_heights[map_ind];
					vertex_mapped_ind = total_vertices_indexes;
					data.vertices_map[map_ind][v_ind] = total_vertices_indexes;
					data.vertex_depth_map_pos[total_vertices_indexes] = v_ind;
					data.vertex_depth_map_index[total_vertices_indexes] = map_ind;
					total_vertices_indexes++;
				}

				mesh->triangles[n_triangles_inds] = vertex_mapped_ind;
				n_triangles_inds++;
			}
		}
	}

	if (mesh->nVertices * 2< total_vertices_indexes)
	{
		if (mesh->nVertices != 0)
			delete[]mesh->vertices;

		mesh->vertices = new VertexC4ubV3f[total_vertices_indexes * 2];
	}


	mesh->nVertices = total_vertices_indexes;
	

	vector<thread> threads; 
	const int n_threads = 4; 
	int step = total_vertices_indexes / n_threads + 1; 
	int actual_vertex = 0; 

	for (int t = 0; t < n_threads; t++)
	{
		int min_v = actual_vertex;
		int max_v = min(actual_vertex + step, total_vertices_indexes); 
		threads.push_back(thread(&MeshGenerator::setVerticesThreadFunction, this, 
			min_v, max_v, std::ref(data.vertex_depth_map_pos), std::ref(data.vertex_depth_map_index), dm_widths,
			dm_heights, mesh->vertices, ip, ep_inv));
		actual_vertex = max_v; 
	}


	for (size_t i = 0; i < threads.size(); i++)
		threads[i].join(); 
}

void MeshGenerator::generateTriangles(Mesh *mesh)
{
	size_t n_maps = current_depth_maps.size();
	resizeIfNeeded(n_maps + 1, n_maps + 1, data.triangle_indexes, true);
	int n_threads_per_map = 1;
	vector<vector<int>> n_triangles(n_maps, vector<int>(n_threads_per_map));

	resizeIfNeeded(n_maps * n_threads_per_map, n_maps * n_threads_per_map, data.partial_indexes, false);
	int current_part_index = 0;
	vector<thread> threads(n_maps * n_threads_per_map);

	for (int cur_map = 0; cur_map < n_maps; cur_map++)
	{
		int pos_y = 0;
		int step = dm_heights[cur_map] / n_threads_per_map + 1;
		vector<unsigned short> &depth_map = current_depth_maps[cur_map];
		for (int i = 0; i < n_threads_per_map; i++)
		{
			int size_y = min(step, dm_heights[cur_map] - pos_y);
			int n_depth_pixels = dm_widths[cur_map] * step;

			resizeIfNeeded(n_depth_pixels * 2, n_depth_pixels * 2, data.partial_indexes[current_part_index]);
			
			threads[current_part_index] = thread(&MeshGenerator::generateTrianglesGradientsRegion, this, 
				depth_map.data(), &data.partial_indexes[current_part_index], dm_widths[cur_map], dm_heights[cur_map], 
				0, pos_y, dm_widths[cur_map], min(pos_y + size_y, dm_heights[cur_map]), &n_triangles[cur_map][i]);
				
			pos_y += size_y;
			current_part_index++;
		}
	}

	generateTrianglesForStiches(data.triangle_indexes[n_maps]);

	for (size_t i = 0; i < threads.size(); i++)
		threads[i].join();

	vector<int> n_triangles_per_map(n_maps, 0);
	for (int cur_map = 0; cur_map < n_maps; cur_map++)
		for (int i = 0; i < n_threads_per_map; i++)
			n_triangles_per_map[cur_map] += n_triangles[cur_map][i];
	
	current_part_index = 0;

	for (int cur_map = 0; cur_map < n_maps; cur_map++)
	{
		int pos = 0;
		resizeIfNeeded(n_triangles_per_map[cur_map] * 2, n_triangles_per_map[cur_map], data.triangle_indexes[cur_map]);
		data.triangle_indexes[cur_map].resize(n_triangles_per_map[cur_map]);
		for (int i = 0; i < n_threads_per_map; i++)
		{
			if (n_triangles[cur_map][i] == 0)
			{
				current_part_index++;
				continue;
			}

			memcpy(data.triangle_indexes[cur_map].data() + pos, data.partial_indexes[current_part_index].data(), n_triangles[cur_map][i] * sizeof(data.partial_indexes[current_part_index][0]));
			pos += n_triangles[cur_map][i];
			current_part_index++;
		}
	}

	formMesh(mesh, data.triangle_indexes);
}

void MeshGenerator::reprojectionCorrection(Mesh *mesh)
{
	int n_vertices = mesh->nVertices;
	size_t n_maps = current_depth_maps.size(); 
	unsigned short best_confidence;
	size_t best_index;
	int best_x, best_y, dest_depth; 
	int n_matches; 
	vector<ExtrinsicCameraParameters> ep_inv = extrinsic_params;
	for (size_t i = 0; i < n_maps; i++)
		ep_inv[i].inv(); 

	Point3f p;

	for (int v = 0; v < n_vertices; v++)
	{
		VertexC4ubV3f &vertex = mesh->vertices[v];
		p.X = vertex.X;
		p.Y = vertex.Y;
		p.Z = vertex.Z;
		int x, y; 
		unsigned short d; 
		best_confidence = 0;
		n_matches = 0;
		for (size_t i = 0; i < n_maps; i++)
		{
			int w = dm_widths[i];
			int h = dm_heights[i];
			pointProjection(p, x, y, d, extrinsic_params[i], intrinsic_params[i]);
			if (d == 0 || x < 0 || y < 0 || x >= w || y >= h)
				continue;
			int pos = x + y*w; 

			unsigned short org_depth = current_depth_maps[i][pos];

			if (abs(d - org_depth) > 20)
				continue;

			if (depth_map_confidence[i][pos] > best_confidence)
			{
				best_confidence = depth_map_confidence[i][pos];
				best_index = i;
				best_x = x;
				best_y = y; 
				dest_depth = d;
				n_matches++;
			}
		}

		if (n_matches > 1)
		{
			RGB &c = current_color_maps[best_index][best_x + best_y * dm_widths[best_index]];
			vertex.R = c.R; 
			vertex.G = c.G; 
			vertex.B = c.B;

			setVertex(vertex, best_x, best_y, dest_depth, ep_inv[best_index], intrinsic_params[best_index]);
		}
	}


}
/*
void MeshGenerator::reprojectionCorrection()
{
	int n_maps = (int)current_depth_maps.size();

	for (size_t map_index = 0; map_index < n_maps; map_index++)
	{
		int w = dm_widths[map_index]; 
		int h = dm_heights[map_index]; 
		int size = w*h;
		for (int i = 0; i < size; i++)
		{
			int x = i % w;
			int y = i / w;
			if (x<1 || y<1 || x > w - 2 || y > y - 2)
				continue; 

			if (current_depth_maps[map_index][i] == 0)
				continue; 

			int confidence = depth_map_confidence[map_index][i];
			if (confidence > 19)
				continue; 
			
			int best_confidence = confidence;
			size_t best_map_index = map_index;
			int best_pos = i;

			for (size_t map_index2 = 0; map_index2 < n_maps; map_index2++)
			{
				if (map_index == map_index2) continue; 

				auto &map_correspondence = map_correspondences[map_index][map_index2];
				
				unsigned short mapped_d = map_correspondence.org_d_mapped[i];
				unsigned short dst_d = map_correspondence.dst_d[i];
				
				if (dst_d == 0 || abs(mapped_d - dst_d) > 20)
					continue; 

				int dst_pos = map_correspondence.x[i] + map_correspondence.y[i] * dm_widths[map_index2];
				int dst_confidence = depth_map_confidence[map_index2][dst_pos];

				if (dst_confidence > confidence)
				{
					best_pos = dst_pos; 
					best_confidence = dst_confidence;
					best_map_index = map_index2; 
				}
			}

			if (best_map_index != map_index)
				current_depth_maps[map_index][i] = map_correspondences[best_map_index][map_index].org_d_mapped[best_pos];
		}

	}
}
*/
void MeshGenerator::generateDepthMapsConfidence()
{
	size_t n_maps = current_depth_maps.size();

	const int et_limit = 20;
	int depth_threshold = 20;
	depth_map_confidence.resize(n_maps);	vector<thread> threads;

	for (int i = 0; i < n_maps; i++)
		threads.push_back(thread(&MeshGenerator::generateMapConfidence, this, std::ref(current_depth_maps[i]), std::ref(depth_map_confidence[i]), dm_widths[i], dm_heights[i], et_limit, depth_threshold));
	
	for (int i = 0; i < threads.size(); i++)
		threads[i].join();
}

void MeshGenerator::initializeVariables(int n_maps, unsigned char* depth_maps, unsigned char *color_maps, float *intr_params, float *extr_params,
	 int *widths, int *heights)
{
	if (current_depth_maps.size() != n_maps)
	{
		current_depth_maps.resize(n_maps);
		current_color_maps.resize(n_maps);
		dm_widths.resize(n_maps);
		dm_heights.resize(n_maps);
		intrinsic_params.resize(n_maps);
		extrinsic_params.resize(n_maps);
	}

	int depth_map_pos = 0;
	int color_map_pos = 0;

	for (int i = 0; i < n_maps; i++)
	{
		intrinsic_params[i] = IntrinsicCameraParameters(intr_params + i * 7);
		extrinsic_params[i] = ExtrinsicCameraParameters(extr_params + i*(9 + 3));
		extrinsic_params[i].inv();

		if (dm_widths[i] != widths[i] || dm_heights[i] != heights[i])
		{
			dm_widths[i] = widths[i];
			dm_heights[i] = heights[i];
			current_depth_maps[i].resize(widths[i] * heights[i]);
			current_color_maps[i].resize(widths[i] * heights[i]);
		}
		
		memcpy(current_depth_maps[i].data(), depth_maps + depth_map_pos, widths[i] * heights[i] * 2);
		memcpy(current_color_maps[i].data(), color_maps + color_map_pos, widths[i] * heights[i] * 3);

		depth_map_pos += widths[i] * heights[i] * 2;
		color_map_pos += widths[i] * heights[i] * 3;

	}

	int max_w = *std::max_element(dm_widths.begin(), dm_widths.end());
	int max_h = *std::max_element(dm_heights.begin(), dm_heights.end());
	int max_size = max_w * max_h;

	if (data.vertices_map.size() != 3 || data.vertices_map[0].size() != max_size)
		data.vertices_map = vector<vector<int>>(3, vector<int>(max_size, -1));
}


rot_matrix MeshGenerator::multiply3x3Matrices(rot_matrix &M1, rot_matrix &M2)
{
	rot_matrix outM(3, vector<float>(3, 0.0f));

	for (int j = 0; j < 3; j++) // column
		for (int i = 0; i < 3; i++)		// row
			for (int k = 0; k < 3; k++)
				outM[i][j] += M1[i][k] * M2[k][j];
	return outM;
}

void MeshGenerator::generateCorrespondencesForMap(int base_map)
{
	unsigned short *depth_map = current_depth_maps[base_map].data();
	IntrinsicCameraParameters in_ip = intrinsic_params[base_map];
	int w = dm_widths[base_map];
	int h = dm_heights[base_map];
	size_t n_maps = current_depth_maps.size(); 

	vector<rot_matrix> rot_matrices(n_maps);

	ExtrinsicCameraParameters base_ex_params_inv = extrinsic_params[base_map];
	base_ex_params_inv.inv();

	for (size_t dest_map = 0; dest_map < n_maps; dest_map++)
	{
		if (dest_map == base_map) continue;
		rot_matrices[dest_map] = multiply3x3Matrices(extrinsic_params[dest_map].R, base_ex_params_inv.R);
	}

	for (int y = 0; y < h; y++)
		for (int x = 0; x < w; x++)
		{
			int pos = x + y*w;
			unsigned short d = depth_map[pos];
			if (d == 0)
				continue;

			Point3f p;
			p.Z = d / 1000.0f;
			p.X = ((x - in_ip.cx) / in_ip.fx) * p.Z + base_ex_params_inv.t[0];
			p.Y = ((in_ip.cy - y) / in_ip.fy) * p.Z + base_ex_params_inv.t[1];
			p.Z += base_ex_params_inv.t[2];

			for (size_t dest_map = 0; dest_map < n_maps; dest_map++)
			{
				int dst_w = dm_widths[dest_map];
				int dst_h = dm_heights[dest_map];
				Point3f p_act = p;
				if (base_map == dest_map)
					continue;

				MapCorrespondence &mc = map_correspondences[base_map][dest_map];

				p_act.rotate(rot_matrices[dest_map]);

				p_act.X += extrinsic_params[dest_map].t[0];
				p_act.Y += extrinsic_params[dest_map].t[1];
				p_act.Z += extrinsic_params[dest_map].t[2];

				float out_x, out_y;
				unsigned short out_d;
				out_x = ((p_act.X * intrinsic_params[dest_map].fx) / p_act.Z + intrinsic_params[dest_map].cx + 0.5f);
				out_y = (intrinsic_params[dest_map].cy - (p_act.Y * intrinsic_params[dest_map].fy) / p_act.Z + 0.5f);
				out_d = static_cast<unsigned short>(min(max(0, (int)(p_act.Z * 1000.0f)), 65535));
				if (out_d == 0 || out_x < 0 || out_y < 0 || out_x >= dst_w || out_y >= dst_h)
					continue;

				mc.x[pos] = static_cast<int>(out_x);
				mc.y[pos] = static_cast<int>(out_y);
				mc.org_d_mapped[pos] = out_d;
				mc.dst_d[pos] = static_cast<unsigned short>(getSubpixelValue(current_depth_maps[dest_map], out_x, out_y, w, h));

				mc.n_mapped_pixels++;
			}
		}
}

void MeshGenerator::generateAllMapsCorrespondences()
{
	size_t n_maps = current_depth_maps.size(); 
	map_correspondences.resize(n_maps);

	vector<thread> threads(n_maps);

	for (size_t base_map = 0; base_map < n_maps; base_map++)
	{
		int w = dm_widths[base_map];
		int h = dm_heights[base_map];
				
		map_correspondences[base_map].resize(n_maps);

		for (size_t dest_map = 0; dest_map < n_maps; dest_map++)
		{
			if (dest_map == base_map) continue;
			map_correspondences[base_map][dest_map].x.resize(w*h);
			map_correspondences[base_map][dest_map].y.resize(w*h);
			map_correspondences[base_map][dest_map].dst_d.resize(w*h);
			map_correspondences[base_map][dest_map].org_d_mapped.resize(w*h);
			memset(map_correspondences[base_map][dest_map].org_d_mapped.data(), 0, w*h * sizeof(unsigned short));
			memset(map_correspondences[base_map][dest_map].dst_d.data(), 0, w*h * sizeof(unsigned short));
			map_correspondences[base_map][dest_map].n_mapped_pixels = 0;
		}

		threads[base_map] = thread(&MeshGenerator::generateCorrespondencesForMap, this, base_map);
	}

	for (size_t base_map = 0; base_map < n_maps; base_map++)
		threads[base_map].join();
}

void MeshGenerator::limitDepthMapToBounds(int map_index)
{
	int w = dm_widths[map_index];
	int h = dm_heights[map_index];
	unsigned short *depth_map = current_depth_maps[map_index].data();
	IntrinsicCameraParameters &ip = intrinsic_params[map_index];
	ExtrinsicCameraParameters ep = extrinsic_params[map_index];
	ep.inv();

	for (int y = 0; y < h; y++)
	{
		unsigned short *row_ptr = depth_map + y * w;
		for (int x = 0; x < w; x++)
		{
			if (row_ptr[x] == 0)
				continue;

			float val = row_ptr[x];

			Point3f p;
			p.Z = val / 1000.0f;
			p.X = ((x - ip.cx) / ip.fx) * p.Z + ep.t[0];
			p.Y = ((ip.cy - y) / ip.fy) * p.Z + ep.t[1];
			p.Z += ep.t[2];

			p.rotate(ep.R);

			if (p.X<bounds[0] || p.X > bounds[1] || p.Y < bounds[2] || p.Y > bounds[3] || p.Z < bounds[4] || p.Z > bounds[5])
				row_ptr[x] = 0;
		}
	}
}

void MeshGenerator::limitDepthMapsToBounds()
{
	size_t n_maps = current_depth_maps.size(); 
	vector<thread> threads(n_maps); 
	for (size_t i = 0; i < n_maps; i++)
		threads[i] = thread(&MeshGenerator::limitDepthMapToBounds, this, i);

	for (size_t i = 0; i < n_maps; i++)
		threads[i].join(); 
}

void MeshGenerator::generateVerticesOnly(Mesh *mesh)
{
	int n_elements = 0; 
	for (size_t depth_map_ind = 0; depth_map_ind < current_depth_maps.size(); depth_map_ind++)
	{
		int size = dm_widths[depth_map_ind] * dm_heights[depth_map_ind];
		for (int i = 0; i < size; i++)
			if (current_depth_maps[depth_map_ind][i] != 0)
				n_elements++;
	}

	mesh->nVertices = n_elements; 
	mesh->vertices = new VertexC4ubV3f[n_elements]; 
	mesh->nTriangles = 0;
	int actual_vertex = 0;

	for (size_t depth_map_ind = 0; depth_map_ind < current_depth_maps.size(); depth_map_ind++)
	{
		ExtrinsicCameraParameters ep = extrinsic_params[depth_map_ind];
		IntrinsicCameraParameters ip = intrinsic_params[depth_map_ind]; 
		ep.inv(); 

		for (int y=0; y<dm_heights[depth_map_ind]; y++)
			for (int x = 0; x < dm_widths[depth_map_ind]; x++)
			{
				int pos = x + y * dm_widths[depth_map_ind];
				unsigned short d = current_depth_maps[depth_map_ind][pos]; 
				if (d == 0)
					continue; 

				setVertex(mesh->vertices[actual_vertex], x, y, d, ep, ip);
				mesh->vertices[actual_vertex].R = current_color_maps[depth_map_ind][pos].R;
				mesh->vertices[actual_vertex].G = current_color_maps[depth_map_ind][pos].G;
				mesh->vertices[actual_vertex].B = current_color_maps[depth_map_ind][pos].B;
				mesh->vertices[actual_vertex].A = 255;
				actual_vertex++;
			}
	}
}


void MeshGenerator::generateMeshFromDepthMaps(int n_maps, unsigned char* depth_maps, unsigned char *depth_colors, int *widths, int *heights,
	float *intr_params, float *extr_params, Mesh *out_mesh, bool bcolor_transfer, bool bgenerate_triangles, bool data_cleaning)
{
	initializeVariables(n_maps, depth_maps, depth_colors, intr_params, extr_params, widths, heights);

#ifdef SAVE_INTERMEDIATE_IMAGES
	for (int i = 0; i < n_maps; i++)
	{
		char tmp[1024];
		sprintf(tmp, "test/depth_map_%d.png", i);
		writeDepthImageHSV(current_depth_maps[i], widths[i], heights[i], tmp);
		sprintf(tmp, "test/color_map_%d.png", i);
		SimpleImage im; 
		im.create(widths[i], heights[i], 3, (unsigned char*)current_color_maps[i].data());
		im.writeToFile(tmp);
	}
#endif

	limitDepthMapsToBounds();

	generateDepthMapsConfidence();
	generateAllMapsCorrespondences();;

	if (bcolor_transfer)
	{
		updateColorCorrectionCoefficients();
		applyAllColorCorrections();
	}

	if (data_cleaning)
		clearAllMapsByOverlapAndFOV();

	if (bgenerate_triangles)
		generateTriangles(out_mesh);
	else
		generateVerticesOnly(out_mesh);

	reprojectionCorrection(out_mesh);
}
