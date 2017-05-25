#include "depthprocessing.h"
#include "meshGenerator.h"
#include <thread>
#include <unordered_map>
#include <algorithm>
#include "pgm.h"


using namespace std; 

void RotatePoint(Point3f &point, std::vector<std::vector<float>> &R)
{
	Point3f res;

	res.X = point.X * R[0][0] + point.Y * R[0][1] + point.Z * R[0][2];
	res.Y = point.X * R[1][0] + point.Y * R[1][1] + point.Z * R[1][2];
	res.Z = point.X * R[2][0] + point.Y * R[2][1] + point.Z * R[2][2];

	point.X = res.X;
	point.Y = res.Y; 
	point.Z = res.Z; 
}

void createVertices(unsigned short *depth_map, unsigned char *depth_colors, int w, int h, IntrinsicCameraParameters params, WorldTranformation world_transform, VerticesWithDepthColorMaps &vertices_with_maps)
{
	auto &vertices = vertices_with_maps.vertices;
	auto &colors = vertices_with_maps.colors;

	colors.resize(w*h * 3);

	vertices.resize(w*h);
	int n_pixels = w*h; 
	int n_vertices = 0;
	unsigned short *row_ptr;
	int pos; 
	vertices_with_maps.depth_to_vertices_map.resize(n_pixels, -1);
	vertices_with_maps.vertices_to_depth_map.resize(n_pixels, -1);
		
	for (int y = 0; y < h; y++)
	{
		row_ptr = depth_map + y*w;
		for (int x = 0; x < w; x++)
		{
			if (row_ptr[x] != 0)
			{
				pos = x + y * w;
				vertices_with_maps.depth_to_vertices_map[pos] = n_vertices;
				vertices_with_maps.vertices_to_depth_map[n_vertices] = pos;

				float val = row_ptr[x];
				int yp = (int)(params.cy * 2 - y);
				float Z = val / 1000.0f;

				float X = (x - params.cx) / params.fx;
				float Y = (yp - params.cy) / params.fy;
				float r = X*X + Y*Y;

				float d = (1.0f - r*params.r2 - r*r * params.r4 - r*r*r*params.r6);

				X = d * X * Z;
				Y = d * Y * Z;

				vertices[n_vertices].X = X;
				vertices[n_vertices].Y = Y;
				vertices[n_vertices].Z = Z;

				colors[n_vertices * 3] = depth_colors[pos * 3];
				colors[n_vertices * 3 + 1] = depth_colors[pos * 3 + 1];
				colors[n_vertices * 3 + 2] = depth_colors[pos * 3 + 2];

				n_vertices++;
			}
		}
	}

	vertices.resize(n_vertices);
	colors.resize(n_vertices * 3);
	vertices_with_maps.vertices_to_depth_map.resize(n_vertices);
	vertices_with_maps.point_assigned = vector<bool>(n_vertices, false);

	for (auto &v : vertices)
	{
		v.X += world_transform.t[0];
		v.Y += world_transform.t[1];
		v.Z += world_transform.t[2];
		RotatePoint(v, world_transform.R);
	}
}

void generateVerticesFromDepthMaps(unsigned char* depth_maps, unsigned char *depth_colors, int *widths, int *heights,
	vector<WorldTranformation> &world_transforms, vector<IntrinsicCameraParameters> &intrinsic_params, vector<VerticesWithDepthColorMaps> &vertices_with_maps)
{
	int depth_pos = 0, colors_pos = 0;
	vector<thread> threads; 
	int n_maps = (int)vertices_with_maps.size(); 

	for (int i = 0; i < n_maps; i++)
	{
		int n_pixels = widths[i] * heights[i];

		threads.push_back(thread(createVertices, (unsigned short*)(depth_maps + depth_pos), depth_colors + colors_pos, widths[i], heights[i], intrinsic_params[i],
			world_transforms[i], std::ref(vertices_with_maps[i])));
		depth_pos += n_pixels * 2;
		colors_pos += n_pixels * 3;
	}

	for (int i = 0; i < n_maps; i++)
		threads[i].join();
}

void pointProjection(Point3f &p, int &out_x, int &out_y, unsigned short &out_d, WorldTranformation &wt, IntrinsicCameraParameters &ip)
{
	Point3f tmp = p;
	RotatePoint(tmp, wt.R);
	tmp.X += wt.t[0];
	tmp.Y += wt.t[1];
	tmp.Z += wt.t[2];

	out_x = (tmp.X * ip.fx) / tmp.Z + ip.cx;
	out_y = ip.cy - (tmp.Y * ip.fy) / tmp.Z;
	out_d = min(max(0, (int)(tmp.Z * 1000.0f)), 65535);
}

void projectUnassignedVerticesIntoDepthMap(VerticesWithDepthColorMaps &vertices_with_maps, vector<int> &depth_to_vertices_map, WorldTranformation &wt, IntrinsicCameraParameters &ip, int w, int h)
{
	vector<Point3f> &vertices = vertices_with_maps.vertices;
	int n_vertices = (int)vertices.size(); 
	vector<unsigned short> &depth_image = vertices_with_maps.depth_map;

	for (int v = 0; v < n_vertices; v++)
	{
		if (vertices_with_maps.point_assigned[v])
			continue; 
		
		int x, y;
		unsigned short d;
		pointProjection(vertices[v], x, y, d, wt, ip);		
		if (x < 0 || x >= w || y < 0 || y >= h)
			continue;

		depth_image[y * w + x] = d; 
		depth_to_vertices_map[y * w + x] = v;
		vertices_with_maps.point_assigned[v] = true;
	}
}

void writeDepthImage(vector<unsigned short> &depth_image, int w, int h, string filename)
{
	vector<unsigned char> image(h*w, 0);

	for (int i = 0; i < w; i++)
		for (int j = 0; j < h; j++)
			image[i + j*w] = depth_image[j * w + i];

	writePGM(filename.c_str(), w, h, image.data());
}

void assignDepthMapOverlay(vector<unsigned short> &depth_map, VerticesWithDepthColorMaps &vertices_with_maps,
	vector<int> &depth_to_vertices_map, WorldTranformation &wt, IntrinsicCameraParameters &ip, vector<vector<unsigned char>> &current_map_indexes, int index, int w, int h)
{
	const int depth_threshold = 10; 

	vector<Point3f> &vertices = vertices_with_maps.vertices;
	int n_vertices = (int)vertices.size();

	for (int v = 0; v < n_vertices; v++)
	{
		if (vertices_with_maps.point_assigned[v])
			continue;

		int x, y;
		unsigned short d;
		pointProjection(vertices[v], x, y, d, wt, ip);
		if (x < 0 || x >= w || y < 0 || y >= h || d == 0)
			continue;		

		if (abs(d - depth_map[y * w + x]) < depth_threshold || depth_map[y * w + x] == 0)
		{
			depth_map[y * w + x] = d;
			vertices_with_maps.point_assigned[v] = true;
			depth_to_vertices_map[y * w + x] = v;
			current_map_indexes[y][x] = index;
		}
	}
}

VerticesWithDepthColorMaps generateSelectedVertices(vector<unsigned short> &depth_map,
	vector<VerticesWithDepthColorMaps> &vertices_with_maps, vector<vector<unsigned char>> &current_map_indexes, vector<int> &depth_to_vertices_map, int w, int h)
{
	VerticesWithDepthColorMaps new_vertices_with_maps;

	new_vertices_with_maps.colors.resize(w * h * 3);
	new_vertices_with_maps.vertices.resize(w * h);
	new_vertices_with_maps.depth_to_vertices_map.resize(w * h);
	int n_new_vertices = 0;

	for (int y = 0; y<h; y++)
		for (int x = 0; x < w; x++)
		{
			if (depth_map[y * w + x] == 0)
				continue; 

			int map_index = current_map_indexes[y][x];
			int v_index = depth_to_vertices_map[x + y*w];
			new_vertices_with_maps.vertices[n_new_vertices] = vertices_with_maps[map_index].vertices[v_index];
			new_vertices_with_maps.colors[n_new_vertices * 3] = vertices_with_maps[map_index].colors[v_index * 3];
			new_vertices_with_maps.colors[n_new_vertices * 3 + 1] = vertices_with_maps[map_index].colors[v_index * 3 + 1];
			new_vertices_with_maps.colors[n_new_vertices * 3 + 2] = vertices_with_maps[map_index].colors[v_index * 3 + 2];
			new_vertices_with_maps.depth_to_vertices_map[y *w + x] = n_new_vertices;
			n_new_vertices++;
		}
	
	new_vertices_with_maps.colors.resize(n_new_vertices * 3);
	new_vertices_with_maps.vertices.resize(n_new_vertices);
	new_vertices_with_maps.depth_map = depth_map;
	

	return new_vertices_with_maps;
}


void generateTrianglesForVertices(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *widths, int *heights, vector<WorldTranformation> &world_transforms,
	 vector<IntrinsicCameraParameters> &intrinsic_params)
{
	int n_maps = (int)vertices_with_maps.size(); 
	vector<VerticesWithDepthColorMaps> new_vertices_with_maps(vertices_with_maps.size());
	
	//FILE *f = fopen("time.txt", "at");
	//auto start = std::chrono::system_clock::now();

	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	{
		int w = widths[current_map_index];
		int h = heights[current_map_index];
		WorldTranformation world_transform = world_transforms[current_map_index];
		world_transform.inv();
		vector<int> depth_to_vertices_map(w * h);
		vertices_with_maps[current_map_index].depth_map.resize(w*h);


		vector<vector<unsigned char>> current_map_indexes(h, vector<unsigned char>(w, current_map_index));
		new_vertices_with_maps[current_map_index].depth_map = vector<unsigned short>(h * w, 0);
		projectUnassignedVerticesIntoDepthMap(vertices_with_maps[current_map_index], depth_to_vertices_map, world_transform, intrinsic_params[current_map_index],
			w, h);

		char tmp[1024];
		//sprintf(tmp, "test_depth_%d_1.pgm", current_map_index);
		//writeDepthImage(vertices_with_maps[current_map_index].depth_map, w, h, tmp);

		for (int i = 0; i < n_maps; i++)
		{
			if (i == current_map_index)
				continue;

			assignDepthMapOverlay(vertices_with_maps[current_map_index].depth_map, vertices_with_maps[i], depth_to_vertices_map, world_transform, intrinsic_params[current_map_index], current_map_indexes, i, w, h);
		}

		//sprintf(tmp, "test_depth_%d_2.pgm", current_map_index);
		//writeDepthImage(vertices_with_maps[current_map_index].depth_map, w, h, tmp);

		new_vertices_with_maps[current_map_index] = generateSelectedVertices(vertices_with_maps[current_map_index].depth_map, vertices_with_maps, current_map_indexes, depth_to_vertices_map, w, h);


	}
	vertices_with_maps = new_vertices_with_maps;

	//auto end = std::chrono::system_clock::now();

	//fprintf(f, "%d\n", (int)(std::chrono::duration_cast<std::chrono::milliseconds>(end - start)).count());
	//fclose(f);
}


DEPTH_PROCESSING_API void __stdcall generateMeshFromDepthMaps(int n_maps, unsigned char* depth_maps,
	unsigned char *depth_colors, int *widths, int *heights, float *intr_params, float *wtransform_params, Mesh *out_mesh)
{
	

	int depth_pos = 0, colors_pos = 0;;
	vector<VerticesWithDepthColorMaps> vertices_with_maps(n_maps);

	vector<IntrinsicCameraParameters> intrinsic_params(n_maps);
	vector<WorldTranformation> world_transforms(n_maps);

	for (int i = 0; i < n_maps; i++)
	{
		intrinsic_params[i] = IntrinsicCameraParameters(intr_params + i * 7);
		world_transforms[i] = WorldTranformation(wtransform_params + i*(9 + 3));
	}

	generateVerticesFromDepthMaps(depth_maps, depth_colors, widths, heights, world_transforms, intrinsic_params, vertices_with_maps);
	generateTrianglesForVertices(vertices_with_maps, widths, heights, world_transforms, intrinsic_params);

	int n_total_vertices = 0;
	for (int i = 0; i < n_maps; i++)
		n_total_vertices += (int)vertices_with_maps[i].vertices.size();

	out_mesh->nVertices = n_total_vertices;
	out_mesh->vertices = new float[n_total_vertices * 3];
	out_mesh->verticesRGB = new unsigned char[n_total_vertices * 3];

	size_t vertices_so_far = 0;
	for (int i = 0; i < n_maps; i++)
	{
		memcpy(out_mesh->vertices + vertices_so_far * 3, (void*)vertices_with_maps[i].vertices.data(), vertices_with_maps[i].vertices.size() * 3 * sizeof(float));
		memcpy(out_mesh->verticesRGB + vertices_so_far * 3, vertices_with_maps[i].colors.data(), vertices_with_maps[i].colors.size());
		vertices_so_far += vertices_with_maps[i].vertices.size();
	}


	depth_pos = 0;
	int n_triangles = 0;
	vector<vector<TriangleIndexes>> triangle_indexes(n_maps);
	for (int i = 0; i < n_maps; i++)
	{
		int n_pixels = widths[i] * heights[i];
		triangle_indexes[i] = MeshGenerator::generateTrianglesGradients((unsigned short*)vertices_with_maps[i].depth_map.data(), vertices_with_maps[i].depth_to_vertices_map, widths[i], heights[i]);
		depth_pos += n_pixels * 2;
		n_triangles += (int)triangle_indexes[i].size(); 
	}

	int act_triangle = 0;
	out_mesh->triangles = new int[n_triangles * 3];
	int act_vertices = 0;
	for (int i = 0; i < n_maps; i++)
	{
		for (int j = 0; j < triangle_indexes[i].size(); j++)
		{
			triangle_indexes[i][j].ind[0] += act_vertices;
			triangle_indexes[i][j].ind[1] += act_vertices;
			triangle_indexes[i][j].ind[2] += act_vertices;
			memcpy(out_mesh->triangles + act_triangle * 3, triangle_indexes[i][j].ind, 3 * sizeof(int));
			act_triangle++;
		}

		act_vertices += (int)vertices_with_maps[i].vertices.size();
	}
	out_mesh->nTriangles = act_triangle;
}


extern "C" DEPTH_PROCESSING_API Mesh* __stdcall createMesh()
{
	Mesh *newMesh = new Mesh();
	newMesh->nTriangles = 0;
	newMesh->nVertices = 0;
	newMesh->triangles = NULL;
	newMesh->vertices = NULL;
	newMesh->verticesRGB = NULL;
	return newMesh; 
}

extern "C" DEPTH_PROCESSING_API void __stdcall deleteMesh(Mesh* mesh)
{
	if (mesh->triangles!=0)
		delete []mesh->triangles;
	
	if (mesh->vertices != 0)
		delete []mesh->vertices;

	if (mesh->verticesRGB != 0)
		delete []mesh->verticesRGB;
}
