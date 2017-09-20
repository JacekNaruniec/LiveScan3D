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

// defines for testing only
//#define STORE_FRAMES_INFORMATION
//#define LOAD_FRAMES_INFORMATION
//#define SHOW_TIMINGS


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

void createVertices(unsigned short *depth_map, unsigned char *depth_colors, int w, int h, IntrinsicCameraParameters params, WorldTranformation world_transform, VerticesWithDepthColorMaps &vertices_with_maps,
	float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
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
	vertices_with_maps.depth_map.resize(w*h);


	for (int y = 0; y < h; y++)
	{
		row_ptr = depth_map + y*w;
		for (int x = 0; x < w; x++)
		{
			if (row_ptr[x] != 0)
			{
				pos = x + y * w;

				Point3f p;
				float val = row_ptr[x];
				p.Z = val / 1000.0f;
				p.X = (x - params.cx) / params.fx;
				p.Y = (params.cy - y) / params.fy;

				p.X = p.X * p.Z;
				p.Y = p.Y * p.Z;

				p.X += world_transform.t[0];
				p.Y += world_transform.t[1];
				p.Z += world_transform.t[2];
				RotatePoint(p, world_transform.R);

				if (p.X<minX || p.X > maxX || p.Y < minY || p.Y > maxY || p.Z <minZ || p.Z >maxZ)
					continue;


				vertices_with_maps.depth_to_vertices_map[pos] = n_vertices;
				vertices_with_maps.vertices_to_depth_map[n_vertices] = pos;

				vertices[n_vertices] = p;

				colors[n_vertices * 3] = depth_colors[pos * 3];
				colors[n_vertices * 3 + 1] = depth_colors[pos * 3 + 1];
				colors[n_vertices * 3 + 2] = depth_colors[pos * 3 + 2];

				n_vertices++;
			}
		}
	}

	vertices.resize(n_vertices);
	colors.resize(n_vertices * 3);

	memcpy(vertices_with_maps.depth_map.data(), depth_map, w*h * sizeof(unsigned short));

	vertices_with_maps.vertices_to_depth_map.resize(n_vertices);
	vertices_with_maps.point_assigned = vector<bool>(n_vertices, false);
}



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

	//writeDepthImage(map_copy, w, h, "before.pgm");

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

	//writeDepthImage(map_copy, w, h, "after.pgm");
	memcpy(depth_map, map_copy.data(), w*h * sizeof(depth_map[0]));
	memcpy(colors, colors_copy.data(), w*h * 3 * sizeof(colors[0]));
}

void generateMapConfidence(VerticesWithDepthColorMaps &vertices_with_maps, int w, int h, int et_limit,
	int depth_threshold)
{
	auto &current_vertices_with_maps = vertices_with_maps;

	vector<unsigned short> &depth_map = vertices_with_maps.depth_map;
	current_vertices_with_maps.confidence_map = vector<unsigned short>(w*h, et_limit);
	vector<unsigned short> &conf_map = current_vertices_with_maps.confidence_map;

	int *pos_x = new int[w*h];
	int *pos_y = new int[w*h];
	int *new_pos_x = new int[w*h];
	int *new_pos_y = new int[w*h];

	int shift_x[] = { -1, 0 ,  1, -1, 1, -1, 0, 1 };
	int shift_y[] = { -1, -1, -1,  0, 0,  1, 1, 1 };

	int pos_size = 0;
	int new_pos_size = 0;

	vector<bool> marked(w*h, false);

	for (int y = 1; y < h - 1; y++)
		for (int x = 1; x < w - 1; x++)
		{
			int pos = y*w + x;
			if (depth_map[pos] == 0)
			{
				conf_map[pos] = 0;
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
				conf_map[pos] = 1;
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
				if (abs(depth - depth_map[new_pos]) < depth_threshold && conf_map[new_pos] == et_limit
					&& depth_map[new_pos] != 0)
				{
					conf_map[new_x + new_y * w] = max_et + 1;
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


void generateVerticesConfidence(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *widths, int *heights)
{
	size_t n_maps = vertices_with_maps.size();

	const int et_limit = 20;
	int depth_threshold = 20;

	vector<thread> threads;

	for (int i = 0; i < n_maps; i++)
	{
		//generateMapConfidence(vertices_with_maps[i], widths[i], heights[i], et_limit, depth_threshold);
		threads.push_back(thread(generateMapConfidence, std::ref(vertices_with_maps[i]), widths[i], heights[i], et_limit, depth_threshold));
	}

	for (int i = 0; i < threads.size(); i++)
		threads[i].join();
	/*
	for (int i = 0; i < n_maps; i++)
	{
	char tmp[1024];
	sprintf(tmp, "test/confidence_%d.pgm", i);
	writeDepthImage(vertices_with_maps[i].confidence_map, widths[i], heights[i], tmp);
	sprintf(tmp, "test/confidence_%d_d.pgm", i);
	writeDepthImage(vertices_with_maps[i].depth_map, widths[i], heights[i], tmp);
	}*/

}

/*
void generateVerticesConfidence(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *widths, int *heights)
{
size_t n_maps = vertices_with_maps.size();
int shift_x[] = { -1, 0 ,  1, -1, 1, -1, 0, 1 };
int shift_y[] = { -1, -1, -1,  0, 0,  1, 1, 1 };

int max_w = 0, max_h = 0;
for (int i = 0; i < n_maps; i++)
{
max_w = max(max_w, widths[i]);
max_h = max(max_h, heights[i]);
}

int depth_threshold = 20;
int *pos_x = new int[max_w*max_h];
int *pos_y = new int[max_w*max_h];
int *new_pos_x = new int[max_w*max_h];
int *new_pos_y = new int[max_w*max_h];

for (int i = 0; i < n_maps; i++)
{
auto &current_vertices_with_maps = vertices_with_maps[i];
int w = widths[i];
int h = heights[i];

vector<unsigned short> &depth_map = vertices_with_maps[i].depth_map;
current_vertices_with_maps.confidence_map = vector<unsigned short>(w*h, 0);
vector<unsigned short> &conf_map = current_vertices_with_maps.confidence_map;
//vector<int> pos_x, pos_y, new_pos_x, new_pos_y;


int pos_size = 0;
int new_pos_size = 0;

//pos_x.reserve((w + h) * 2);
//pos_y.reserve((w + h) * 2);

vector<bool> marked(w*h, false);

for (int y = 1; y < h - 1; y++)
for (int x = 1; x < w - 1; x++)
{
int pos = y*w + x;
if (depth_map[pos] == 0)
continue;

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
conf_map[pos] = 1;
marked[pos] = true;
}
}



for (int y = 1; y < h - 1; y++)
for (int x = 1; x < w - 1; x++)
{

int pos = y*w + x;
if (depth_map[pos] > 0 && conf_map[pos] == 0)
{
pos_x[pos_size] = x;
pos_y[pos_size] = y;

pos_size++;

int min_et = 9999999;
for (int shift = 0; shift < 8; shift++)
{
unsigned short val = conf_map[pos + w *shift_y[shift] + shift_x[shift]];
if (val != 0 && val < min_et)
min_et = val;
}

if (min_et == 9999999)
continue;

min_et = min_et + 1;

marked[x + y*w] = true;

while (pos_size!=0)
{
for (size_t el = 0; el < pos_size; el++)
{
bool requirement_met = false;
int el_pos = pos_x[el] + pos_y[el] * w;
for (int shift = 0; shift < 8; shift++)
{
int new_x = pos_x[el] + shift_x[shift];
int new_y = pos_y[el] + shift_y[shift];

if (new_x <= 0 || new_y <= 0 || new_x >= w || new_y >= h)
continue;
int new_pos = new_x + new_y * w;

// find point with confidence min_et-1
if (abs(depth_map[el_pos] - depth_map[new_pos]) < depth_threshold && conf_map[new_pos] == min_et - 1)
{
requirement_met = true;
break;
}

}

if (!requirement_met) continue;

conf_map[pos_x[el] + pos_y[el] * w] = min_et;
for (int shift = 0; shift < 8; shift++)
{
int x1 = pos_x[el] + shift_x[shift];
int y1 = pos_y[el] + shift_y[shift];
if (x1 <= 0 || y1 <= 0 || x1 >= w || y1 >= h) continue;

if (!marked[x1 + y1 * w] && depth_map[x1 + y1 * w] != 0)
{
new_pos_x[new_pos_size] = x1;
new_pos_y[new_pos_size] = y1;
new_pos_size++;
marked[x1 + y1 * w] = true;
}
}

}
memcpy(pos_x, new_pos_x, new_pos_size * sizeof(new_pos_x[0]));
memcpy(pos_y, new_pos_y, new_pos_size * sizeof(new_pos_y[0]));
pos_size = new_pos_size;
new_pos_size = 0;
}
}
}




// for testing only

//char tmp[1024];
//sprintf(tmp, "test/confidence_%d.pgm", i);
//writeDepthImage(current_vertices_with_maps.confidence_map, w, h, tmp);
//sprintf(tmp, "test/confidence_%d_d.pgm", i);
//writeDepthImage(current_vertices_with_maps.depth_map, w, h, tmp);
// ---------------------------
}

delete[]pos_x;
delete[]pos_y;
delete[]new_pos_x;
delete[]new_pos_y;
}
*/

void generateVerticesFromDepthMaps(unsigned char* depth_maps, unsigned char *depth_colors, int *widths, int *heights,
	vector<WorldTranformation> &world_transforms, vector<IntrinsicCameraParameters> &intrinsic_params, vector<VerticesWithDepthColorMaps> &vertices_with_maps,
	float minX, float minY, float minZ, float maxX, float maxY, float maxZ, int map_index = -1)
{
	vector<thread> threads;
	int n_maps = (int)vertices_with_maps.size();

	int depth_pos = 0, colors_pos = 0;
	for (int i = 0; i < n_maps; i++)
	{
		int n_pixels = widths[i] * heights[i];

		if (map_index == -1 || map_index == i)
		{
			threads.push_back(thread(createVertices, (unsigned short*)(depth_maps + depth_pos), depth_colors + colors_pos, widths[i], heights[i], intrinsic_params[i],
				world_transforms[i], std::ref(vertices_with_maps[i]), minX, minY, minZ, maxX, maxY, maxZ));
		}



		depth_pos += n_pixels * 2;
		colors_pos += n_pixels * 3;
	}


	for (int i = 0; i < threads.size(); i++)
		threads[i].join();
}

void pointProjection(Point3f &p, int &out_x, int &out_y, unsigned short &out_d, WorldTranformation &wt, IntrinsicCameraParameters &ip)
{
	Point3f tmp = p;
	RotatePoint(tmp, wt.R);
	tmp.X += wt.t[0];
	tmp.Y += wt.t[1];
	tmp.Z += wt.t[2];

	out_x = static_cast<int>((tmp.X * ip.fx) / tmp.Z + ip.cx + 0.5);
	//out_y = static_cast<int>((tmp.Y * ip.fy) / tmp.Z + ip.cy);
	out_y = static_cast<int>(ip.cy - (tmp.Y * ip.fy) / tmp.Z + 0.5);
	out_d = static_cast<unsigned short>(min(max(0, (int)(tmp.Z * 1000.0f)), 65535));
}

void projectVerticesIntoDepthMap(VerticesWithDepthColorMaps &vertices_with_maps, WorldTranformation &wt, IntrinsicCameraParameters &ip, int w, int h, bool includeAssigned)
{
	vector<Point3f> &vertices = vertices_with_maps.vertices;
	int n_vertices = (int)vertices.size();

	vector<unsigned short> &out_depth_map = vertices_with_maps.depth_map;

	for (int v = 0; v < n_vertices; v++)
	{
		if (!includeAssigned && vertices_with_maps.point_assigned[v])
			continue;

		int x, y;
		unsigned short d;
		pointProjection(vertices[v], x, y, d, wt, ip);
		if (x < 0 || x >= w || y < 0 || y >= h)
			continue;

		out_depth_map[y * w + x] = d;
		vertices_with_maps.depth_to_vertices_map[y * w + x] = v;
	}
}

void writeDepthImage(vector<unsigned short> &depth_image, int w, int h, string filename)
{
	vector<unsigned char> image(h*w, 0);

	for (int i = 0; i < w; i++)
		for (int j = 0; j < h; j++)
			image[i + j*w] = (unsigned char)depth_image[j * w + i];

	writePGM(filename.c_str(), w, h, image.data());
}


void pointsProjectionBounded(Point3f *vertices, int n_vertices, vector<PointProjection> &projections,
	int minX, int maxX, int minY, int maxY, int indexes_shift, WorldTranformation wt, IntrinsicCameraParameters ip,
	int &n_valid_projections)
{
	projections.resize(n_vertices);
	int pos = 0;

	for (int v = 0; v < n_vertices; v++)
	{
		int x, y;
		unsigned short d;
		pointProjection(vertices[v], x, y, d, wt, ip);
		
		if (x < minX || x >= maxX || y < minY || y >= maxY || d == 0)
			continue;

		projections[pos].x = x; 
		projections[pos].y = y;
		projections[pos].index = v + indexes_shift;
		projections[pos].d = d;
		pos++;
	}

	n_valid_projections = pos; 
}

void assignDepthMapOverlay(vector<VerticesWithDepthColorMaps> &vertices_with_maps,
	vector<int> &depth_to_vertices_map, vector<vector<unsigned char>> &map_indexes, WorldTranformation &wt, IntrinsicCameraParameters &ip, int overlayed_index, int base_map_index, int w, int h)
{
	const int depth_threshold = 20;
	const int confidence_threshold = 5;
	vector<unsigned short> &depth_map = vertices_with_maps[base_map_index].depth_map;
	vector<Point3f> &overlay_vertices = vertices_with_maps[overlayed_index].vertices;

	int n_vertices = (int)overlay_vertices.size();
	

	vector<PointProjection> projections; 

	for (int v = 0; v < n_vertices; v++)
	{
		if (vertices_with_maps[overlayed_index].point_assigned[v])
			continue;

		int x, y;
		unsigned short d, cur_d;
		int overlay_verticle_to_pos = vertices_with_maps[overlayed_index].vertices_to_depth_map[v];
		pointProjection(overlay_vertices[v], x, y, d, wt, ip);

		if (x < 2 || x >= w || y < 2 || y >= h || d == 0)
			continue;

		int cur_map_index = map_indexes[y][x];
		int cur_depth_to_verticle = vertices_with_maps[cur_map_index].depth_to_vertices_map[y*w + x];
		int pos = y*w + x;
		cur_d = depth_map[pos];

		if (cur_d == 0 || abs(d - cur_d) < depth_threshold)
		{
			int assignedMapIndex = base_map_index, assignedDepthToVerticeIndex;
			unsigned short assignedDepthValue;

			if (cur_d == 0)  // no choice, pick the only existing
			{
				assignedMapIndex = overlayed_index;
				assignedDepthToVerticeIndex = v;
				assignedDepthValue = d;
			}
			else if (vertices_with_maps[base_map_index].confidence_map[pos] > 5)
			{
				assignedDepthToVerticeIndex = v;
				assignedDepthValue = cur_d;
			}
			else {
				// check which combination can deliver more trianglulation possibilities - depth value "d" or "cur_d"?
				depth_map[pos] = d;
				int n1 = MeshGenerator::getNTrianglesPassingConditions(depth_map.data() + pos, w);
				depth_map[pos] = cur_d;
				int n2 = MeshGenerator::getNTrianglesPassingConditions(depth_map.data() + pos, w);

				if (n1 > n2)
				{
					assignedMapIndex = overlayed_index;
					assignedDepthValue = d;
					assignedDepthToVerticeIndex = v;
				}
				else
					if (n1 == n2 && vertices_with_maps[base_map_index].confidence_map[pos] < 5 &&
						vertices_with_maps[base_map_index].confidence_map[pos] < vertices_with_maps[overlayed_index].confidence_map[overlay_verticle_to_pos])
					{
						assignedMapIndex = overlayed_index;
						assignedDepthValue = d;
						assignedDepthToVerticeIndex = v;
					}
			}

			if (assignedMapIndex == overlayed_index)
			{
				depth_to_vertices_map[pos] = assignedDepthToVerticeIndex;
				map_indexes[y][x] = assignedMapIndex;
				depth_map[pos] = assignedDepthValue;
			}

			// additional 5 pix border is used to prevent lack of continuity in the borders
			// of different depth maps when connected 
			if (vertices_with_maps[base_map_index].confidence_map[x + y*w] > 1)
				if (x>5 && y>5 && x<(w - 5) && y<(h - 5))
					vertices_with_maps[overlayed_index].point_assigned[v] = true;
		}
	}

	// ------------ delete me ----------
	//writeDepthImage(overlayed_depth, w, h, "overlay.pgm");
	// ---------------------------------
}

VerticesWithDepthColorMaps generateSelectedVertices(vector<unsigned short> &depth_map,
	vector<VerticesWithDepthColorMaps> &vertices_with_maps, vector<vector<unsigned char>> &current_map_indexes, vector<int> &depth_to_vertices_map, int w, int h)
{
	VerticesWithDepthColorMaps new_vertices_with_maps;

	new_vertices_with_maps.colors.resize(w * h * 3);
	new_vertices_with_maps.vertices.resize(w * h);
	new_vertices_with_maps.depth_to_vertices_map = vector<int>(w * h, -1);
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


void mergeVerticesForViews(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *widths, int *heights, vector<WorldTranformation> &world_transforms,
	vector<IntrinsicCameraParameters> &intrinsic_params)
{
	int n_maps = (int)vertices_with_maps.size();
	vector<VerticesWithDepthColorMaps> new_vertices_with_maps(vertices_with_maps.size());

	SimpleTimer tim; 

	/*
	char tmp[1024];
	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	{
	sprintf(tmp, "test/depth_%d_1.pgm", current_map_index);
	writeDepthImage(vertices_with_maps[current_map_index].depth_map, widths[current_map_index], heights[current_map_index], tmp);
	}
	*/

	//FILE *f = fopen("time.txt", "at");
	//auto start = std::chrono::system_clock::now();

	tim.start();
	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	{
		int w = widths[current_map_index];
		int h = heights[current_map_index];
		WorldTranformation world_transform = world_transforms[current_map_index];
		world_transform.inv();
		vector<int> depth_to_vertices_map(w * h);

		vertices_with_maps[current_map_index].depth_map = vector<unsigned short>(w*h, 0);

		new_vertices_with_maps[current_map_index].depth_map = vector<unsigned short>(h * w, 0);

		projectVerticesIntoDepthMap(vertices_with_maps[current_map_index], world_transform, intrinsic_params[current_map_index], w, h, false);

		tim.printLapTimeAndRestart("mergeVerticesForViews 1 ");

		for (auto &v : vertices_with_maps[current_map_index].point_assigned) v = true;
		/*
		sprintf(tmp, "test/test_depth_%d_1.pgm", current_map_index);
		writeDepthImage(vertices_with_maps[current_map_index].depth_map, w, h, tmp);
		*/
		depth_to_vertices_map = vertices_with_maps[current_map_index].depth_to_vertices_map;
		vector<vector<unsigned char>> map_indexes(h, vector<unsigned char>(w, current_map_index));

		for (int i = 0; i < n_maps; i++)
		{
			if (i == current_map_index)
				continue;

			assignDepthMapOverlay(vertices_with_maps, depth_to_vertices_map, map_indexes, world_transform,
				intrinsic_params[current_map_index], i, current_map_index, w, h);
		}
		tim.printLapTimeAndRestart("mergeVerticesForViews 2 ");
		/*
		sprintf(tmp, "test/test_depth_%d_2.pgm", current_map_index);
		writeDepthImage(vertices_with_maps[current_map_index].depth_map, w, h, tmp);
		*/
		new_vertices_with_maps[current_map_index] = generateSelectedVertices(vertices_with_maps[current_map_index].depth_map, vertices_with_maps, map_indexes, depth_to_vertices_map, w, h);
		tim.printLapTimeAndRestart("mergeVerticesForViews 3 ");

	}
	vertices_with_maps = new_vertices_with_maps;

	//auto end = std::chrono::system_clock::now();

	//fprintf(f, "%d\n", (int)(std::chrono::duration_cast<std::chrono::milliseconds>(end - start)).count());
	//fclose(f);
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

void loadAllFramesInformation(string filename, int &n_maps, unsigned char** depth_maps,
	unsigned char **depth_colors, int **widths, int **heights, float **intr_params, float **wtransform_params)
{
	FILE *f = fopen(filename.c_str(), "rb");

	fread(&n_maps, sizeof(n_maps), 1, f);
	*widths = new int[n_maps];
	*heights = new int[n_maps];

	if (n_maps > 0)
	{
		fread(*widths, sizeof((*widths)[0]), n_maps, f);
		fread(*heights, sizeof((*heights)[0]), n_maps, f);
	}

	int pos_d = 0, pos_c = 0;
	for (int i = 0; i < n_maps; i++)
	{
		pos_d += (*widths)[i] * (*heights)[i] * 2;
		pos_c += (*widths)[i] * (*heights)[i] * 3;
	}

	*depth_maps = new unsigned char[pos_d];
	*depth_colors = new unsigned char[pos_c];
	pos_c = 0;
	pos_d = 0;

	for (int i = 0; i < n_maps; i++)
	{
		fread(*depth_maps + pos_d, 1, (*widths)[i] * (*heights)[i] * 2, f);
		fread(*depth_colors + pos_c, 1, (*widths)[i] * (*heights)[i] * 3, f);
		pos_d += (*widths)[i] * (*heights)[i] * 2;
		pos_c += (*widths)[i] * (*heights)[i] * 3;
	}

	*intr_params = new float[7 * n_maps];
	*wtransform_params = new float[12 * n_maps];

	fread(*intr_params, sizeof((*intr_params)[0]), 7 * n_maps, f);
	fread(*wtransform_params, sizeof((*wtransform_params)[0]), 12 * n_maps, f);

	fclose(f);
}

int calculateMapsCoverage(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *widths, int *heights, vector<WorldTranformation> &world_transforms,
	vector<IntrinsicCameraParameters> &intrinsic_params, int index1, int index2)
{
	vector<Point3f> &vertices = vertices_with_maps[index2].vertices;
	int n_vertices = (int)vertices.size();

	vector<unsigned short> &depth_map = vertices_with_maps[index1].depth_map;
	WorldTranformation wt = world_transforms[index1];
	wt.inv();
	IntrinsicCameraParameters ip = intrinsic_params[index1];
	int w = widths[index1];
	int h = heights[index1];
	int n_common_pixels = 0;
	const int depth_threshold = 20;

	for (int v = 0; v < n_vertices; v++)
	{
		int x, y;
		unsigned short d1;
		int pos_vertex_to_depth = vertices_with_maps[index2].vertices_to_depth_map[v];
		pointProjection(vertices[v], x, y, d1, wt, ip);

		if (x < 0 || x >= w || y < 0 || y >= h || d1 == 0)
			continue;

		// don't take vertices with low confidence (near the object edge), as the color can be distorted
		if (vertices_with_maps[index1].confidence_map[x + y * w] < 5 ||
			vertices_with_maps[index2].confidence_map[pos_vertex_to_depth] < 5)
			continue;

		unsigned short d2 = depth_map[x + y*w];

		if (d2 > 0 && abs(d1 - d2) < depth_threshold)
			n_common_pixels++;
	}

	return n_common_pixels;
}

ColorCorrectionParams getColorCorrectionTransform(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *widths, int *heights, vector<WorldTranformation> &world_transforms,
	vector<IntrinsicCameraParameters> &intrinsic_params, int index1, int index2)
{
	vector<Point3f> &vertices = vertices_with_maps[index2].vertices;
	int n_vertices = (int)vertices.size();

	vector<unsigned char> &colors1 = vertices_with_maps[index1].colors;
	vector<unsigned char> &colors2 = vertices_with_maps[index2].colors;
	ColorCorrectionParams transform;

	vector<unsigned short> &depth_map = vertices_with_maps[index1].depth_map;
	WorldTranformation wt = world_transforms[index1];
	wt.inv();
	IntrinsicCameraParameters ip = intrinsic_params[index1];
	int w = widths[index1];
	int h = heights[index1];
	int n_common_pixels = 0;
	const int depth_threshold = 20;

	vector<unsigned char> rgb_src(n_vertices * 3);
	vector<unsigned char> rgb_dst(n_vertices * 3);
	int n_elements = 0;

	for (int v = 0; v < n_vertices; v++)
	{
		int x, y;
		unsigned short d1;
		pointProjection(vertices[v], x, y, d1, wt, ip);
		if (x < 0 || x >= w || y < 0 || y >= h)
			continue;

		int pos_depth_2 = vertices_with_maps[index2].vertices_to_depth_map[v];
		int pos_vertex_1 = vertices_with_maps[index1].depth_to_vertices_map[x + y*w];

		// don't take vertices with low confidence (near the object edge), as the color can be distorted
		if (vertices_with_maps[index1].confidence_map[x + y * w] < 5 ||
			vertices_with_maps[index2].confidence_map[pos_depth_2] < 5)
			continue;

		unsigned short d2 = depth_map[x + y*w];

		if (d2 > 0 && abs(d1 - d2) < depth_threshold)
		{
			rgb_src[n_elements * 3] = colors1[pos_vertex_1 * 3];
			rgb_src[n_elements * 3 + 1] = colors1[pos_vertex_1 * 3 + 1];
			rgb_src[n_elements * 3 + 2] = colors1[pos_vertex_1 * 3 + 2];

			rgb_dst[n_elements * 3] = colors2[v * 3];
			rgb_dst[n_elements * 3 + 1] = colors2[v * 3 + 1];
			rgb_dst[n_elements * 3 + 2] = colors2[v * 3 + 2];

			n_elements++;
		}
	}

	rgb_src.resize(n_elements * 3);
	rgb_dst.resize(n_elements * 3);

	transform = getColorCorrectionTransform(rgb_src, rgb_dst);
	transform.base_map_index = index1;
	transform.map_index = index2;

	return transform;
}

void updateColorCorrectionCoefficients(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *widths, int *heights, vector<WorldTranformation> &world_transforms,
	vector<IntrinsicCameraParameters> &intrinsic_params, vector<ColorCorrectionParams> &coeffs)
{
	coeffs.clear();
	size_t n_maps = vertices_with_maps.size();
	vector<vector<int>> coverage(n_maps, vector<int>(n_maps, 0));
	vector<bool> colors_assigned(n_maps, false);
	int coverage_threshold = 100;

	for (int map1_index = 0; map1_index < n_maps; map1_index++)
	{
		for (int map2_index = map1_index + 1; map2_index < n_maps; map2_index++)
		{
			coverage[map1_index][map2_index] = calculateMapsCoverage(vertices_with_maps, widths, heights, world_transforms,
				intrinsic_params, map1_index, map2_index);
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

			transform = getColorCorrectionTransform(vertices_with_maps, widths, heights, world_transforms,
				intrinsic_params, max_val_index_1, max_val_index_2);

			colors_assigned[max_val_index_1] = true;
			colors_assigned[max_val_index_2] = true;

			coeffs.push_back(transform);
			no_more_to_assign = false;
		}
	}


}

void applyColorCorrection(vector<VerticesWithDepthColorMaps> &vertices_with_maps, vector<ColorCorrectionParams> &color_correction_coeffs)
{
	size_t n_tranforms = color_correction_coeffs.size();


	for (size_t i = 0; i < n_tranforms; i++)
	{
		int target_map_index = color_correction_coeffs[i].map_index;
		vector<unsigned char> &colors = vertices_with_maps[target_map_index].colors;

		applyColorCorrection(colors, color_correction_coeffs[i]);
	}
}


void formMesh(Mesh *out_mesh, vector<VerticesWithDepthColorMaps> &vertices_with_maps, vector<vector<TriangleIndexes>> &triangle_indexes)
{
	int n_maps = (int)vertices_with_maps.size();
	size_t n_total_vertices = 0;
	size_t n_total_triangles = 0;

	for (int i = 0; i<n_maps; i++)
	{
		n_total_triangles += triangle_indexes[i].size();
		n_total_vertices += vertices_with_maps[i].vertices.size();
	}

	out_mesh->nVertices = (int)n_total_vertices;
	out_mesh->vertices = new VertexC4ubV3f[n_total_vertices];

	size_t vertices_so_far = 0;
	for (int i = 0; i < n_maps; i++)
	{
		for (int j = 0; j < vertices_with_maps[i].vertices.size(); j++)
		{
			out_mesh->vertices[j + vertices_so_far].R = vertices_with_maps[i].colors[j * 3];
			out_mesh->vertices[j + vertices_so_far].G = vertices_with_maps[i].colors[j * 3 + 1];
			out_mesh->vertices[j + vertices_so_far].B = vertices_with_maps[i].colors[j * 3 + 2];
			out_mesh->vertices[j + vertices_so_far].A = 255;
			out_mesh->vertices[j + vertices_so_far].X = vertices_with_maps[i].vertices[j].X;
			out_mesh->vertices[j + vertices_so_far].Y = vertices_with_maps[i].vertices[j].Y;
			out_mesh->vertices[j + vertices_so_far].Z = vertices_with_maps[i].vertices[j].Z;

		}
		vertices_so_far += vertices_with_maps[i].vertices.size();
	}


	int act_triangle = 0;
	out_mesh->triangles = new int[n_total_triangles * 3];
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

DEPTH_PROCESSING_API void __stdcall generateMeshFromDepthMaps(int n_maps, unsigned char* depth_maps,
	unsigned char *depth_colors, int *widths, int *heights, float *intr_params, float *wtransform_params, Mesh *out_mesh, bool bcolor_transfer,
	float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
{
	int depth_pos = 0, colors_pos = 0;;

#ifdef STORE_FRAMES_INFORMATION
	// for testing purposes only
	storeAllFramesInformation("frames_info.bin", n_maps, depth_maps, depth_colors, widths, heights, intr_params, wtransform_params);
#endif

#ifdef LOAD_FRAMES_INFORMATION
	// for testing purposes only 
	loadAllFramesInformation("frames_info.bin", n_maps, &depth_maps, &depth_colors, &widths, &heights, &intr_params, &wtransform_params);
#endif

	vector<VerticesWithDepthColorMaps> vertices_with_maps(n_maps);

	vector<IntrinsicCameraParameters> intrinsic_params(n_maps);
	vector<WorldTranformation> world_transforms(n_maps);
	vector<ColorCorrectionParams> color_correction_coeffs;

	for (int i = 0; i < n_maps; i++)
	{
		intrinsic_params[i] = IntrinsicCameraParameters(intr_params + i * 7);
		world_transforms[i] = WorldTranformation(wtransform_params + i*(9 + 3));
	}

#ifdef SHOW_TIMINGS
	SimpleTimer timer;

#endif

	generateVerticesFromDepthMaps(depth_maps, depth_colors, widths, heights, world_transforms, intrinsic_params, vertices_with_maps,
		minX, minY, minZ, maxX, maxY, maxZ);

#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("1");
#endif
	generateVerticesConfidence(vertices_with_maps, widths, heights);
#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("2");
#endif



	if (bcolor_transfer)
	{
		updateColorCorrectionCoefficients(vertices_with_maps, widths, heights, world_transforms, intrinsic_params, color_correction_coeffs);
		applyColorCorrection(vertices_with_maps, color_correction_coeffs);
	}
#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("3");
#endif

	mergeVerticesForViews(vertices_with_maps, widths, heights, world_transforms, intrinsic_params);
#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("4");
#endif


	int n_total_vertices = 0;
	for (int i = 0; i < n_maps; i++)
		n_total_vertices += (int)vertices_with_maps[i].vertices.size();

	depth_pos = 0;
	int n_triangles = 0;
	vector<vector<TriangleIndexes>> triangle_indexes(n_maps);
	vector<std::thread> generate_triangles_threads;
	for (int i = 0; i < n_maps; i++)
	{
		int n_pixels = widths[i] * heights[i];
		/*
		char tmp[1024];
		sprintf(tmp, "d:/temp/depth_for_triangles_%.4d.pgm", i);
		writeDepthImage(vertices_with_maps[i].depth_map, widths[i], heights[i], tmp);
		*/
		//MeshGenerator::generateTrianglesGradients((unsigned short*)vertices_with_maps[i].depth_map.data(),
		//std::ref(vertices_with_maps[i].depth_to_vertices_map), std::ref(triangle_indexes[i]), widths[i], heights[i]);
		generate_triangles_threads.push_back(thread(MeshGenerator::generateTrianglesGradients, (unsigned short*)vertices_with_maps[i].depth_map.data(),
			std::ref(vertices_with_maps[i].depth_to_vertices_map), std::ref(triangle_indexes[i]), widths[i], heights[i]));

		//triangle_indexes[i] = MeshGenerator::generateTrianglesGradients();
		depth_pos += n_pixels * 2;
	}

	for (int i = 0; i < n_maps; i++)
	{
		generate_triangles_threads[i].join();
		n_triangles += (int)triangle_indexes[i].size();
	}
#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("5");
#endif

	formMesh(out_mesh, vertices_with_maps, triangle_indexes);
#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("6");
#endif
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

extern "C" DEPTH_PROCESSING_API void __stdcall deleteMesh(Mesh* mesh)
{
	if (mesh->triangles != 0)
		delete[]mesh->triangles;

	if (mesh->vertices != 0)
		delete[]mesh->vertices;
}

