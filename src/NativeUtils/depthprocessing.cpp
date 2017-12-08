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

#include "simpleimage.h"

// defines for testing only
//#define STORE_FRAMES_INFORMATION
#define LOAD_FRAMES_INFORMATION
//#define DEBUG_IMAGES
#define SHOW_TIMINGS
#define PERFORM_ICP
//#define LOAD_REFINED_ICP
#define FILTER_FLYING_PIXELS
#define CONVEX_HULL

using namespace std;


// DELETE ME!!! - i'm in kinectCapture.cpp
void filterFlyingPixels(int neighbourhoodSize, float thr, int maxNonFittingNeighbours, int w, int h, unsigned short *depth_map)
{
	int nDepthPixels = w*h;
	int nNeighbours = (neighbourhoodSize * 2 + 1) * (neighbourhoodSize * 2 + 1) - 1;
	std::vector<int> shifts(nNeighbours);
	int *pshifts = shifts.data();
	int shift_n = 0;

	for (int x = -neighbourhoodSize; x <= neighbourhoodSize; x++)
		for (int y = -neighbourhoodSize; y <= neighbourhoodSize; y++)
		{
			if (x == 0 && y == 0)
				continue;

			shifts[shift_n] = x * w + y;
			shift_n++;
		}

	maxNonFittingNeighbours = nNeighbours / 2;

	std::vector<int> indexesToRemove;
	for (int y = neighbourhoodSize; y < h - neighbourhoodSize; y++)
	{
		int rowPos = y*w;
		UINT16 *rowPtr = depth_map + rowPos;
		for (int x = neighbourhoodSize; x < w - neighbourhoodSize; x++)
		{
			int val = rowPtr[x];
			int n_diff = 0;
			for (int shift = 0; shift < nNeighbours; shift++)
			{
				int diff = abs(rowPtr[x + shifts[shift]] - val);
				if (diff > thr)
					n_diff++;
			}
			if (n_diff > maxNonFittingNeighbours)
				indexesToRemove.push_back(rowPos + x);
		}
	}

	for (size_t i = 0; i < indexesToRemove.size(); i++)
		depth_map[indexesToRemove[i]] = 0;
}
// --------------------------------------

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

inline void getVertexCoorinates(int x, int y, unsigned short d, Point3f &p, IntrinsicCameraParameters &params, WorldTransformation &world_transform)
{
	p.Z = d / 1000.0f;
	p.X = (x - params.cx) / params.fx;
	p.Y = (params.cy - y) / params.fy;

	p.X = p.X * p.Z;
	p.Y = p.Y * p.Z;

	p.X += world_transform.t[0];
	p.Y += world_transform.t[1];
	p.Z += world_transform.t[2];
	RotatePoint(p, world_transform.R);
}

void createVertices(unsigned short *depth_map, unsigned char *depth_colors, int w, int h, IntrinsicCameraParameters params, WorldTransformation world_transform, VerticesWithDepthColorMaps &vertices_with_maps,
	float minX, float minY, float minZ, float maxX, float maxY, float maxZ)
{
	auto &vertices = vertices_with_maps.vertices;
	auto &colors = vertices_with_maps.colors;
	auto &colors_map = vertices_with_maps.colors_map;

	colors_map.resize(w*h * 3);
	colors.resize(w*h * 3);
	vertices.resize(w*h);
	int n_pixels = w*h;
	int n_vertices = 0;
	unsigned short *row_ptr;
	int pos;
	vertices_with_maps.depth_to_vertices_map = vector<int>(n_pixels, -1);
	vertices_with_maps.vertices_to_depth_map = vector<int>(n_pixels, -1);
	vertices_with_maps.depth_map = vector<unsigned short>(w*h);

	world_transform.inv();

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
	memcpy(vertices_with_maps.colors_map.data(), depth_colors, w*h * 3);

	vertices_with_maps.vertices_to_depth_map.resize(n_vertices);
	vertices_with_maps.point_assigned = vector<bool>(w*h, false);
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

#ifdef DEBUG_IMAGES
	writeDepthImage(map_copy, w, h, "before_radial_correction.pgm");
#endif
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
#ifdef DEBUG_IMAGES
	writeDepthImage(map_copy, w, h, "after_radial_correction.pgm");
#endif
	memcpy(depth_map, map_copy.data(), w*h * sizeof(depth_map[0]));
	memcpy(colors, colors_copy.data(), w*h * 3 * sizeof(colors[0]));
}

void generateMapConfidence(vector<unsigned short> &depth_map, vector<unsigned char> &confidence_map, int w, int h, int et_limit,
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


void generateVerticesConfidence(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *widths, int *heights)
{
	size_t n_maps = vertices_with_maps.size();

	const int et_limit = 20;
	int depth_threshold = 20;

	vector<thread> threads;

	for (int i = 0; i < n_maps; i++)
	{
		threads.push_back(thread(generateMapConfidence, std::ref(vertices_with_maps[i].depth_map), std::ref(vertices_with_maps[i].confidence_map), widths[i], heights[i], et_limit, depth_threshold));
		//generateMapConfidence(std::ref(vertices_with_maps[i].depth_map), std::ref(vertices_with_maps[i].confidence_map), widths[i], heights[i], et_limit, depth_threshold);
	}
	for (int i = 0; i < threads.size(); i++)
	{
		threads[i].join();
	}

	for (int i = 0; i < n_maps; i++)
	{
#ifdef DEBUG_IMAGES
		char tmp[1024];
		sprintf(tmp, "test/confidence_%d.pgm", i);
		vector<unsigned char> temp_cm = vertices_with_maps[i].confidence_map;
		for (int j = 0; j < widths[i] * heights[i]; j++)
			temp_cm[j] *= 10;
		writePGM(tmp, widths[i], heights[i], temp_cm.data());
#endif
	}

}

inline int iround(float x)
{
	return (int)(x + 0.5f);
}

// base code for this function from http://forum.devmaster.net/t/advanced-rasterization/6145
// barycetric coordinates implemented according to https://codeplea.com/triangular-interpolation
void drawTriangle(const int &v1_x, const int &v1_y, const int &v1_d,
	const int &v2_x, const int &v2_y, const int &v2_d,
	const int &v3_x, const int &v3_y, const int &v3_d,
	unsigned short *depth_map, int w, int h, unsigned short *tag2_map, unsigned short tag2)
{
	// 28.4 fixed-point coordinates
	const int Y1 = iround(16.0f * v1_y);
	const int Y2 = iround(16.0f * v2_y);
	const int Y3 = iround(16.0f * v3_y);

	const int X1 = iround(16.0f * v1_x);
	const int X2 = iround(16.0f * v2_x);
	const int X3 = iround(16.0f * v3_x);

	// Deltas
	const int DX12 = X1 - X2;
	const int DX23 = X2 - X3;
	const int DX31 = X3 - X1;

	const int DY12 = Y1 - Y2;
	const int DY23 = Y2 - Y3;
	const int DY31 = Y3 - Y1;

	// Fixed-point deltas
	const int FDX12 = DX12 << 4;
	const int FDX23 = DX23 << 4;
	const int FDX31 = DX31 << 4;

	const int FDY12 = DY12 << 4;
	const int FDY23 = DY23 << 4;
	const int FDY31 = DY31 << 4;

	// Bounding rectangle
	int minx = (min(min(X1, X2), X3) + 0xF) >> 4;
	int maxx = (max(max(X1, X2), X3) + 0xF) >> 4;
	int miny = (min(min(Y1, Y2), Y3) + 0xF) >> 4;
	int maxy = (max(max(Y1, Y2), Y3) + 0xF) >> 4;

	depth_map += miny * w;
	//tag2_map += miny * w;

	// Half-edge constants
	int C1 = DY12 * X1 - DX12 * Y1;
	int C2 = DY23 * X2 - DX23 * Y2;
	int C3 = DY31 * X3 - DX31 * Y3;

	// Correct for fill convention
	if (DY12 < 0 || (DY12 == 0 && DX12 > 0)) C1++;
	if (DY23 < 0 || (DY23 == 0 && DX23 > 0)) C2++;
	if (DY31 < 0 || (DY31 == 0 && DX31 > 0)) C3++;

	int CY1 = C1 + DX12 * (miny << 4) - DY12 * (minx << 4);
	int CY2 = C2 + DX23 * (miny << 4) - DY23 * (minx << 4);
	int CY3 = C3 + DX31 * (miny << 4) - DY31 * (minx << 4);

	unsigned short min_v = 9999;
	unsigned short max_v = 0;

	float y23_diff = (float)(v2_y - v3_y);
	float x32_diff = (float)(v3_x - v2_x);

	float y31_diff = (float)(v3_y - v1_y);
	float x13_diff = (float)(v1_x - v3_x);
	float y13_diff = (float)(v1_y - v3_y);

	float den = y23_diff*x13_diff + x32_diff*y13_diff;

	if (den == 0.0f)
		return;

	for (int y = miny; y < maxy; y++)
	{
		int CX1 = CY1;
		int CX2 = CY2;
		int CX3 = CY3;
		float term21 = x32_diff * (y - v3_y);
		float term22 = x13_diff * (y - v3_y);
		for (int x = minx; x < maxx; x++)
		{
			if (CX1 >= 0 && CX2 >= 0 && CX3 >= 0)
			{
				float w1 = (y23_diff * (x - v3_x) + term21) / den;
				float w2 = (y31_diff * (x - v3_x) + term22) / den;
				float w3 = 1.0f - w1 - w2;

				unsigned short d = depth_map[x];
				unsigned short val = (unsigned short)(v1_d * w1 + v2_d * w2 + v3_d * w3);
				if (d == 0 || val < depth_map[x])
				{
					depth_map[x] = val;
					tag2_map[x + y*w] = tag2;
				}
			}

			CX1 -= FDY12;
			CX2 -= FDY23;
			CX3 -= FDY31;
		}

		CY1 += FDX12;
		CY2 += FDX23;
		CY3 += FDX31;
		depth_map += w;
		//tag2_map += w;

	}
}
void generateVerticesFromDepthMaps(unsigned char* depth_maps, unsigned char *depth_colors, int *widths, int *heights,
	vector<WorldTransformation> &world_transforms, vector<IntrinsicCameraParameters> &intrinsic_params, vector<VerticesWithDepthColorMaps> &vertices_with_maps,
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

void pointProjection(Point3f &p, int &out_x, int &out_y, unsigned short &out_d, WorldTransformation &wt, IntrinsicCameraParameters &ip)
{
	Point3f tmp = p;
	RotatePoint(tmp, wt.R);
	tmp.X += wt.t[0];
	tmp.Y += wt.t[1];
	tmp.Z += wt.t[2];

	out_x = static_cast<int>((tmp.X * ip.fx) / tmp.Z + ip.cx + 0.5);
	out_y = static_cast<int>(ip.cy - (tmp.Y * ip.fy) / tmp.Z + 0.5);
	out_d = static_cast<unsigned short>(min(max(0, (int)(tmp.Z * 1000.0f)), 65535));
}

void projectVerticesIntoDepthMap(VerticesWithDepthColorMaps &vertices_with_maps, WorldTransformation &wt, IntrinsicCameraParameters &ip, int w, int h, bool includeAssigned)
{
	vector<Point3f> &vertices = vertices_with_maps.vertices;
	int n_vertices = (int)vertices.size();

	vector<unsigned short> &out_depth_map = vertices_with_maps.depth_map;

	for (int i = 0; i < w*h; i++)
	{
		vertices_with_maps.depth_to_vertices_map[i] = -1;
		out_depth_map[i] = 0;
	}

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
	int minX, int maxX, int minY, int maxY, int indexes_shift, WorldTransformation wt, IntrinsicCameraParameters ip,
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

/*
void mapDepthMap(VerticesWithDepthColorMaps &vertices_with_maps, WorldTransformation out_wt, IntrinsicCameraParameters out_ip, vector<unsigned short> &out_depth_map, int w, int h, vector<unsigned short> &confidence_map)
{
vector<TriangleIndexes> indexes;
MeshGenerator::generateTrianglesGradients(vertices_with_maps.depth_map.data(), vertices_with_maps.depth_to_vertices_map,
indexes, w, h);
vector<Point3f> &vertices = vertices_with_maps.vertices;

out_wt.inv();

vector<unsigned short> depth_map(w*h, 0);
size_t n_vertices = vertices.size();
vector<int> xs(n_vertices, 0);
vector<int> ys(n_vertices, 0);
vector<int> ds(n_vertices, 0);
vector<unsigned short> confidence(n_vertices, 0);

for (int v = 0; v < n_vertices; v++)
{
if (vertices_with_maps.point_assigned[v])
continue;

int x, y;
unsigned short d;
int overlay_vertice_to_pos = vertices_with_maps.vertices_to_depth_map[v];
pointProjection(vertices[v], x, y, d, out_wt, out_ip);
if (x < 1 || x >= w || y < 1 || y >= h || d == 0)
continue;

ds[v] = d;
xs[v] = x;
ys[v] = y;
confidence[v] = vertices_with_maps.confidence_map[overlay_vertice_to_pos];
}

size_t n_triangles = indexes.size();
for (int t = 0; t < n_triangles; t++)
{
int *inds = indexes[t].ind;
unsigned short d1, d2, d3;
int i1 = inds[0];
int i2 = inds[1];
int i3 = inds[2];

d1 = ds[i1];
d2 = ds[i2];
d3 = ds[i3];

if f== 0 || d2 == 0 || d3 == 0)
continue;

unsigned short confidence_val = (confidence[i1] + confidence[i2] + confidence[i3]) / 3;
drawTriangle(xs[i1], ys[i1], d1, xs[i2], ys[i2], d2, xs[i3], ys[i3], d3, depth_map.data(), w, h, confidence_map.data(), confidence_val);
}

out_depth_map = depth_map;
#ifdef 	DEBUG_IMAGES
writeDepthImage(out_depth_map, w, h, "test/last_mapped_depth.pgm");
#endif

}
*/

vector<vector<float>> multiply3x3Matrices(vector<vector<float>> &M1, vector<vector<float>> &M2)
{
	vector<vector<float>> outM(3, vector<float>(3, 0.0f));

	for (int j = 0; j < 3; j++) // column
		for (int i = 0; i < 3; i++)		// row
			for (int k = 0; k < 3; k++)
				outM[i][j] += M1[i][k] * M2[k][j];
	return outM;
}

void mapPointToDifferentDepthMap(int src_x, int src_y, unsigned short src_d, int &out_x, int &out_y, unsigned short &out_d,
	vector<vector<float>> &totalR, WorldTransformation &in_wt, IntrinsicCameraParameters &in_ip,
	WorldTransformation &out_wt, IntrinsicCameraParameters &out_ip)
{
	Point3f p;

	p.Z = src_d / 1000.0f;
	p.X = ((src_x - in_ip.cx) / in_ip.fx) * p.Z + in_wt.t[0];
	p.Y = ((in_ip.cy - src_y) / in_ip.fy) * p.Z + in_wt.t[1];
	p.Z += in_wt.t[2];

	RotatePoint(p, totalR);

	p.X += out_wt.t[0];
	p.Y += out_wt.t[1];
	p.Z += out_wt.t[2];

	out_x = static_cast<int>((p.X * out_ip.fx) / p.Z + out_ip.cx + 0.5);
	out_y = static_cast<int>(out_ip.cy - (p.Y * out_ip.fy) / p.Z + 0.5);
	out_d = static_cast<unsigned short>(min(max(0, (int)(p.Z * 1000.0f)), 65535));

}

void mapPointToDifferentDepthMapf(int src_x, int src_y, unsigned short src_d, float &out_x, float &out_y, float &out_d,
	vector<vector<float>> &totalR, WorldTransformation &in_wt, IntrinsicCameraParameters &in_ip,
	WorldTransformation &out_wt, IntrinsicCameraParameters &out_ip)
{
	Point3f p;

	p.Z = src_d / 1000.0f;
	p.X = ((src_x - in_ip.cx) / in_ip.fx) * p.Z + in_wt.t[0];
	p.Y = ((in_ip.cy - src_y) / in_ip.fy) * p.Z + in_wt.t[1];
	p.Z += in_wt.t[2];

	RotatePoint(p, totalR);

	p.X += out_wt.t[0];
	p.Y += out_wt.t[1];
	p.Z += out_wt.t[2];

	out_x = (p.X * out_ip.fx) / p.Z + out_ip.cx;
	out_y = out_ip.cy - (p.Y * out_ip.fy) / p.Z;
	out_d = min(max(0.0f, p.Z * 1000.0f), 65535.0f);

}


void mapDepthMap(VerticesWithDepthColorMaps &vertices_with_maps, WorldTransformation out_wt, IntrinsicCameraParameters out_ip, WorldTransformation in_wt, IntrinsicCameraParameters in_ip,
	vector<unsigned short> &out_depth_map, int w, int h, vector<unsigned short> &confidence_map)
{
	vector<TriangleIndexes> indexes;
	vector<unsigned short> source_depth_map = vertices_with_maps.depth_map;
	MeshGenerator::generateTrianglesGradients(source_depth_map.data(), indexes, w, h);
	vector<Point3f> &vertices = vertices_with_maps.vertices;


	vector<unsigned short> depth_map(w*h, 0);
	size_t n_vertices = vertices.size();
	vector<int> xs(w*h, 0);
	vector<int> ys(w*h, 0);
	vector<int> ds(w*h, 0);
	vector<unsigned short> confidence(w*h, 0);

	vector<vector<float>> totalR;

	//out_wt.inv();
	in_wt.inv();

	totalR = multiply3x3Matrices(out_wt.R, in_wt.R);

	//totalR = multiply3x3Matrices(out_wt.R, in_wt.R);

	for (int src_y = 0; src_y < h; src_y++)
	{
		for (int src_x = 0; src_x < w; src_x++)
		{
			int el = src_x + src_y*w;

			int src_d = source_depth_map[el];
			if (src_d == 0 || vertices_with_maps.point_assigned[el])
				continue;

			int x, y;
			unsigned short d;

			mapPointToDifferentDepthMap(src_x, src_y, src_d, x, y, d,
				totalR, in_wt, in_ip, out_wt, out_ip);

			if (x < 1 || x >= w || y < 1 || y >= h || d == 0)
				continue;

			//depth_map[x + y * w] = d;

			ds[el] = d;
			xs[el] = x;
			ys[el] = y;
			confidence[el] = vertices_with_maps.confidence_map[el];
		}
	}

	size_t n_triangles = indexes.size();
	for (int t = 0; t < n_triangles; t++)
	{
		int *inds = indexes[t].ind;
		unsigned short d1, d2, d3;
		int i1 = inds[0];
		int i2 = inds[1];
		int i3 = inds[2];

		d1 = ds[i1];
		d2 = ds[i2];
		d3 = ds[i3];

		if (d1 == 0 || d2 == 0 || d3 == 0)
			continue;

		unsigned short confidence_val = (confidence[i1] + confidence[i2] + confidence[i3]) / 3;

		drawTriangle(xs[i1], ys[i1], d1, xs[i2], ys[i2], d2, xs[i3], ys[i3], d3, depth_map.data(), w, h, confidence_map.data(), confidence_val);
	}

	out_depth_map = depth_map;

#ifdef 	DEBUG_IMAGES
	writeDepthImage(out_depth_map, w, h, "test/last_mapped_depth.pgm");
#endif
}


void morphologyErode(vector<unsigned char> &replace_mask, int w, int h)
{
	int n_shifts = 8;
	int shift_x[] = { -1, 0 ,  1, -1, 1, -1, 0, 1 };
	int shift_y[] = { -1, -1, -1,  0, 0,  1, 1, 1 };

	vector<unsigned char> mask_copy = replace_mask;

	for (int j = 1; j<h - 1; j++)
		for (int i = 1; i < w - 1; i++)
		{
			int pos = i + j*w;
			if (replace_mask[pos] == 0)
				continue;
			for (int shift = 0; shift < n_shifts; shift++)
			{
				int shifted_pos = pos + shift_x[shift] + w * shift_y[shift];
				if (replace_mask[shifted_pos] == 0)
				{
					mask_copy[pos] = 0;
					break;
				}
			}
		}

	replace_mask = mask_copy;
}

void morphologyDilate(vector<unsigned char> &replace_mask, int w, int h)
{
	int n_shifts = 8;
	int shift_x[] = { -1, 0 ,  1, -1, 1, -1, 0, 1 };
	int shift_y[] = { -1, -1, -1,  0, 0,  1, 1, 1 };

	vector<unsigned char> mask_copy = replace_mask;

	for (int j = 1; j<h - 1; j++)
		for (int i = 1; i < w - 1; i++)
		{
			int pos = i + j*w;
			if (replace_mask[pos] == 255)
				continue;
			for (int shift = 0; shift < n_shifts; shift++)
			{
				int shifted_pos = pos + shift_x[shift] + w * shift_y[shift];
				if (replace_mask[shifted_pos] == 255)
				{
					mask_copy[pos] = 255;
					break;
				}
			}
		}

	replace_mask = mask_copy;
}

void morphologyDilate(vector<unsigned short> &depth_map, int w, int h)
{
	int n_shifts = 8;
	int shift_x[] = { -1, 0 ,  1, -1, 1, -1, 0, 1 };
	int shift_y[] = { -1, -1, -1,  0, 0,  1, 1, 1 };

	vector<unsigned short> map_copy = depth_map;

	for (int j = 1; j<h - 1; j++)
		for (int i = 1; i < w - 1; i++)
		{
			int pos = i + j*w;

			for (int shift = 0; shift < n_shifts; shift++)
			{
				int shifted_pos = pos + shift_x[shift] + w * shift_y[shift];
				map_copy[pos] = max(map_copy[pos], depth_map[shifted_pos]);
			}
		}

	depth_map = map_copy;
}

void morphologyErode(vector<unsigned short> &depth_map, int w, int h)
{
	int n_shifts = 8;
	int shift_x[] = { -1, 0 ,  1, -1, 1, -1, 0, 1 };
	int shift_y[] = { -1, -1, -1,  0, 0,  1, 1, 1 };

	vector<unsigned short> map_copy = depth_map;

	for (int j = 1; j<h - 1; j++)
		for (int i = 1; i < w - 1; i++)
		{
			int pos = i + j*w;

			for (int shift = 0; shift < n_shifts; shift++)
			{
				int shifted_pos = pos + shift_x[shift] + w * shift_y[shift];
				map_copy[pos] = min(map_copy[pos], depth_map[shifted_pos]);
			}
		}

	depth_map = map_copy;
}
/*
void assignDepthMapOverlay(vector<VerticesWithDepthColorMaps> &vertices_with_maps,
vector<int> &depth_to_vertices_map, vector<vector<unsigned char>> &map_indexes, vector<WorldTransformation> &wt, vector<IntrinsicCameraParameters> &ip, int overlayed_index, int base_map_index, int w, int h)
{
//const int depth_threshold = 20;
int depth_threshold;

vector<unsigned short> &depth_map = vertices_with_maps[base_map_index].depth_map;
WorldTransformation base_wt = wt[base_map_index];
base_wt.inv();

vector<unsigned short> &base_confidence = vertices_with_maps[base_map_index].confidence_map;
vector<unsigned short> overlay_confidence(w*h);

const int confidence_threshold = 10;

vector<unsigned short> mapped_depth_map;

mapDepthMap(vertices_with_maps[overlayed_index], wt[base_map_index], ip[base_map_index], wt[overlayed_index], ip[overlayed_index],
mapped_depth_map, w, h, overlay_confidence);

vector<unsigned char> replace_mask(w * h, 0);
vector<PointProjection> projections;
int n_points_assigned = 0;

// test code
#ifdef 	DEBUG_IMAGES
writeDepthImage(overlay_confidence, w, h, "test/confidence_1.pgm");
vector<SimpleImage> testImages(vertices_with_maps.size());
for (int i = 0; i < vertices_with_maps.size(); i++)
{
testImages[i].create(w, h, 3, nullptr);
for (int el = 0; el < w*h; el++)
testImages[i].data_ptr[el * 3 + 1] = (unsigned char)vertices_with_maps[i].depth_map[el];
}
#endif

for (int el = 0; el < w*h; el++)
{
if (depth_map[el] == 0 || vertices_with_maps[base_map_index].point_assigned[el])
continue;

depth_threshold = (int)(depth_map[el] * 0.00272 + 7.273) + 13.727;
//depth_threshold = 20;
int diff = abs(depth_map[el] - mapped_depth_map[el]);
if (diff < depth_threshold)
{
if ((overlay_confidence[el] > confidence_threshold
|| overlay_confidence[el] >= base_confidence[el]) && base_confidence[el] > 2)
{
#ifdef 	DEBUG_IMAGES
testImages[base_map_index].data_ptr[el * 3] = 255;
testImages[base_map_index].data_ptr[el * 3 + 1] = 0;
testImages[base_map_index].data_ptr[el * 3 + 2] = 0;
#endif
replace_mask[el] = 255;
}
}
}

#ifdef DEBUG_IMAGES
writePGM("test/replace_mask_1.pgm", w, h, replace_mask.data());
#endif

morphologyErode(replace_mask, w, h);
morphologyErode(replace_mask, w, h);
morphologyDilate(replace_mask, w, h);
morphologyDilate(replace_mask, w, h);
//morphologyDilate(replace_mask, w, h);

#ifdef DEBUG_IMAGES
writePGM("test/replace_mask_2.pgm", w, h, replace_mask.data());
#endif

for (int i=0; i<w*h; i++)
if (replace_mask[i] != 0)
{
if (replace_mask[i] == 255)
{
depth_map[i] = vertices_with_maps[overlayed_index].depth_map[i];
base_confidence[i] = 0;
depth_map[i] = 0;
vertices_with_maps[base_map_index].point_assigned[i] = true;
}
n_points_assigned++;
}

#ifdef DEBUG_IMAGES
for (int i = 0; i < testImages.size(); i++)
{
char tmp[1024];
sprintf(tmp, "test/outTest%d.png", i);
testImages[i].writeToFile(tmp);
}
#endif
}
*/

template <class T>
float getSubpixelValue(vector<T> &map, float x, float y, int w, int h)
{
	float x0, y0;
	float x_r, y_r;
	float val = 0.0;
	float diff_x, diff_y;

	if (x<0.0 || y<0.0 || x>(w - 2) || y>(h - 2)) return val;

	x0 = floor(x);
	y0 = floor(y);

	int pos = (unsigned short)x0 + (unsigned short)y0 * w;

	x_r = x - x0;
	y_r = y - y0;

	diff_x = 1.0f - x_r;
	diff_y = 1.0f - y_r;

	val = diff_x*diff_y*map[pos] + x_r*diff_y*map[pos + 1] +
		diff_x*y_r*map[pos + w] + x_r*y_r*map[pos + w + 1];

	return val;
}

void assignDepthMapOverlay(vector<VerticesWithDepthColorMaps> &vertices_with_maps,
	vector<int> &depth_to_vertices_map, vector<vector<unsigned char>> &map_indexes, vector<WorldTransformation> &wt, vector<IntrinsicCameraParameters> &ip, int overlayed_index, int base_map_index, int w, int h)
{
	//const int depth_threshold = 20;
	int depth_threshold;

	vector<unsigned short> &depth_map = vertices_with_maps[base_map_index].depth_map;
	WorldTransformation base_wt = wt[base_map_index];
	base_wt.inv();

	vector<vector<float>> totalR = multiply3x3Matrices(wt[overlayed_index].R, base_wt.R);

	vector<unsigned char> &base_confidence = vertices_with_maps[base_map_index].confidence_map;
	unsigned char overlay_confidence;

	const int confidence_threshold = 10;

	vector<unsigned char> replace_mask(w * h, 0);
	vector<PointProjection> projections;
	int n_points_assigned = 0;

	// test code
#ifdef 	DEBUG_IMAGES
	//writeDepthImage(overlay_confidence, w, h, "test/confidence_1.pgm");
	vector<SimpleImage> testImages(vertices_with_maps.size());
	for (int i = 0; i < vertices_with_maps.size(); i++)
	{
		testImages[i].create(w, h, 3, nullptr);
		for (int el = 0; el < w*h; el++)
			testImages[i].data_ptr[el * 3 + 1] = (unsigned char)vertices_with_maps[i].depth_map[el];
	}
#endif

	for (int el = 0; el < w*h; el++)
	{
		unsigned short d = depth_map[el];
		if (d == 0 || vertices_with_maps[base_map_index].point_assigned[el])
			continue;

		float out_x, out_y, out_depth;
		mapPointToDifferentDepthMapf(el % w, el / w, d, out_x, out_y, out_depth, totalR, base_wt,
			ip[base_map_index], wt[overlayed_index], ip[overlayed_index]);

		if (out_x < 0 || out_y < 0 || out_x >= (w - 1) || out_y >= (h - 1))
			continue;

		//int pos_overlay = (int)(out_x) + (int)(out_y) * w;

		overlay_confidence = (unsigned char)(getSubpixelValue(vertices_with_maps[overlayed_index].confidence_map, out_x,
			out_y, w, h) + 0.5f);
		float org_depth_overlay = getSubpixelValue(vertices_with_maps[overlayed_index].depth_map, out_x,
			out_y, w, h);

		depth_threshold = (int)(depth_map[el] * 0.00272 + 20);

		float diff = abs(org_depth_overlay - out_depth);
		if (diff < depth_threshold)
		{
			if ((overlay_confidence > confidence_threshold
				|| overlay_confidence >= base_confidence[el]) && base_confidence[el] > 2)
			{
#ifdef 	DEBUG_IMAGES
				testImages[base_map_index].data_ptr[el * 3] = 255;
				testImages[base_map_index].data_ptr[el * 3 + 1] = 0;
				testImages[base_map_index].data_ptr[el * 3 + 2] = 0;
#endif				
				replace_mask[el] = 255;
			}
		}
	}

#ifdef DEBUG_IMAGES
	writePGM("test/replace_mask_1.pgm", w, h, replace_mask.data());
#endif

	morphologyErode(replace_mask, w, h);
	morphologyErode(replace_mask, w, h);
	morphologyDilate(replace_mask, w, h);
	morphologyDilate(replace_mask, w, h);
	//morphologyDilate(replace_mask, w, h);

#ifdef DEBUG_IMAGES
	writePGM("test/replace_mask_2.pgm", w, h, replace_mask.data());
#endif

	for (int i = 0; i<w*h; i++)
		if (replace_mask[i] != 0)
		{
			if (replace_mask[i] == 255)
			{
				depth_map[i] = 0;
				vertices_with_maps[base_map_index].point_assigned[i] = true;
			}
			n_points_assigned++;
		}

#ifdef DEBUG_IMAGES
	for (int i = 0; i < testImages.size(); i++)
	{
		char tmp[1024];
		sprintf(tmp, "test/outTest%d.png", i);
		testImages[i].writeToFile(tmp);
	}
#endif
}

float calculateCosAngleBetweenVectors(float X0, float Y0, float Z0,
	float X1, float Y1, float Z1, float X2, float Y2, float Z2)
{
	float v1[3] = { X1 - X0, Y1 - Y0, Z1 - Z0 };
	float v2[3] = { X2 - X0, Y2 - Y0, Z2 - Z0 };

	float dot = v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
	float mag1 = sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2] + 0.000001f);
	float mag2 = sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2] + 0.000001f);

	return dot / (mag1 * mag2 + 0.000001f);
}


void clearMapConvexHull(vector<VerticesWithDepthColorMaps> &vertices_with_maps, vector<WorldTransformation> &world_transforms,
	vector<IntrinsicCameraParameters> &intrinsic_params, int overlayed_index, int base_map_index, int w, int h,
	vector<unsigned short> &cleared_depth_map)
{
	const int depth_threshold = 20;

	vector<unsigned short> &depth_map = cleared_depth_map;
	//vector<unsigned short> depth_map_copy = vertices_with_maps[base_map_index].depth_map;
	vector<unsigned short> overlay_depth_map = vertices_with_maps[overlayed_index].depth_map;
	vector<unsigned char> &base_confidence = vertices_with_maps[base_map_index].confidence_map;
	vector<unsigned short> overlay_confidence(w*h);


	const int confidence_threshold = 10;

	vector<vector<float>> totalRBaseToOverlay, totalROverlayToBase;
	WorldTransformation overlay_wt = world_transforms[overlayed_index];
	WorldTransformation overlay_wt_inv = world_transforms[overlayed_index];
	WorldTransformation base_wt = world_transforms[base_map_index];
	WorldTransformation base_wt_inv = world_transforms[base_map_index];
	IntrinsicCameraParameters overlay_ip = intrinsic_params[overlayed_index];
	IntrinsicCameraParameters base_ip = intrinsic_params[base_map_index];

	const int shifts[] = { -w - 1, -w, -w + 1, -1, 1, w - 1, w, w + 1 };
	const int n_shifts = 8;

	overlay_wt_inv.inv();
	base_wt_inv.inv();
	totalRBaseToOverlay = multiply3x3Matrices(overlay_wt.R, base_wt_inv.R);

	totalROverlayToBase = multiply3x3Matrices(base_wt.R, overlay_wt_inv.R);

#ifdef DEBUG_IMAGES
	writeDepthImage(overlay_depth_map, w, h, "test/before_dilate.pgm");
#endif
	//	morphologyDilate(overlay_depth_map, w, h);
#ifdef DEBUG_IMAGES
	writeDepthImage(overlay_depth_map, w, h, "test/after_dilate.pgm");
#endif

	for (int y = 0; y<h; y++)
		for (int x = 0; x<w; x++)
		{
			int el = x + y*w;
			unsigned short d = depth_map[el];
			if (d == 0)
				continue;

			// check if point is in the other camera field of view
			int ov_x, ov_y;
			unsigned short ov_d;
			mapPointToDifferentDepthMap(x, y, d, ov_x, ov_y, ov_d,
				totalRBaseToOverlay, base_wt_inv, base_ip, overlay_wt, overlay_ip);

			if (ov_x < 5 || ov_y < 40 || ov_x >= (w - 5) || ov_y >= (h - 40) || ov_d == 0)
				continue;

			int n = 0;
			unsigned short ovd_org = overlay_depth_map[ov_x + ov_y * w];

			if (vertices_with_maps[overlayed_index].confidence_map[ov_x + ov_y] < 2)
				continue;

			if (ovd_org == 0)
			{
				depth_map[el] = 0;
				continue;
			}
			int depth_to_vert_overlay = vertices_with_maps[overlayed_index].depth_to_vertices_map[ov_x + ov_y * w];

			if (ovd_org > ov_d + 100)
			{
				depth_map[el] = 0;
				continue;
			}

		}
}


void clearAllMapsConvexHull(vector<VerticesWithDepthColorMaps> &vertices_with_maps, vector<WorldTransformation> &world_transforms,
	vector<IntrinsicCameraParameters> &intrinsic_params, int* widths, int *heights)
{
	int n_maps = (int)vertices_with_maps.size();
	vector<vector<unsigned short>> cleared_depth_maps(n_maps);

	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	{
		cleared_depth_maps[current_map_index] = vertices_with_maps[current_map_index].depth_map;
		int w = widths[current_map_index];
		int h = heights[current_map_index];
		WorldTransformation world_transform = world_transforms[current_map_index];

		for (int i = 0; i < n_maps; i++)
		{
			if (i == current_map_index)
				continue;

			clearMapConvexHull(vertices_with_maps, world_transforms,
				intrinsic_params, i, current_map_index, w, h, cleared_depth_maps[current_map_index]);
		}
	}

	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
		vertices_with_maps[current_map_index].depth_map = cleared_depth_maps[current_map_index];
}


void mergeVerticesForViews(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *widths, int *heights, vector<WorldTransformation> &world_transforms,
	vector<IntrinsicCameraParameters> &intrinsic_params)
{
	int n_maps = (int)vertices_with_maps.size();

	SimpleTimer tim;

	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	{
		int w = widths[current_map_index];
		int h = heights[current_map_index];
		WorldTransformation world_transform = world_transforms[current_map_index];
		projectVerticesIntoDepthMap(vertices_with_maps[current_map_index], world_transform, intrinsic_params[current_map_index], w, h, false);
	}

#ifdef DEBUG_IMAGES
	char tmp[1024];
	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	{
		sprintf(tmp, "test/depth_%d_1.pgm", current_map_index);
		writeDepthImage(vertices_with_maps[current_map_index].depth_map, widths[current_map_index], heights[current_map_index], tmp);
	}
#endif

	//for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	//removeEdgePixels(vertices_with_maps[current_map_index].depth_map, vertices_with_maps[current_map_index].confidence_map, widths[current_map_index], heights[current_map_index]);

	tim.start();
	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	{
		int w = widths[current_map_index];
		int h = heights[current_map_index];
		WorldTransformation world_transform = world_transforms[current_map_index];
		vector<int> depth_to_vertices_map(w * h);

#ifdef DEBUG_IMAGES
		sprintf(tmp, "test/test_depth_%d_1.pgm", current_map_index);
		writeDepthImage(vertices_with_maps[current_map_index].depth_map, w, h, tmp);
#endif

		depth_to_vertices_map = vertices_with_maps[current_map_index].depth_to_vertices_map;
		vector<vector<unsigned char>> map_indexes(h, vector<unsigned char>(w, current_map_index));

		for (int i = 0; i < n_maps; i++)
		{
			if (i == current_map_index)
				continue;

			assignDepthMapOverlay(vertices_with_maps, depth_to_vertices_map, map_indexes, world_transforms,
				intrinsic_params, i, current_map_index, w, h);
		}

#ifdef SHOW_TIMINGS
		tim.printLapTimeAndRestart("mergeVerticesForViews part ");
#endif

#ifdef DEBUG_IMAGES
		sprintf(tmp, "test/test_depth_%d_2.pgm", current_map_index);
		writeDepthImage(vertices_with_maps[current_map_index].depth_map, w, h, tmp);
#endif
	}
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

int calculateMapsCoverage(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *widths, int *heights, vector<WorldTransformation> &world_transforms,
	vector<IntrinsicCameraParameters> &intrinsic_params, int index1, int index2)
{
	vector<Point3f> &vertices = vertices_with_maps[index2].vertices;
	int n_vertices = (int)vertices.size();

	vector<unsigned short> &depth_map = vertices_with_maps[index1].depth_map;
	WorldTransformation wt = world_transforms[index1];
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

ColorCorrectionParams getColorCorrectionTransform(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *widths, int *heights, vector<WorldTransformation> &world_transforms,
	vector<IntrinsicCameraParameters> &intrinsic_params, int index1, int index2)
{
	vector<Point3f> &vertices = vertices_with_maps[index2].vertices;
	int n_vertices = (int)vertices.size();

	vector<unsigned char> &colors1 = vertices_with_maps[index1].colors;
	vector<unsigned char> &colors2 = vertices_with_maps[index2].colors;
	ColorCorrectionParams transform;

	vector<unsigned short> &depth_map = vertices_with_maps[index1].depth_map;
	WorldTransformation wt = world_transforms[index1];
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

void updateColorCorrectionCoefficients(vector<VerticesWithDepthColorMaps> &vertices_with_maps,
	int *widths, int *heights, vector<WorldTransformation> &world_transforms,
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
		vector<unsigned char> &colors = vertices_with_maps[i].colors;

		applyColorCorrection(colors, color_correction_coeffs[i]);
	}
}


void formMesh(Mesh *out_mesh, vector<VerticesWithDepthColorMaps> &vertices_with_maps, vector<vector<TriangleIndexes>> &triangle_indexes)
{
	int n_maps = (int)vertices_with_maps.size();
	size_t n_total_vertices = 0;
	size_t n_total_triangles = 0;

	vector<int> vertices_shift(n_maps); 

	for (int i = 0; i<n_maps; i++)
	{
		n_total_triangles += triangle_indexes[i].size();
		n_total_vertices += vertices_with_maps[i].vertices.size();
	}

	n_total_triangles += triangle_indexes[n_maps].size();  // stiches

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
		vertices_shift[i] = vertices_so_far;
		vertices_so_far += vertices_with_maps[i].vertices.size();
	}


	int act_triangle = 0;
	out_mesh->triangles = new int[n_total_triangles * 3];
	int act_vertices = 0;
	for (int i = 0; i < n_maps; i++)
	{
		for (int j = 0; j < triangle_indexes[i].size(); j++)
		{
			int inds[3];

			inds[0] = triangle_indexes[i][j].ind[0];
			inds[1] = triangle_indexes[i][j].ind[1];
			inds[2] = triangle_indexes[i][j].ind[2];

			inds[0] = vertices_with_maps[i].depth_to_vertices_map[inds[0]];
			inds[1] = vertices_with_maps[i].depth_to_vertices_map[inds[1]];
			inds[2] = vertices_with_maps[i].depth_to_vertices_map[inds[2]];

			if (inds[0] == -1 || inds[1] == -1 || inds[2] == -1)
				continue;

			inds[0] += act_vertices;
			inds[1] += act_vertices;
			inds[2] += act_vertices;

			memcpy(out_mesh->triangles + act_triangle * 3, inds, 3 * sizeof(int));
			act_triangle++;
		}
		act_vertices += (int)vertices_with_maps[i].vertices.size();
	}

	// stiches
	for (int j = 0; j < triangle_indexes[n_maps].size(); j++)
	{
		int inds[3], maps[3];

		inds[0] = triangle_indexes[n_maps][j].ind[0];
		inds[1] = triangle_indexes[n_maps][j].ind[1];
		inds[2] = triangle_indexes[n_maps][j].ind[2];

		maps[0] = triangle_indexes[n_maps][j].map_ind[0];
		maps[1] = triangle_indexes[n_maps][j].map_ind[1];
		maps[2] = triangle_indexes[n_maps][j].map_ind[2];

		inds[0] = vertices_with_maps[maps[0]].depth_to_vertices_map[inds[0]];
		inds[1] = vertices_with_maps[maps[1]].depth_to_vertices_map[inds[1]];
		inds[2] = vertices_with_maps[maps[2]].depth_to_vertices_map[inds[2]];
		
		if (inds[0] == -1 || inds[1] == -1 || inds[2] == -1)
			continue;

		inds[0] += vertices_shift[maps[0]];
		inds[1] += vertices_shift[maps[1]];
		inds[2] += vertices_shift[maps[2]];

		float dist = 0.5;
		if (abs(out_mesh->vertices[inds[0]].X - out_mesh->vertices[inds[1]].X) > dist ||
			abs(out_mesh->vertices[inds[0]].Y - out_mesh->vertices[inds[1]].Y) > dist ||
			abs(out_mesh->vertices[inds[0]].Z - out_mesh->vertices[inds[1]].Z) > dist)
			inds[0] = inds[0];


		memcpy(out_mesh->triangles + act_triangle * 3, inds, 3 * sizeof(int));
		act_triangle++;
	}

	out_mesh->nTriangles = act_triangle;

}

extern "C" DEPTH_PROCESSING_API void __stdcall generateVerticesFromDepthMap(unsigned char* depth_maps,
	unsigned char *depth_colors, int *widths, int *heights, float *intr_params, float *wtransform_params, Mesh *out_mesh,
	float minX, float minY, float minZ, float maxX, float maxY, float maxZ, int depth_map_index)
{
	int depth_pos = 0, colors_pos = 0;;

	vector<VerticesWithDepthColorMaps> vertices_with_maps(1);

	IntrinsicCameraParameters intrinsic_params;
	WorldTransformation world_transform;
	vector<vector<TriangleIndexes>> triangle_indexes(1);

	intrinsic_params = IntrinsicCameraParameters(intr_params + depth_map_index * 7);
	world_transform = WorldTransformation(wtransform_params + depth_map_index*(9 + 3));
	world_transform.inv();

	for (int i = 0; i < depth_map_index; i++)
	{
		depth_pos += widths[i] * heights[i] * sizeof(unsigned short);
		colors_pos += widths[i] * heights[i] * sizeof(unsigned char) * 3;
	}

	createVertices((unsigned short*)(depth_maps + depth_pos), depth_colors + colors_pos, widths[depth_map_index],
		heights[depth_map_index], intrinsic_params, world_transform,
		std::ref(vertices_with_maps[0]), minX, minY, minZ, maxX, maxY, maxZ);

	formMesh(out_mesh, vertices_with_maps, triangle_indexes);
}

void reprojectionCorrection(vector<VerticesWithDepthColorMaps> &vertices_with_maps,
	vector<VerticesWithDepthColorMaps> &original_vertices_with_maps, int *heights, int *widths,
	vector<vector<TriangleIndexes>> &triangle_indexes, vector<WorldTransformation> &wt, vector<IntrinsicCameraParameters> &ip)
{
	int n_maps = (int)vertices_with_maps.size();
	vector<unsigned short> confidences(n_maps);
	vector<unsigned short> depths(n_maps);
	vector<unsigned char> colors(n_maps * 3);
	vector<int> map_indexes(n_maps);
	vector<WorldTransformation> wt_inv = wt;
	int n_matches;
	int w = widths[0];
	int h = heights[0];

	for (int map_index = 0; map_index < n_maps; map_index++)
		wt_inv[map_index].inv();

	for (int map_index = 0; map_index < n_maps; map_index++)
	{
		size_t n_vertices = vertices_with_maps[map_index].vertices.size();

		for (int v = 0; v < n_vertices; v++)
		{
			Point3f &p = vertices_with_maps[map_index].vertices[v];
			unsigned char *c = vertices_with_maps[map_index].colors.data() + v * 3;

			n_matches = 0;
			for (int i = 0; i < n_maps; i++)
			{
				int x, y;
				unsigned short d;
				pointProjection(p, x, y, d, wt[i], ip[i]);
				if (d == 0 || x < 0 || y < 0 || x >= w || y >= h)
					continue;

				unsigned short org_depth = vertices_with_maps[i].depth_map[(x + y * w)];

				if (abs(d - org_depth) > 20)
					continue;

				depths[n_matches] = org_depth;
				colors[n_matches * 3] = vertices_with_maps[i].colors_map[(x + y * w) * 3];
				colors[n_matches * 3 + 1] = vertices_with_maps[i].colors_map[(x + y * w) * 3 + 1];
				colors[n_matches * 3 + 2] = vertices_with_maps[i].colors_map[(x + y * w) * 3 + 2];
				confidences[n_matches] = vertices_with_maps[i].confidence_map[x + y * w];
				map_indexes[n_matches] = i;
				n_matches++;

			}


			if (n_matches > 0)
			{
				int best_index = -1;
				int sum_depth = 0;
				int n_chosen = 0;
				unsigned short best_confidence = 0;
				for (int i = 0; i < n_matches; i++)
				{
					if (confidences[i] > best_confidence)
					{
						best_confidence = confidences[i];
						best_index = 0;
						n_chosen++;
					}
				}

				if (best_index != -1)
				{

					c[0] = colors[best_index * 3];
					c[1] = colors[best_index * 3 + 1];
					c[2] = colors[best_index * 3 + 2];
				}
			}
		}
	}
}


int generateTriangles(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *heights, int *widths,
	vector<vector<TriangleIndexes>> &triangle_indexes, vector<WorldTransformation> &world_transforms,
	vector<IntrinsicCameraParameters> &intrinsic_params)
{
	size_t n_maps = vertices_with_maps.size();
	vector<std::thread> generate_triangles_threads;
	int depth_pos = 0;
	int n_triangles = 0;

	for (int i = 0; i < n_maps; i++)
	{
		//writeDepthImage(vertices_with_maps[i].depth_map, widths[0], heights[0], "test/dt.pgm");
		int n_pixels = widths[i] * heights[i];

		generate_triangles_threads.push_back(thread(MeshGenerator::generateTrianglesGradients, (unsigned short*)vertices_with_maps[i].depth_map.data()
			, std::ref(triangle_indexes[i]), widths[i], heights[i]));

		depth_pos += n_pixels * 2;
	}

	triangle_indexes[n_maps] = generateTrianglesForStiches(vertices_with_maps, heights, widths, world_transforms, intrinsic_params);

	for (int i = 0; i < n_maps; i++)
	{
		generate_triangles_threads[i].join();
		n_triangles += (int)triangle_indexes[i].size();
	}


	return n_triangles;
}

void performICP(vector<VerticesWithDepthColorMaps> &vertices_with_maps, vector<WorldTransformation> &world_transforms, int refine_iters)
{
	vector<vector<float>> Rs(vertices_with_maps.size(), vector<float>(9, 0.0f));
	vector<vector<float>> ts(vertices_with_maps.size(), vector<float>(3, 0.0f));

	for (int i = 0; i<vertices_with_maps.size(); i++)
		for (int j = 0; j < 3; j++)
		{
			Rs[i][j * 3 + j] = 1;
			ts[i][j] = 0;
		}

	for (int refineIter = 0; refineIter < refine_iters; refineIter++)
	{
		for (int i = 0; i < vertices_with_maps.size(); i++)
		{
			vector<Point3f> points_rest;

			for (int j = 0; j < vertices_with_maps.size(); j++)
			{
				if (j == i)
					continue;

				points_rest.insert(points_rest.end(), vertices_with_maps[j].vertices.begin(), vertices_with_maps[j].vertices.end());
			}

			ICP(points_rest.data(),
				vertices_with_maps[i].vertices.data(),
				(int)points_rest.size(), (int)vertices_with_maps[i].vertices.size(),
				Rs[i].data(), ts[i].data());
		}
	}

	//Update the calibration data 
	for (int i = 0; i < world_transforms.size(); i++)
	{
		vector<float> tempT(3, 0.0f);
		vector<vector<float>> tempR(3, vector<float>(3, 0.0f));

		for (int j = 0; j < 3; j++)
		{
			for (int k = 0; k < 3; k++)
				tempT[j] += ts[i][k] * world_transforms[i].R[j][k];

			world_transforms[i].t[j] -= tempT[j];
		}

		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
			{
				for (int l = 0; l < 3; l++)
				{
					tempR[j][k] += Rs[i][l * 3 + j] * world_transforms[i].R[k][l];
				}
				world_transforms[i].R[k][j] = tempR[j][k];
			}
	}
}


inline bool edgePixel(int x, int y, int w, int h, unsigned short *data)
{
	const int n_shifts = 8;
	const int shifts[] = { -w - 1, -w, -w + 1, 1, w + 1, w, w - 1, -1 };
	int pos = x + y * w;

	if (data[pos] == 0)
		return false;

	for (int i = 0; i < n_shifts; i++)
		if (data[pos + shifts[i]] == 0)
			return true;

	return false;
}

/*
void getEdgeSegments(vector<unsigned char> &edges, int w, int h, int n_edges, vector<int> &positions_x, vector<int> &positions_y, vector<int> &segment_indexes)
{
	const int n_shifts = 8;
	const int shifts_x[] = {  1, 1, 1, 0, -1, -1, -1,  0 };
	const int shifts_y[] = { -1, 0, 1, 1,  1,  0, -1, -1 };

	positions_x.resize(n_edges);
	positions_y.resize(n_edges);
	segment_indexes.resize(n_edges); 
	int current_index = 0;
	int current_edge = 0;

	vector<unsigned char> visited(w*h, false);

	for (int y=0; y<h; y++)
		for (int x = 0; x < w; x++)
		{
			int pos = x + y*w;
			int cur_x = x;
			int cur_y = y;

			if (visited[pos] || edges[pos] == 0)
				continue; 

			positions_x[current_edge] = cur_x;
			positions_y[current_edge] = cur_y;
			segment_indexes[current_edge] = current_index; 
			visited[pos] = true;
			current_edge++;
			bool edge_found; 

			while (true)
			{
				edge_found = false; 
				for (int shift = 0; shift < n_shifts; shift++)
				{
					int shifted_x = cur_x + shifts_x[shift];
					int shifted_y = cur_y + shifts_y[shift];
					if (shifted_x < 0 || shifted_y < 0 || shifted_x >= w || shifted_y >= h)
						continue; 

					int shifted_pos = shifted_x + shifted_y * w; 

					if (visited[shifted_pos] || edges[shifted_pos] == 0)
						continue; 

					positions_x[current_edge] = shifted_x; 
					positions_y[current_edge] = shifted_y;
					segment_indexes[current_edge] = current_index; 
					visited[shifted_pos] = true; 
					cur_x = shifted_x;
					cur_y = shifted_y;

					if (cur_x == 299 && cur_y == 207)
						cur_x = cur_x;

					current_edge++;
					edge_found = true; 
					break; 
				}
				if (!edge_found)
					break; 
			}

			current_index++;
		}

}
*/

vector<unsigned char> getEdges(int w, int h, unsigned short *depth_map)
{
	vector<unsigned char> current_edges(w * h, 0);

	const int shifts[] = { -w - 1, -w, -w + 1, 1, w + 1, w, w - 1, -1 };

	for (int y = 1; y<h - 1; y++)
		for (int x = 1; x < w - 1; x++)
		{
			if (edgePixel(x, y, w, h, depth_map))
			{
				current_edges[x + y * w] = 255;
				continue;
			}
		}

	return current_edges; 
}


bool findEdgeConnection(int x, int y, int w, int h, unsigned short *first_depth_map, unsigned short *sec_depth_map,
	unsigned char *sec_edge_map, vector<vector<float>> &totalR, WorldTransformation &in_wt, WorldTransformation &out_wt,
	IntrinsicCameraParameters &in_ip, IntrinsicCameraParameters &out_ip, Connection &out_c)
{
	unsigned short src_d = first_depth_map[x + y*w];
	int out_x, out_y;
	unsigned short out_d;
	const int n_shifts = 9;
	const int shifts_x[] = { 0, -1,  0,  1, 1, 1, 0, -1, -1 };
	const int shifts_y[] = { 0, -1, -1, -1, 0, 1, 1,  1,  0 };

	const int depth_threshold = 20;

	mapPointToDifferentDepthMap(x, y, src_d, out_x, out_y, out_d,
		totalR, in_wt, in_ip, out_wt, out_ip);

	if (out_x < 1 || out_y < 1 || out_x > w - 1 || out_y > h - 1)
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


vector<TriangleIndexes> createTrianglesForSegment(int seed_x, int seed_y, int w, int h, unsigned short *first_depth_map, unsigned short *second_depth_map,
	unsigned char *first_edge_map, unsigned char *second_edge_map, vector<vector<float>> &totalR, WorldTransformation &in_wt, WorldTransformation &out_wt,
	IntrinsicCameraParameters &in_ip, IntrinsicCameraParameters &out_ip, Connection &c_1, vector<unsigned char> &points_visited, int depth_map_index1, int depth_map_index2)
{
	const int n_shifts = 8;
	const int shifts_x[] = { -1,  0,  1, 1, 1, 0, -1, -1 };
	const int shifts_y[] = { -1, -1, -1, 0, 1, 1,  1,  0 };
	const int depth_treshold = 20;
	int n_pos = 1;
	int n_new_pos = 0;
	vector<Connection> connections, new_connections;
	connections.reserve(w);
	new_connections.reserve(w);
	connections.push_back(c_1);
	vector<TriangleIndexes> triangles; 

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

				if (!findEdgeConnection(new_x, new_y, w, h, first_depth_map, second_depth_map,
					second_edge_map, totalR, in_wt, out_wt, in_ip, out_ip, c_2))
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


				if (tr[0]) triangles.push_back(TriangleIndexes(c_1.point1_index,      c_2.point2_index,      c_2.point1_index, 
															   depth_map_index1,      depth_map_index2,      depth_map_index1));
				if (tr[1]) triangles.push_back(TriangleIndexes(c_1.point1_index,      c_2.point2_index,      c_1.point2_index,
										   					   depth_map_index1,      depth_map_index2,      depth_map_index2));

				if (tr[2]) triangles.push_back(TriangleIndexes(c_1.point1_index, c_2.point1_index, c_1.point2_index,
															   depth_map_index1, depth_map_index1, depth_map_index2));
				
				if (tr[3]) triangles.push_back(TriangleIndexes(c_2.point1_index, c_2.point2_index, c_1.point2_index,
															   depth_map_index1, depth_map_index2, depth_map_index2));

				new_connections.push_back(c_2);
			}
		}
		connections = new_connections;
		new_connections.clear();
		n_pos = (int)connections.size();
	}

	return triangles; 
}


vector<TriangleIndexes> generateTrianglesForStiches(vector<VerticesWithDepthColorMaps> &vertices_with_maps, int *heights, int *widths, vector<WorldTransformation> &world_transforms,
	vector<IntrinsicCameraParameters> &intrinsic_params)
{
	int n_maps = (int)vertices_with_maps.size();
	vector<vector<unsigned char>> edges(n_maps);
	vector<int> n_edges(n_maps);

	for (int cur_map = 0; cur_map < n_maps; cur_map++)
		edges[cur_map] = getEdges(widths[cur_map], heights[cur_map], vertices_with_maps[cur_map].depth_map.data());

	vector<Connection> connections;

	vector<TriangleIndexes> all_triangles; 

	for (int first_map = 0; first_map < n_maps; first_map++)
	{
		unsigned short *first_depth_map = vertices_with_maps[first_map].depth_map.data();
		unsigned char *first_edge_map = edges[first_map].data();
		int w = widths[first_map];
		int h = heights[first_map];

		WorldTransformation in_wt = world_transforms[first_map];
		in_wt.inv();

		vector<unsigned char> points_visited(w*h, 0);

		for (int second_map = 0; second_map < n_maps; second_map++)
		{
			if (first_map == second_map)
				continue;
			WorldTransformation out_wt = world_transforms[second_map];
			vector<vector<float>> totalR = multiply3x3Matrices(out_wt.R, in_wt.R);
			unsigned short *second_depth_map = vertices_with_maps[second_map].depth_map.data();
			unsigned char *second_edge_map = edges[second_map].data();

			for (int y = 0; y < h; y++)
				for (int x = 0; x < w; x++)
				{
					if (points_visited[x + y*w] || first_edge_map[x + y*w] != 255)
						continue;

					Connection connection;

					if (!findEdgeConnection(x, y, w, h, first_depth_map, second_depth_map,
						second_edge_map, totalR, in_wt, out_wt, intrinsic_params[first_map], intrinsic_params[second_map],
						connection))
						continue;

					vector<TriangleIndexes> triangles = createTrianglesForSegment(x, y, w, h, first_depth_map, second_depth_map,
						first_edge_map, second_edge_map, totalR, in_wt, out_wt, intrinsic_params[first_map], intrinsic_params[second_map],
						connection, points_visited, first_map, second_map);

					all_triangles.insert(all_triangles.end(), triangles.begin(), triangles.end());

				}
		}
	}
	return all_triangles;

}



DEPTH_PROCESSING_API void __stdcall generateMeshFromDepthMaps(int n_maps, unsigned char* depth_maps,
	unsigned char *depth_colors, int *widths, int *heights, float *intr_params, float *wtransform_params, Mesh *out_mesh, bool bcolor_transfer,
	float minX, float minY, float minZ, float maxX, float maxY, float maxZ, bool bgenerate_triangles)
{
	int depth_pos = 0, colors_pos = 0;;


#ifdef STORE_FRAMES_INFORMATION
	// for testing purposes only
	storeAllFramesInformation("frames_info.bin", n_maps, depth_maps, depth_colors, widths, heights, intr_params, wtransform_params);
#endif

#ifdef LOAD_FRAMES_INFORMATION
	// for testing purposes only 
	string filenames[] = { "d:/Projekty/LiveScan3D/dane/gitara1.bin",   // 0
		"d:/Projekty/LiveScan3D/dane/gitara2.bin",						// 1
		"d:/Projekty/LiveScan3D/dane/gitara3.bin",						// 2
		"d:/Projekty/LiveScan3D/dane/gitara_tlo.bin",					// 3
		"d:/Projekty/LiveScan3D/dane/gitara_marek.bin",					// 4
		"d:/Projekty/LiveScan3D/dane/gitara_marek2.bin",				// 5
		"d:/Projekty/LiveScan3D/dane/gitara_marek3.bin",				// 6
		"d:/Projekty/LiveScan3D/dane/pokoj1.bin",						// 7
		"d:/Projekty/LiveScan3D/dane/pokoj2.bin",						// 8
		"d:/Projekty/LiveScan3D/dane/marekpokoj.bin",					// 9
		"d:/Projekty/LiveScan3D/dane/marekpokoj2.bin",					// 10
		"d:/Projekty/LiveScan3D/dane/marek_gitara3.bin",				// 11
		"d:/Projekty/LiveScan3D/dane/frames_info_3_na_gorze.bin",		// 12
		"d:/Projekty/LiveScan3D/dane/frames_info_face.bin",				// 13
		"d:/Projekty/LiveScan3D/dane/frames_info_znacznik_3_bez_filtra.bin", // 14
		"d:/Projekty/LiveScan3D/dane/frames_info_gitara_3_bez_filtra_dobre!.bin", // 15
		"d:/Projekty/LiveScan3D/dane/frames_info_gitara_3_kinecty_ez_filtra.bin", // 16
		"d:/Projekty/LiveScan3D/dane/frames_info_frames_info_gitara_bez filtra_jeden_k.bin.bin", // 17
	};

	loadAllFramesInformation(filenames[6], n_maps, &depth_maps, &depth_colors, &widths, &heights, &intr_params, &wtransform_params);

#endif
	//memset(depth_maps + widths[0] * heights[0] * 2, 0, 4 * widths[0] * heights[0]);


#ifdef SHOW_TIMINGS
	SimpleTimer timer;
#endif

#ifdef FILTER_FLYING_PIXELS

	for (int i = 0; i<n_maps; i++)
		filterFlyingPixels(1, 20, 4, widths[0], heights[0], (unsigned short*)(depth_maps + i * widths[0] * heights[0] * 2));

#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("Flying pixels: ");
#endif

#endif

	vector<VerticesWithDepthColorMaps> vertices_with_maps(n_maps);
	vector<IntrinsicCameraParameters> intrinsic_params(n_maps);
	vector<WorldTransformation> world_transforms(n_maps);
	vector<ColorCorrectionParams> color_correction_coeffs;
	vector<vector<TriangleIndexes>> triangle_indexes(n_maps + 1);  // additional map for the stiches

	for (int i = 0; i < n_maps; i++)
	{
		intrinsic_params[i] = IntrinsicCameraParameters(intr_params + i * 7);
		world_transforms[i] = WorldTransformation(wtransform_params + i*(9 + 3));
		world_transforms[i].inv();
	}

	// DELETE ME!!!
	//for (int i = 0; i < n_maps; i++)
	//	if (i != 0)
	//		memset(depth_maps + i * widths[0] * heights[0] * 2, 0, widths[0] * heights[0] * 2);
	// 0----------------0

#ifdef LOAD_REFINED_ICP
	FILE *f2 = fopen("wt.txt", "rt");
	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	{
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				fscanf(f2, "%f", &world_transforms[current_map_index].R[i][j]);

		for (int i = 0; i < 3; i++)
			fscanf(f2, "%f", &world_transforms[current_map_index].t[i]);
	}
	fclose(f2);
#endif

	generateVerticesFromDepthMaps(depth_maps, depth_colors, widths, heights, world_transforms, intrinsic_params, vertices_with_maps,
		minX, minY, minZ, maxX, maxY, maxZ);

#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("Generating vertices: ");
#endif


#ifdef PERFORM_ICP
	printf("Performing ICP\n");

	performICP(vertices_with_maps, world_transforms, 3);
	FILE *f = fopen("wt.txt", "wt");
#endif
	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	{
		int w = widths[current_map_index];
		int h = heights[current_map_index];
		WorldTransformation world_transform = world_transforms[current_map_index];

#ifdef PERFORM_ICP
		for (int i = 0; i < 3; i++)
			for (int j = 0; j < 3; j++)
				fprintf(f, "%f ", world_transform.R[i][j]);
		for (int i = 0; i < 3; i++)
			fprintf(f, "%f ", world_transform.t[i]);
#endif
		projectVerticesIntoDepthMap(vertices_with_maps[current_map_index], world_transform, intrinsic_params[current_map_index], w, h, false);
	}

	vector<VerticesWithDepthColorMaps> original_vertices_with_maps = vertices_with_maps;

#ifdef PERFORM_ICP
	fclose(f);

	generateVerticesFromDepthMaps(depth_maps, depth_colors, widths, heights, world_transforms, intrinsic_params, vertices_with_maps,
		minX, minY, minZ, maxX, maxY, maxZ);

#endif

#ifdef LOAD_REFINED_ICP
	//	generateVerticesFromDepthMaps(depth_maps, depth_colors, widths, heights, world_transforms, intrinsic_params, vertices_with_maps,
	//		minX, minY, minZ, maxX, maxY, maxZ);

#endif

#ifdef DEBUG_IMAGES
	char tmp[1024];
	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	{
		sprintf(tmp, "test/depth_xxx_before_convex_hulls_%d_1.pgm", current_map_index);
		writeDepthImage(vertices_with_maps[current_map_index].depth_map, widths[current_map_index], heights[current_map_index], tmp);
	}
#endif

#ifdef SHOW_TIMINGS
	timer.start();
#endif

	if (bcolor_transfer || bgenerate_triangles)
	{
		generateVerticesConfidence(vertices_with_maps, widths, heights);
	}
#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("generateVerticesConfidence");
#endif

#ifdef CONVEX_HULL
	clearAllMapsConvexHull(vertices_with_maps, world_transforms, intrinsic_params, widths, heights);
	for (int i = 0; i < vertices_with_maps.size(); i++)
		memcpy(depth_maps + i * widths[0] * heights[0] * 2, vertices_with_maps[i].depth_map.data(), widths[0] * heights[0] * 2);
#endif
#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("Convex hull: ");
#endif

	generateVerticesFromDepthMaps(depth_maps, depth_colors, widths, heights, world_transforms, intrinsic_params, vertices_with_maps,
		minX, minY, minZ, maxX, maxY, maxZ);


#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("generateVerticesFromDepthMaps");
#endif


#ifdef DEBUG_IMAGES
	for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
	{
		sprintf(tmp, "test/depth_xxx_after_convex_hulls_%d_1.pgm", current_map_index);
		writeDepthImage(vertices_with_maps[current_map_index].depth_map, widths[current_map_index], heights[current_map_index], tmp);
	}
#endif


	if (bcolor_transfer)
	{
		updateColorCorrectionCoefficients(vertices_with_maps, widths, heights, world_transforms, intrinsic_params, color_correction_coeffs);
		for (size_t i = 0; i < color_correction_coeffs.size(); i++)
		{
			int target_map = color_correction_coeffs[i].map_index;
			applyColorCorrection(vertices_with_maps[target_map].colors, color_correction_coeffs[i]);
			applyColorCorrection(vertices_with_maps[target_map].colors_map, color_correction_coeffs[i]);
			applyColorCorrection(original_vertices_with_maps[target_map].colors, color_correction_coeffs[i]);
			applyColorCorrection(original_vertices_with_maps[target_map].colors_map, color_correction_coeffs[i]);

		}
	}

#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("Color correction");
#endif

	if (bgenerate_triangles)
	{
		mergeVerticesForViews(vertices_with_maps, widths, heights, world_transforms, intrinsic_params);

		//generateTriangles(vertices_with_maps, heights, widths, triangle_indexes);
		//mergeTrianglesForViews(vertices_with_maps, widths, heights, triangle_indexes, world_transforms, intrinsic_params); 
#ifdef SHOW_TIMINGS
		timer.printLapTimeAndRestart("mergeVerticesForViews");
#endif

#ifdef DEBUG_IMAGES
		char tmp[1024];
		for (int current_map_index = 0; current_map_index < n_maps; current_map_index++)
		{
			sprintf(tmp, "test/depth_to_triangulation_%d_1.pgm", current_map_index);
			writeDepthImage(vertices_with_maps[current_map_index].depth_map, widths[current_map_index], heights[current_map_index], tmp);
		}

#endif
		generateTriangles(vertices_with_maps, heights, widths, triangle_indexes, world_transforms, intrinsic_params);
	}

	reprojectionCorrection(vertices_with_maps, original_vertices_with_maps, heights, widths, triangle_indexes, world_transforms, intrinsic_params);

#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("generateTrianglesGradients");
#endif

	formMesh(out_mesh, vertices_with_maps, triangle_indexes);
#ifdef SHOW_TIMINGS
	timer.printLapTimeAndRestart("formMesh");
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

