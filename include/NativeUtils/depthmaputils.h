#pragma once

#include <vector>
#include <string>

void writeDepthImageHSV(std::vector<unsigned short> &depth_image, int w, int h, std::string filename);
inline bool edgePixel(int x, int y, int w, int h, unsigned short *data);
void getEdges(int w, int h, unsigned short *depth_map, int &n_edges, std::vector<unsigned char> &out_edges);

template <class T>
float getSubpixelValue(std::vector<T> &map, float x, float y, int w, int h)
{
	float x0, y0;
	float x_r, y_r;
	float val = 0.0f;
	float diff_x, diff_y;

	if (x<0.0 || y<0.0 || x>(w - 2) || y>(h - 2)) return 0.0f;

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

