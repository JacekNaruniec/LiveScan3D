#include "depthmaputils.h"

#pragma once

#include <vector>
#include <string>
#include "simpleimage.h"
#include "hsvrgb.h"

void writeDepthImageHSV(std::vector<unsigned short> &depth_image, int w, int h, std::string filename)
{
	SimpleImage im;
	im.create(w, h, 3, nullptr);
	unsigned short min_val = 65535;
	unsigned short max_val = 0;

	for (int i = 0; i < w; i++)
		for (int j = 0; j < h; j++)
		{
			if (depth_image[j*w + i] == 0)
				continue;
			max_val = max(max_val, depth_image[j*w + i]);
			min_val = min(min_val, depth_image[j*w + i]);
		}

	if (max_val - min_val != 0)
		for (int i = 0; i < w; i++)
			for (int j = 0; j < h; j++)
			{
				if (depth_image[j * w + i] == 0)
					continue;
				float val = 1.0f - ((depth_image[j * w + i] - min_val) / (float)(max_val - min_val));
				float R, G, B, H = val * 360, S = 1.0f, V = 1.0f;
				HSVtoRGB(R, G, B, H, S, V);
				im[j][i * 3] = (unsigned char)(R * 255.0f);
				im[j][i * 3 + 1] = (unsigned char)(G * 255.0f);
				im[j][i * 3 + 2] = (unsigned char)(B * 255.0f);
			}

	im.writeToFile(filename.c_str());
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


void getEdges(int w, int h, unsigned short *depth_map, int &n_edges, vector<unsigned char> &current_edges)
{
	n_edges = 0;
	std::fill(current_edges.begin(), current_edges.end(), 0);
	const int shifts[] = { -w - 1, -w, -w + 1, 1, w + 1, w, w - 1, -1 };

	for (int y = 1; y<h - 1; y++)
		for (int x = 1; x < w - 1; x++)
		{
			if (edgePixel(x, y, w, h, depth_map))
			{
				current_edges[x + y * w] = 255;
				n_edges++;
				continue;
			} 
		}

}

