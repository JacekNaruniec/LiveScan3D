#pragma once

// code based on https://github.com/anlcnydn/bilateral

#include <vector>

class BilateralFilter
{
public:
	BilateralFilter(int diameter = 5, double sigmaI = 12, double sigmaS = 16);

	template<class T>
	void bilateralFilter(T *image, int width, int height)
	{
		if (width == 0 || height == 0)
			return;
		std::vector<T> filtered(width * height, 0);

		int half_diameter = diameter / 2;

		for (int i = half_diameter; i < height - half_diameter; i++) {
			for (int j = half_diameter; j < width - half_diameter; j++) {
				filtered[j + i * width] = bilateralFilterPixel(image, j, i, width);
			}
		}

		memcpy(image, filtered.data(), width*height * sizeof(filtered[0]));
	}


private:
	float distance(int x, int y, int i, int j);
	double gaussian(double x, double sigma);

	template<class T>
	T bilateralFilterPixel(T *source, int x, int y, int width)
	{
		double iFiltered = 0;
		double wP = 0;
		int neighbor_x = 0;
		int neighbor_y = 0;
		int half = diameter / 2;
		T val = source[x + y * width];
		double sigmaIsq = sigmaI * sigmaI;
		int ind = 0;

		for (int i = 0; i < diameter; i++) {
			for (int j = 0; j < diameter; j++) {
				neighbor_x = x - (half - i);
				neighbor_y = y - (half - j);
				T neigh_val = source[neighbor_y * width + neighbor_x];

				double gi = gaussILUT[abs(val - neigh_val)];
				double w = gi * coeffsS[ind];
				iFiltered = iFiltered + neigh_val * w;
				ind++;
				wP = wP + w;
			}
		}
		iFiltered = iFiltered / wP;
		return static_cast<T>(iFiltered);
	}

	std::vector<double> coeffsS;
	std::vector<double> gaussILUT;
	int diameter = 5;
	double sigmaI = 12, sigmaS = 16;

};
