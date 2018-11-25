#pragma once

#include <vector>

struct IntrinsicCameraParameters
{
	float cx, cy;        // principal points
	float fx, fy;        // focal lengths
	float r2, r4, r6;    // camera radial distortion parameters (second, fourth and sixth order)
	IntrinsicCameraParameters() {}
	IntrinsicCameraParameters(float *p) :
		cx(p[0]), cy(p[1]), fx(p[2]), fy(p[3]), r2(p[4]), r4(p[5]), r6(p[6]) {}
};


struct ExtrinsicCameraParameters
{
	std::vector<float> t;
	std::vector<std::vector<float>> R;

	ExtrinsicCameraParameters() {}
	ExtrinsicCameraParameters(float *p)
	{
		t.resize(3);
		memcpy(t.data(), p, 3 * sizeof(float));
		R = std::vector<std::vector<float>>(3, std::vector<float>(3));
		for (int i = 0; i < 3; i++)
			memcpy(R[i].data(), p + 3 + 3 * i, 3 * sizeof(float));
	}

	void toFloatP(float *p)
	{
		memcpy(p, t.data(), 3 * sizeof(float));
		for (int i = 0; i < 3; i++)
			memcpy(p + 3 + 3 * i, R[i].data(), 3 * sizeof(float));
	}

	void inv()
	{
		std::vector<std::vector<float>> Rt(3, std::vector<float>(3));
		for (int i = 0; i < 3; i++)
		{
			t[i] = -t[i];
			for (int j = 0; j < 3; j++)
				Rt[i][j] = R[j][i];
		}
		R = Rt;
	}
};
