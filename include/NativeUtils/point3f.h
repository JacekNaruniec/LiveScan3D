#pragma once

#include <vector>

struct Point3f
{
	float X, Y, Z;

	void rotate(std::vector<std::vector<float>> &R)
	{
		Point3f res; 

		res.X = X * R[0][0] + Y * R[0][1] + Z * R[0][2];
		res.Y = X * R[1][0] + Y * R[1][1] + Z * R[1][2];
		res.Z = X * R[2][0] + Y * R[2][1] + Z * R[2][2];

		X = res.X;
		Y = res.Y;
		Z = res.Z;
	}
};