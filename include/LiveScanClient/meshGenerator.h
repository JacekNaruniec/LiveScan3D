#pragma once

struct Triangle
{
	float X, Y, Z;
	float R, G, B;
};

class MeshGenerator
{
public:
	MeshGenerator();

	void generateMeshFromDepthImage(short *depth_image);
};