#pragma once

struct VertexC4ubV3f
{
	unsigned char R, G, B, A;
	float X, Y, Z;
};

struct Mesh
{
	int nVertices;
	VertexC4ubV3f *vertices;
	int nTriangles;
	int *triangles;
};