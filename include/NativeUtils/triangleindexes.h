#pragma once

struct TriangleIndexes
{
	int ind[3];
	int map_ind[3];
	void set(const int ind1, const int ind2, const int ind3,
		const int map_ind1, const int map_ind2, const int map_ind3)
	{
		ind[0] = ind1; ind[1] = ind2; ind[2] = ind3;
		map_ind[0] = map_ind1; map_ind[1] = map_ind2; map_ind[2] = map_ind3;
	}

	TriangleIndexes(int i1, int i2, int i3) :
		ind{ i1, i2, i3 } { }

	TriangleIndexes(int i1, int i2, int i3, int m1, int m2, int m3) :
		ind{ i1, i2, i3 }, map_ind{ m1, m2, m3 } { }

	TriangleIndexes() {}
};
