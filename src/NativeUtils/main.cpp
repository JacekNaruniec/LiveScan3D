//   Copyright (C) 2015  Marek Kowalski (M.Kowalski@ire.pw.edu.pl), Jacek Naruniec (J.Naruniec@ire.pw.edu.pl)
//   License: MIT Software License   See LICENSE.txt for the full license.

//   If you use this software in your research, then please use the following citation:

//    Kowalski, M.; Naruniec, J.; Daniluk, M.: "LiveScan3D: A Fast and Inexpensive 3D Data
//    Acquisition System for Multiple Kinect v2 Sensors". in 3D Vision (3DV), 2015 International Conference on, Lyon, France, 2015

//    @INPROCEEDINGS{Kowalski15,
//        author={Kowalski, M. and Naruniec, J. and Daniluk, M.},
//        booktitle={3D Vision (3DV), 2015 International Conference on},
//        title={LiveScan3D: A Fast and Inexpensive 3D Data Acquisition System for Multiple Kinect v2 Sensors},
//        year={2015},
//    }
#include <stdio.h>
#include <vector>
#include "simpletimer.h"

//#include "icp.h"
//#include "opencv\cv.h"
#include "depthprocessing.h"
#include "bilateralFilter.h"

using namespace std;

//#define PERFORM_ICP

struct Triangle
{
	int v1, v2, v3;
};

void savePLY(std::string filename, std::vector<Point3f> &vertices, std::vector<RGB> &colors, std::vector<Triangle> &triangles)
{

	unsigned int numVertices = (unsigned int)vertices.size();
	unsigned int numColors = (unsigned int)colors.size();

	// Open File
	FILE *meshFile = NULL;
	errno_t err = fopen_s(&meshFile, filename.c_str(), "wt");


	// Write the header line
	std::string header = "ply\nformat ascii 1.0\n";
	fwrite(header.c_str(), sizeof(char), header.length(), meshFile);

	const unsigned int bufSize = 1000 * 3;
	char outStr[bufSize];
	int written = 0;

	// Elements are: x,y,z, r,g,b
	written = sprintf_s(outStr, bufSize, "element vertex %u\nproperty float x\nproperty float y\nproperty float z\nproperty uchar red\nproperty uchar green\nproperty uchar blue\n", numVertices);
	fwrite(outStr, sizeof(char), written, meshFile);

	if (triangles.size() > 0)
	{
		written = sprintf_s(outStr, bufSize, "element face %d\nproperty list uchar int vertex_index\n", (int)triangles.size());
		fwrite(outStr, sizeof(char), written, meshFile);
	}
	written = sprintf_s(outStr, bufSize, "end_header\n");
	fwrite(outStr, sizeof(char), written, meshFile);


	for (unsigned int vertexIndex = 0; vertexIndex < numVertices; vertexIndex++)
	{
		unsigned int color0 = colors[vertexIndex].R;
		unsigned int color1 = colors[vertexIndex].G;
		unsigned int color2 = colors[vertexIndex].B;

		written = sprintf_s(outStr, bufSize, "%f %f %f %u %u %u\n",
			vertices[vertexIndex].X, vertices[vertexIndex].Y, vertices[vertexIndex].Z,
			((color0)& 255), ((color1)& 255), (color2 & 255));

		fwrite(outStr, sizeof(char), written, meshFile);
	}

	for (unsigned int triangleIndex = 0; triangleIndex < triangles.size(); triangleIndex++)
	{
		written = sprintf_s(outStr, bufSize, "3 %d %d %d\n", triangles[triangleIndex].v1, 
			triangles[triangleIndex].v2, triangles[triangleIndex].v3);

		fwrite(outStr, sizeof(char), written, meshFile);

	}

	fflush(meshFile);
	fclose(meshFile);
}

void loadPLY(string filename, vector<Point3f> &verts, vector<RGB> &colors)
{
	FILE *f;
	int nVerts;

	fopen_s(&f, filename.c_str(), "r");

	char buffer[100];
	fgets(buffer, 100, f);
	fgets(buffer, 100, f);
	fgets(buffer, 100, f);
	sscanf_s(buffer, "element vertex %d\n", &nVerts);

	for (int i = 0; i < 7; i++)
		fgets(buffer, 100, f);

	for (int i = 0; i < nVerts; i++)
	{
		fgets(buffer, 100, f);

		Point3f point;
		RGB rgb;
		int R, G, B;
		sscanf_s(buffer, "%f %f %f %d %d %d\n", &point.X, &point.Y, &point.Z, &R, &G, &B);
		
		rgb.R = R;
		rgb.G = G;
		rgb.B = B;

		verts.push_back(point);
		colors.push_back(rgb);
	}
	fclose(f);
}

void savePLY(std::string filename, Mesh &mesh)
{
	vector<Point3f> vertices(mesh.nVertices);
	vector<RGB> colors(mesh.nVertices);
	vector<Triangle> triangles(mesh.nTriangles);

	for (int vertexIndex = 0; vertexIndex < mesh.nVertices; vertexIndex++)
	{
		vertices[vertexIndex].X = mesh.vertices[vertexIndex].X;
		vertices[vertexIndex].Y = mesh.vertices[vertexIndex].Y;
		vertices[vertexIndex].Z = mesh.vertices[vertexIndex].Z;

		colors[vertexIndex].R = mesh.vertices[vertexIndex].R;
		colors[vertexIndex].G = mesh.vertices[vertexIndex].G;
		colors[vertexIndex].B = mesh.vertices[vertexIndex].B;
	}

	for (int triangleIndex = 0; triangleIndex < mesh.nTriangles; triangleIndex++)
	{
		triangles[triangleIndex].v1 = mesh.triangles[triangleIndex * 3 + 0];
		triangles[triangleIndex].v2 = mesh.triangles[triangleIndex * 3 + 1];
		triangles[triangleIndex].v3 = mesh.triangles[triangleIndex * 3 + 2];
	}

	savePLY(filename, vertices, colors, triangles);


}
/*
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
*/

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
		unsigned short *rowPtr = depth_map + rowPos;
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

vector<Point3f> createVertices(unsigned char* depth_map, unsigned char *depth_colors, int width, int height,
	float *intr_params, float *extr_params)
{
	MeshGenerator mesh_generator;
	Mesh out_mesh;

	mesh_generator.setBounds(-1.0f, 1.0f, -1.0, 1.0f, -0.5f, 1.0f);

	mesh_generator.generateMeshFromDepthMaps(1, depth_map, depth_colors, &width, &height, intr_params,
		extr_params, &out_mesh, false, false, true);

	vector<Point3f> points(out_mesh.nVertices);
	for (int i = 0; i < out_mesh.nVertices; i++)
	{
		points[i].X = out_mesh.vertices[i].X;
		points[i].Y = out_mesh.vertices[i].Y;
		points[i].Z = out_mesh.vertices[i].Z;
	}
	return points; 
}

void performICP(int n_maps, unsigned char* depth_maps, unsigned char *depth_colors, int *widths, int *heights,
	float *intr_params, float *_extr_params, int refine_iters)
{
	vector<vector<float>> Rs(n_maps, vector<float>(9, 0.0f));
	vector<vector<float>> ts(n_maps, vector<float>(3, 0.0f));

	vector<ExtrinsicCameraParameters> ep(n_maps);
	for (int i = 0; i < n_maps; i++)
	{
		ep[i] = ExtrinsicCameraParameters(_extr_params + i * 12);
	}

	for (int i = 0; i<n_maps; i++)
		for (int j = 0; j < 3; j++)
		{
			Rs[i][j * 3 + j] = 1;
			ts[i][j] = 0;
		}

	vector<vector<Point3f>> points(n_maps);

	int pos = 0;
	for (int map = 0; map < n_maps; map++)
	{
		points[map] = createVertices(depth_maps + pos * 2,
			depth_colors + pos * 3, widths[map], heights[map],
			intr_params + map * 7, _extr_params + map * 12);
		pos += widths[map] * heights[map];
	}
	

	for (int refineIter = 0; refineIter < refine_iters; refineIter++)
	{
		float sum_err = 0.0f;
		for (int i = 0; i < n_maps; i++)
		{
			vector<Point3f> points_rest;

			for (int j = 0; j < n_maps; j++)
			{
				if (j == i)
					continue;

				points_rest.insert(points_rest.end(), points[j].begin(), points[j].end());
			}
			sum_err += ICP(points_rest.data(),
				points[i].data(),
				(int)points_rest.size(), (int)points[i].size(),
				Rs[i].data(), ts[i].data());
		}
		printf("err = %f ", sum_err);
	/*	for (int j = 0; j < n_maps; j++)
		{
			printf("map = %d\nRs: ", j);
			for (int i = 0; i < 9; i++)
				printf("%f ", Rs[j][i]);
			printf("\nts: ");
			for (int i = 0; i < 3; i++)
				printf("%f ", ts[j][i]);
			printf("\n");
		}
		printf("\n");
		//getchar();*/
	}
	printf("\n");
	//Update the calibration data 
	for (int i = 0; i < n_maps; i++)
	{
		vector<float> tempT(3, 0.0f);
		vector<vector<float>> tempR(3, vector<float>(3, 0.0f));
		vector<vector<float>> &out_R = ep[i].R; 
		vector<float> &out_ts = ep[i].t;

		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				tempT[j] += ts[i][k] * out_R[k][j];

		for (int j = 0; j < 3; j++)
			out_ts[j] += tempT[j];


		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				for (int l = 0; l < 3; l++)
					tempR[j][k] += Rs[i][l * 3 + j] * out_R[l][k];
			

		for (int j = 0; j < 3; j++)
			for (int k = 0; k < 3; k++)
				out_R[j][k] = tempR[j][k];
	}

	for (int i = 0; i < n_maps; i++)
	{
		ep[i].toFloatP(_extr_params + i * 12);
	}
}



//This function here can be used to test the ICP functionality, it aligns the points clouds in "test1.ply" and "test2.ply"
int main()
{
	Mesh mesh; 
	unsigned char *depth_maps = nullptr, *depth_colors = nullptr;
	int *widths = nullptr, *heights = nullptr;
	float *intr_params = nullptr, *wtransform_params = nullptr;

	SimpleTimer timer; 
	BilateralFilter filter; 

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
		"d:/Projekty/LiveScan3D/dane/frames_info_gitara_bez filtra_jeden_k.bin", // 17
	};
	MeshGenerator mesh_generator;
	MeshChunks mesh_chunks; 
	timer.start();
	int total_ms = 0, n_total = 0;
	long long n_total_vertices = 0;
	for (int i = 0; i <18; i++)
	{
#ifndef PERFORM_ICP
		filenames[i] = string(filenames[i].begin(), filenames[i].end() - 4);
		filenames[i].append("_icp.bin");
#endif

		printf("%s\n", filenames[i].c_str()); 
		int n_maps;
		loadAllFramesInformation(filenames[i], n_maps, &depth_maps, &depth_colors,
			&widths, &heights, &intr_params, &wtransform_params);


		for (int i = 0; i < n_maps; i++)
		{
			filterFlyingPixels(1, 20, 4, widths[0], heights[0], (unsigned short*)(depth_maps + i * widths[0] * heights[0] * 2));
			filter.bilateralFilter((unsigned short*)(depth_maps + i * widths[0] * heights[0] * 2), widths[i], heights[i]);
		}


#ifdef PERFORM_ICP
		printf("Performing ICP\n");
		performICP(n_maps, depth_maps, depth_colors, widths, heights, intr_params, wtransform_params,
			15);
		printf("ICP done \n");

		string out_name = string(filenames[i].begin(), filenames[i].end() - 4);
		out_name.append("_icp.bin");
		storeAllFramesInformation(out_name, n_maps, depth_maps, depth_colors, widths, heights, 
			intr_params, wtransform_params);
#endif


		//mesh_generator.setBounds(-1.0f, 1.0f, -1.0, 1.0f, -0.5f, 1.0f);
		mesh_generator.setBounds(-100.0f, 100.0f, -100.0, 100.0f, -100.0f, 100.0f);

		//if (i==0)
		//	mesh_generator.generateMeshFromDepthMaps(n_maps, depth_maps, depth_colors,
		//		widths, heights, intr_params, wtransform_params, &mesh, true, true);
		//          

		timer.start(); 
		for (int j = 0; j < 1; j++)
		{
			n_total++;
			mesh_generator.generateMeshFromDepthMaps(n_maps, depth_maps, depth_colors,
				widths, heights, intr_params, wtransform_params, &mesh, true, true);

			mesh_generator.formMeshChunks(mesh, mesh_chunks);
			n_total_vertices += mesh.nVertices;
			//printf("%s\n", filenames[i].c_str());
		}
		timer.stop();
		total_ms += (int)timer.getMilliseconds();
		//generateMeshFromDepthMaps(3, depth_maps, depth_colors,
		//widths, heights, intr_params, wtransform_params, &mesh, true, -1.0f, -1.0f,
		//	-0.5f, 1.0f, 1.0f, 1.0f, true, filenames[i]);
		//generateMeshFromDepthMaps(1, depth_maps, depth_colors,
		//	widths, heights, intr_params, wtransform_params, &mesh, true, -500.0f, -500.0f,
		//	-500.0f, 500.0f, 500.0f, 500.0f, true, filenames[i]);
		
		char tmp[1024];
		sprintf(tmp, "%s.ply", filenames[i].c_str());
//		printf("\nSaving mesh to PLY file\n");
		//savePLY(tmp, mesh);
//		printf("PLY saved\n");
		

	}
	timer.stop();
	//int ms = (int)timer.getMilliseconds();
	printf("\nProcessing took %d [ms] ", total_ms);
	printf("\nMean time %d [ms] ", total_ms / n_total);
	printf("\nMean n_vertices %d [ms] ", n_total_vertices / n_total);
	getchar();

	return 0;

	/* create reference data */
	/*FILE *ref = fopen("ref.bin", "wb");
	fwrite(&mesh.nTriangles, sizeof(mesh.nTriangles), 1, ref);
	fwrite(mesh.triangles, sizeof(mesh.triangles[0]), mesh.nTriangles, ref);
	fwrite(&mesh.nVertices, sizeof(mesh.nVertices), 1, ref);
	fwrite(mesh.vertices, sizeof(mesh.vertices[0]), mesh.nVertices, ref);
	fclose(ref);*/
	
	printf("\n Created %d vertices", mesh.nVertices);
	printf("\n Created %d triangles", mesh.nTriangles);


	getchar();
	return 0;

	// compare to reference data
	//FILE *ref_test = fopen("ref_face.bin", "rb");
	FILE *ref_test = fopen("ref_na_gorze.bin", "rb");
	Mesh ref_mesh; 
	fread(&ref_mesh.nTriangles, sizeof(ref_mesh.nTriangles), 1, ref_test);
	ref_mesh.triangles = new int[ref_mesh.nTriangles];
	fread(ref_mesh.triangles, sizeof(ref_mesh.triangles[0]), ref_mesh.nTriangles, ref_test);

	fread(&ref_mesh.nVertices, sizeof(ref_mesh.nVertices), 1, ref_test);
	ref_mesh.vertices = new VertexC4ubV3f[ref_mesh.nVertices];
	fread(ref_mesh.vertices, sizeof(ref_mesh.vertices[0]), ref_mesh.nVertices, ref_test);
	fclose(ref_test);

	printf("\n\n Reference %d vertices", ref_mesh.nVertices);
	printf("\n Reference %d triangles", ref_mesh.nTriangles);



	if (ref_mesh.nVertices != mesh.nVertices)
	{
		printf("\nNumbers of vertices are not equal!");
		getchar(); 
		return 0;
	}

	if (ref_mesh.nTriangles != mesh.nTriangles)
	{
		printf("\nNumbers of triangles are not equal!");
		getchar();
		return 0;
	}

	for (int i=0; i<ref_mesh.nTriangles; i++)
		if (ref_mesh.triangles[i] != mesh.triangles[i])
		{
			printf("\n Different triangle %d!! ", i);
			getchar();
			return 0;
		}

	for (int i = 0; i<ref_mesh.nVertices; i++)
		if (ref_mesh.vertices[i].X != mesh.vertices[i].X ||
			ref_mesh.vertices[i].Y != mesh.vertices[i].Y ||
			ref_mesh.vertices[i].Z != mesh.vertices[i].Z ||
			ref_mesh.vertices[i].R != mesh.vertices[i].R ||
			ref_mesh.vertices[i].G != mesh.vertices[i].G ||
			ref_mesh.vertices[i].B != mesh.vertices[i].B ||
			ref_mesh.vertices[i].A != mesh.vertices[i].A)
		{
			printf("\n Different vertex %d!! ", i);
			getchar();
			return 0;
		}
	
	
	printf("\nTest PASSED");
	
	getchar();  
	
	return 0;
	/*
	vector<Point3f> verts1, verts2;
	vector<RGB> colors1, colors2;
	loadPLY("../test1.ply", verts1, colors1);
	loadPLY("../test2.ply", verts2, colors2);
	
	cv::Mat R = cv::Mat::eye(3, 3, CV_32F);
	cv::Mat t(1, 3, CV_32F, cv::Scalar(0));
	cv::Mat verts2Mat((int)verts2.size(), 3, CV_32F, (float*)verts2.data());
	
	ICP(verts1.data(), verts2.data(), (int)verts1.size(), (int)verts2.size(), (float*)R.data, (float*)t.data, 1);
	savePLY("../testResult.ply", verts2, colors2);
	return 0;
	*/
}
