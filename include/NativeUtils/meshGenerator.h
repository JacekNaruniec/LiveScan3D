#pragma once

#include <vector>
#include "mesh.h"
#include "cameraparameters.h"
#include "point3f.h"
#include "colorcorrection.h"
#include "meshgeneratordata.h"

typedef std::vector<std::vector<float>> rot_matrix;

struct Connection
{
	int point1_index;
	int point2_index;
	unsigned short depth1, depth2;
};

struct MapCorrespondence
{
	std::vector<int> x;
	std::vector<int> y;
	std::vector<unsigned short> org_d_mapped;
	std::vector<unsigned short> dst_d;
	int n_mapped_pixels; 
};

class MeshGenerator
{
public:
	MeshGenerator();

	
	void setBounds(float minX, float maxX, float minY, float maxY, float minZ, float maxZ);
	void generateMeshFromDepthMaps(int n_maps, unsigned char* depth_maps, unsigned char *depth_colors, int *widths, int *heights,
		float *intr_params, float *extr_params, Mesh *out_mesh, bool bcolor_transfer, bool bgenerate_triangles, bool data_cleaning = true);
	
	int getNTrianglesPassingConditions(unsigned short  *initialPos, int w);
	void formMeshChunks(Mesh &mesh, MeshChunks &mesh_chunks);

private:
	void formVerticesChunksOnly(Mesh &mesh, MeshChunks &mesh_chunks);

	bool checkTriangleConstraints(unsigned short  *depth_ptr1, unsigned short  *depth_ptr2, unsigned short  *depth_ptr3);
	void generateTrianglesGradientsRegion(unsigned short  *depth_image, std::vector<TriangleIndexes> *indexes, int ndepth_frame_width, int ndepth_frame_height,
		int minX, int minY, int maxX, int maxY, int *out_n_triangles);
	void initializeVariables(int n_maps, unsigned char* depth_maps, unsigned char *color_maps, float *intr_params, float *extr_params,
		int *widths, int *heights);
	bool findEdgeConnection(int x, int y, int first_depth_map_index, int sec_depth_map_index,
		unsigned char *sec_edge_map, Connection &out_c);

	static unsigned short getDepthThreshold(unsigned short depth);
	void pointProjection(const Point3f p, int &x, int &y, unsigned short &d, ExtrinsicCameraParameters &ep, IntrinsicCameraParameters &ip);


	virtual void clearAllMapsByOverlapAndFOV();
	void updateColorCorrectionCoefficients();
	void generateTriangles(Mesh *mesh);
	void generateVerticesOnly(Mesh *mesh);

	void reprojectionCorrection(Mesh *mesh);
	void formMesh(Mesh *mesh, std::vector<std::vector<TriangleIndexes>> &triangle_indexes);
	void generateDepthMapsConfidence();
	void generateMapConfidence(std::vector<unsigned short> &depth_map, std::vector<unsigned char> &confidence_map, int w, int h, int et_limit, int depth_threshold);
	void setVertex(VertexC4ubV3f &v, int x, int y, unsigned short depth, ExtrinsicCameraParameters &ep, IntrinsicCameraParameters &ip);

	void applyAllColorCorrections();

	void generateAllMapsCorrespondences();
	void generateCorrespondencesForMap(int base_map);

	void limitDepthMapsToBounds();
	void limitDepthMapToBounds(int map_index);
	rot_matrix multiply3x3Matrices(rot_matrix &M1, rot_matrix &M2);
	ColorCorrectionParams getColorCorrectionTransform(int index_1, int index_2);
	bool checkDepthPointConstraints(int current_map_index, int x, int y);
	void generateTrianglesForStiches(std::vector<TriangleIndexes> &triangles);	

	void createTrianglesForSegment(int seed_x, int seed_y, int w, int h, unsigned short *first_depth_map, unsigned short *second_depth_map,
		unsigned char *first_edge_map, unsigned char *second_edge_map, Connection &c_1, std::vector<unsigned char> &points_visited, int depth_map_index1, int depth_map_index2,
		TriangleIndexes *indexes, int &n_triangles);
	
	void setVerticesThreadFunction(int min_vertex, int max_vertex,
		const std::vector<int> &vertex_depth_map_pos, const std::vector<int> &vertex_depth_map_index,
		std::vector<int> dm_widths, std::vector<int> dm_heights, VertexC4ubV3f *vertices,
		std::vector<IntrinsicCameraParameters> ip, std::vector<ExtrinsicCameraParameters> ep_inv);


	std::vector<float> bounds; 
	std::vector<std::vector<unsigned short>> current_depth_maps;
	std::vector<std::vector<RGB>> current_color_maps;
	std::vector<std::vector<unsigned char>> depth_map_confidence;

	std::vector<int> dm_widths; 
	std::vector<int> dm_heights;

	std::vector<IntrinsicCameraParameters> intrinsic_params;
	std::vector<ExtrinsicCameraParameters> extrinsic_params;
	std::vector<std::vector<MapCorrespondence>> map_correspondences; 
	std::vector<ColorCorrectionParams> color_correction_params;

	MeshGeneratorData data; 

	template<class T>
	bool resizeIfNeeded(size_t n_elem_after_resize, size_t n_elem_min_size, std::vector<T> &var, bool forceExactSize = false)
	{
			if ((!forceExactSize && var.size() < n_elem_min_size) || (forceExactSize && var.size() != n_elem_min_size))
			{
				var.resize(n_elem_after_resize);
				return true;
			}

		return false;
	}
};