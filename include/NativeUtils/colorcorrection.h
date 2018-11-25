#pragma once

#include <vector>

enum ColorSpace { CS_RGB, CS_LALPHABETA, CS_YUV };

struct RGB
{
	unsigned char R, G, B;
};

struct ColorCorrectionParams
{
	double mean_a1 = 0.0, mean_b1 = 0.0, mean_c1 = 0.0;
	double mean_a2 = 0.0, mean_b2 = 0.0, mean_c2 = 0.0;
	double std_scale_a = 1.0, std_scale_b = 1.0, std_scale_c = 1.0;
	int base_map_index = 0, map_index = 0;
	double Y_diff = 0.0; 
	ColorSpace color_space; 
};

ColorCorrectionParams getColorCorrectionTransformForPoints(std::vector<unsigned char> &RGB_source, std::vector<unsigned char> &RGB_dst, ColorSpace = CS_YUV);
void convertLAlphaBetaToRGB(const double l, const double alpha, const double beta, unsigned char &R, unsigned char &G, unsigned char &B);
void convertRGBToLAlphaBeta(const unsigned char R, const unsigned char G, const unsigned char B, double &l, double &alpha, double &beta);

void RGBToYUV(const unsigned char R, const unsigned char G, const unsigned char B, double &Y, double &U, double &V);
void YUVtoRGB(const double Y, const double U, const double V, unsigned char &R, unsigned char &G, unsigned char &B);

void applyColorCorrection(std::vector<unsigned char> &rgb, ColorCorrectionParams &color_correction_coeffs);
void applyColorCorrectionRGB(RGB* rgb, size_t n_elements, ColorCorrectionParams &color_correction_coeffs);
