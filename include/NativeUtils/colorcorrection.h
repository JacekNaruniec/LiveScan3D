#pragma once

#include <vector>

enum ColorSpace { CS_RGB, CS_LALPHABETA };

struct ColorCorrectionParams
{
	double mean_a1 = 0.0, mean_b1 = 0.0, mean_c1 = 0.0;
	double mean_a2 = 0.0, mean_b2 = 0.0, mean_c2 = 0.0;
	double std_scale_a = 1.0, std_scale_b = 1.0, std_scale_c = 1.0;
	int base_map_index = 0, map_index = 0;
	ColorSpace color_space; 
};

ColorCorrectionParams getColorCorrectionTransform(std::vector<unsigned char> &RGB_source, std::vector<unsigned char> &RGB_dst, ColorSpace = CS_RGB);
void convertLAlphaBetaToRGB(const double l, const double alpha, const double beta, unsigned char &R, unsigned char &G, unsigned char &B);
void convertRGBToLAlphaBeta(const unsigned char R, const unsigned char G, const unsigned char B, double &l, double &alpha, double &beta);
void applyColorCorrection(std::vector<unsigned char> &RGB, ColorCorrectionParams &color_correction_coeffs);
