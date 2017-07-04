#include "colorcorrection.h"
#include <algorithm>

using namespace std; 

ColorCorrectionParams getColorCorrectionTransform(std::vector<unsigned char> &RGB_source, std::vector<unsigned char> &RGB_dst, ColorSpace color_space)
{
	ColorCorrectionParams transform;

	if (RGB_dst.size() != RGB_source.size() || (RGB_dst.size() % 3) != 0 || RGB_dst.size() == 0)
		return transform;

	transform.color_space = color_space;

	size_t n_elements = RGB_source.size() / 3; 
	vector<double> values1(RGB_source.size());
	vector<double> values2(RGB_source.size());


	for (size_t i=0; i<RGB_source.size() / 3; i++)
	{
		double R1 = RGB_source[i * 3], G1 = RGB_source[i * 3 + 1], B1 = RGB_source[i * 3 + 2];
		double R2 = RGB_dst[i * 3], G2 = RGB_dst[i * 3 + 1], B2 = RGB_dst[i * 3 + 2];

		if (color_space == CS_LALPHABETA)
		{
			double l1, beta1, alpha1, l2, beta2, alpha2;
			convertRGBToLAlphaBeta(R1, G1, B1, l1, alpha1, beta1);
			convertRGBToLAlphaBeta(R2, G2, B2, l2, alpha2, beta2);

			values1[i * 3] = l1;
			values1[i * 3 + 1] = alpha1;
			values1[i * 3 + 2] = beta1;


			values2[i * 3] = l2;
			values2[i * 3 + 1] = alpha2;
			values2[i * 3 + 2] = beta2;
		}
		else
		{
			values1[i * 3] = R1;
			values1[i * 3 + 1] = G1;
			values1[i * 3 + 2] = B1;


			values2[i * 3] = R2;
			values2[i * 3 + 1] = G2;
			values2[i * 3 + 2] = B2;
		}
	}

	// calculate mean
	double mean1[3] = { 0.0, 0.0, 0.0 }, mean2[3] = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < n_elements; i++)
		for (int j = 0; j < 3; j++)
		{
			mean1[j] += values1[i * 3 + j];
			mean2[j] += values2[i * 3 + j];
		}

	for (int i = 0; i < 3; i++)
	{
		mean1[i] /= max(1.0, (double)n_elements);
		mean2[i] /= max(1.0, (double)n_elements);
	}

	// calculate stddev
	double stddev1[3] = { 0.0, 0.0, 0.0 }, stddev2[3] = { 0.0, 0.0, 0.0 };
	for (int i = 0; i < n_elements; i++)
		for (int j = 0; j < 3; j++)
		{
			stddev1[j] += abs(values1[i * 3 + j] - mean1[j]);
			stddev2[j] += abs(values2[i * 3 + j] - mean2[j]);
		}

	for (int i = 0; i < 3; i++)
	{
		stddev1[i] /= n_elements;
		stddev2[i] /= n_elements;
		stddev1[i] += 1e-15;
		stddev2[i] += 1e-15;
	}

	transform.std_scale_a = stddev1[0] / stddev2[0];
	transform.std_scale_b = stddev1[1] / stddev2[1];
	transform.std_scale_c = stddev1[2] / stddev2[2];

	transform.mean_a1 = mean1[0];
	transform.mean_b1 = mean1[1];
	transform.mean_c1 = mean1[2];

	transform.mean_a2 = mean2[0];
	transform.mean_b2 = mean2[1];
	transform.mean_c2 = mean2[2];

	return transform;
}


// equations from the article by E. Reinhart et al. "Color Transfer between Images", 2001
void convertRGBToLAlphaBeta(const unsigned char R, const unsigned char G, const unsigned char B, double &l, double &alpha, double &beta)
{
	double L = log10(0.3811 * R + 0.5783 * G + 0.0402 * B + 1e-15);
	double M = log10(0.1967 * R + 0.7244 * G + 0.0782 * B + 1e-15);
	double S = log10(0.0241 * R + 0.1288 * G + 0.8444 * B + 1e-15);

	l = (L + M + S) * 0.5773502691896258;
	alpha = (L + M - 2.0 * S) * 0.408248290463863;
	beta = (L - M) * 0.7071067811865475;
}

void convertLAlphaBetaToRGB(const double l, const double alpha, const double beta, unsigned char &R, unsigned char &G, unsigned char &B)
{
	double l1 = 0.5773502691896258 * l;
	double a1 = 0.408248290463863 * alpha;
	double b1 = 0.7071067811865475 * beta;

	double L = l1 + a1 + b1;
	double M = l1 + a1 - b1;
	double S = l1 - a1 * 2.0;

	L = pow(10, L);
	M = pow(10, M);
	S = pow(10, S);

	double Rp = L  * 4.4679 - M * 3.5873 + S * 0.1193;
	double Gp = -L * 1.2186 + M * 2.3809 - S * 0.1624;
	double Bp = L  * 0.0497 - M * 0.2439 + S * 1.2045;

	R = static_cast<unsigned char>(max(0.0, min(255.0, Rp)));
	G = static_cast<unsigned char>(max(0.0, min(255.0, Gp)));
	B = static_cast<unsigned char>(max(0.0, min(255.0, Bp)));
}


void applyColorCorrection(vector<unsigned char> &RGB, ColorCorrectionParams &color_correction_coeffs)
{
	size_t n_elements = RGB.size() / 3;

	for (size_t i = 0; i < n_elements; i++)
	{
		unsigned char R = RGB[i * 3];
		unsigned char G = RGB[i * 3 + 1];
		unsigned char B = RGB[i * 3 + 2];

		double a, b, c;

		if (color_correction_coeffs.color_space == CS_LALPHABETA)
			convertRGBToLAlphaBeta(R, G, B, a, b, c);
		else
		{	a = R; b = G; c = B;	}

		a = (a - color_correction_coeffs.mean_a2) * color_correction_coeffs.std_scale_a + color_correction_coeffs.mean_a1;
		b = (b - color_correction_coeffs.mean_b2) * color_correction_coeffs.std_scale_b + color_correction_coeffs.mean_b1;
		c = (c - color_correction_coeffs.mean_c2) * color_correction_coeffs.std_scale_c + color_correction_coeffs.mean_c1;
		
		if (color_correction_coeffs.color_space == CS_LALPHABETA)
			convertLAlphaBetaToRGB(a, b, c, R, G, B);
		else
		{
			R = (unsigned char)min(255, max(0, (int)a));
			G = (unsigned char)min(255, max(0, (int)b));
			B = (unsigned char)min(255, max(0, (int)c));
		}
		RGB[i * 3] = R;
		RGB[i * 3 + 1] = G;
		RGB[i * 3 + 2] = B;
	}
}

