#include "bilateralFilter.h"
#include <math.h>


BilateralFilter::BilateralFilter(int diameter, double sigmaI, double sigmaS) : 
	diameter(diameter), sigmaI(sigmaI), sigmaS(sigmaS)
{
	coeffsS.resize(diameter * diameter);
	int half_diameter = diameter / 2;
	for (int i = 0; i < diameter; i++) {
		for (int j = 0; j < diameter; j++) {
			int pos = j + i * diameter;

			double gs = gaussian(distance(0, 0, i - half_diameter, j - half_diameter), sigmaS);
			coeffsS[pos] = gs; 
		}
	}

	gaussILUT.resize(65535 * 2);
	for (int i = 0; i < gaussILUT.size(); i++)
		gaussILUT[i] = gaussian(i, sigmaI);
}

float BilateralFilter::distance(int x, int y, int i, int j) {
	return float(sqrt(pow(x - i, 2) + pow(y - j, 2)));
}

double BilateralFilter::gaussian(double x, double sigma) {
	return exp(-(pow(x, 2)) / (2 * pow(sigma, 2))) / (2 * 3.14 * pow(sigma, 2));

}