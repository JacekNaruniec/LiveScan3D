#pragma once

#include <memory>
#include <windows.h>
#include <vector>
#include <string>


enum FileType { FT_PGM, FT_JPG, FT_PNG, FT_UNKNOWN, FT_AUTO };

class SimpleImage
{
public:
	SimpleImage() : width(-1), height(-1), data_ptr(NULL), bytes_per_pixel(1), bip(NULL) { }
	SimpleImage(const SimpleImage &si);
	
	int* getIntegralImagePtr();
	void updateIntegralImage();
	void subSample(double factor);

	int* getRotatedIntegralImagePtr();
	void updateRotatedIntegralImage();
	void create(int width, int height, int bytes_per_pixel, unsigned char *data);
	int width, height; 
	unsigned char *data_ptr;
	int bytes_per_pixel;

	std::vector<int> integral_image, sq_integral_image;
	std::vector<int> rotated_integral_image, sq_rotated_integral_image;

	void convertToBinary(unsigned char threshold);
	void convertToRGB();
	void cutImage(int x, int y, int w, int h);
	int scaleImage(double scale);
	void addBorder(int n_pixels);

	int getRegionSum(int x1, int y1, int w, int h) const;
	int getSqRegionSum(int x1, int y1, int w, int h) const;
	int getRotatedRegionSum(int x1, int y1, int w, int h) const;
	int getSqRotatedRegionSum(int x1, int y1, int w, int h) const;
    int readFromFile(const char *filename, FileType file_type = FT_AUTO);
    int writeToFile(const char *filename, FileType file_type = FT_AUTO);
	void addImage(SimpleImage &image);

	SimpleImage& operator=(const SimpleImage &si);
	unsigned char* operator[](int row); 

	void setSize(int _width, int _height);
	BITMAPINFO *getBip();
	void prepareBitmapInfo();

	void flipVertically();
	SimpleImage& convertToGrayscale();

	~SimpleImage();
private:
	void zeroVariables();
	void freeMemory();
    FileType recognizeFileType(std::string filename);


	BITMAPINFO *bip;
};
