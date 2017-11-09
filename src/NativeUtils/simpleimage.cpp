#include "simpleimage.h"
#include "pgm.h"
#include "fileutils.h"
#include "simplegraphic.h"
#include <assert.h>
#include <string>
#include <algorithm>

using namespace std; 

SimpleImage::SimpleImage(const SimpleImage &si)
{
	zeroVariables();
	*this = si;
}

void SimpleImage::create(int width, int height, int bytes_per_pixel, unsigned char *data)
{
	if (data_ptr != NULL) delete[]data_ptr;


	if (data != nullptr)
	{
		data_ptr = new unsigned char[width*height*bytes_per_pixel];
		memcpy(data_ptr, data, width*height*bytes_per_pixel);
	}
	else
		data_ptr = new unsigned char[width*height*bytes_per_pixel]();

	this->width = width;
	this->height = height;
	this->bytes_per_pixel = bytes_per_pixel;

	prepareBitmapInfo();
}

void SimpleImage::addBorder(int n_pixels)
{
	int new_width = width + n_pixels * 2;
	int new_height = height + n_pixels * 2;
	unsigned char *new_data_ptr = new unsigned char[new_width * new_height * bytes_per_pixel];
	memset(new_data_ptr, 0, new_width * new_height * bytes_per_pixel);

	for (int y = 0; y < height; y++)
		memcpy(new_data_ptr + (n_pixels + (y + n_pixels) * new_width) * bytes_per_pixel,
		data_ptr + y * width * bytes_per_pixel, width*bytes_per_pixel);

	this->width = new_width;
	this->height = new_height;
	delete[]data_ptr;
	data_ptr = new_data_ptr;

	prepareBitmapInfo();
	
}


SimpleImage& SimpleImage::convertToGrayscale()
{
	if (bytes_per_pixel!=3 || width<0 || height<0) return *this;

	unsigned char *new_ptr = new unsigned char[width*height];

	copyRGBToGrayscale(new_ptr, data_ptr, width, height, 0);

	delete []data_ptr;
	data_ptr = new_ptr;
	bytes_per_pixel = 1;

	return *this;
}

void SimpleImage::subSample(double factor)
{
		int new_width = (int)(width / factor);
		int new_height = (int)(height / factor);
		int oy, ox;
		unsigned char *new_im = new unsigned char[new_height * new_width];

		for (int x = 0; x<new_width; x++)
		for (int y = 0; y<new_height; y++)
		{
			ox = min((int)(x * factor), width);
			oy = min((int)(y * factor), height);
			new_im[y * new_width + x] = data_ptr[oy * width + ox];
		}
		width = new_width;
		height = new_height;
		delete[]data_ptr;
		data_ptr = new_im;
}

void SimpleImage::addImage(SimpleImage &image)
{
	int new_w, new_h;
	new_w = width + image.width;
	new_h = max(height, image.height);

	unsigned char* new_data = new unsigned char[new_w * new_h];
	memset(new_data, 0, new_w*new_h);

	for (int i=0; i<height; i++)
		memcpy(new_data + i*new_w, data_ptr + i*width, width);

	for (int i=0; i<image.height; i++)
		memcpy(new_data + width + i*new_w, image.data_ptr + i*image.width, image.width);

	width = new_w;
	height = new_h;
	delete []data_ptr;
	data_ptr = new_data;
}


void SimpleImage::updateIntegralImage()
{
	if (integral_image.size()!=((width+1)*(height+1)))
	{
		integral_image.resize((width+1)*(height+1));
		sq_integral_image.resize((width+1)*(height+1));
	}
	findIntegralImage(data_ptr, width, height, integral_image.data());

	int *sq_data = new int[width*height];
	
	for (int i=0; i<width*height; i++)
		sq_data[i] = data_ptr[i] * data_ptr[i];
	findIntegralImage(sq_data, width, height, sq_integral_image.data());

	delete []sq_data;
}

void SimpleImage::flipVertically()
{
	unsigned char *tmp_im = new unsigned char[width*height*bytes_per_pixel];

	for (int y=0; y<height; y++)
		memcpy(tmp_im + y*width*bytes_per_pixel, data_ptr + (height - y - 1) * width * bytes_per_pixel,
			width*bytes_per_pixel);

	memcpy(data_ptr, tmp_im, width*height*bytes_per_pixel);
	delete []tmp_im;
}

void SimpleImage::convertToRGB()
{
	if (bytes_per_pixel==3 || width<=0 || height<=0) return;

	unsigned char *new_ptr;
	new_ptr = new unsigned char[width*height*3];
	int counter = 0;
	unsigned char val;

	for (int i=0; i<width*height; i++)
	{
		val = data_ptr[i];
	
		for (int j=0; j<3; j++)
			new_ptr[counter++] = val;
	}

	bytes_per_pixel = 3;
	delete []data_ptr;
	data_ptr = new_ptr;
}


void SimpleImage::convertToBinary(unsigned char threshold)
{
	for (int i=0; i<width*height; i++)
		if (data_ptr[i]>threshold)
			data_ptr[i] = 1;
		else
			data_ptr[i] = 0;
}

void SimpleImage::cutImage(int x, int y, int w, int h)
{
	unsigned char *new_ptr = new unsigned char[w*h*bytes_per_pixel];

	if (width != -1)
	{

		for (int i = 0; i < h; i++)
			memcpy(new_ptr + w*i*bytes_per_pixel, data_ptr + bytes_per_pixel*((y + i)*width + x), w*bytes_per_pixel);

		delete[]data_ptr;
	}

	integral_image.clear();
	rotated_integral_image.clear();

	data_ptr = new_ptr;
	width = w;
	height = h;
	prepareBitmapInfo();
}

void SimpleImage::updateRotatedIntegralImage()
{
	if (rotated_integral_image.size()!=((width+2)*(height+1)))
	{
		sq_rotated_integral_image.resize((width+2)*(height+2));
		rotated_integral_image.resize((width+2)*(height+1));
	}
	
	findRotatedIntegralImage(data_ptr, width, height, rotated_integral_image.data());

	int *sq_data = new int[width*height];
	
	for (int i=0; i<width*height; i++)
		sq_data[i] = data_ptr[i] * data_ptr[i];
	findRotatedIntegralImage(sq_data, width, height, sq_rotated_integral_image.data());

	delete []sq_data;
}

FileType SimpleImage::recognizeFileType(string filename)
{
    string type = filename;
    type.erase(type.begin(), type.end() - 3);
    std::transform(type.begin(), type.end(), type.begin(), ::tolower);
    if (type == "png")
        return FT_PNG;
    else if (type == "jpg")
        return FT_JPG;
    else if (type == "pgm")
        return FT_PGM;
    else
        return FT_UNKNOWN;
}

int SimpleImage::readFromFile(const char *filename, FileType file_type)
{
	freeMemory();
	zeroVariables();

    if (file_type == FileType::FT_AUTO)
    {
        file_type = recognizeFileType(filename);
        if (file_type == FT_UNKNOWN)
            return -1;
    }

    if (file_type == FileType::FT_PGM)
    {
        data_ptr = readPGM(filename, width, height, false);
        bytes_per_pixel = 1;
    }
    else if (file_type == FileType::FT_JPG)
		data_ptr = loadJPG(filename, width, height, bytes_per_pixel);
    
    else if (file_type == FileType::FT_PNG)
        data_ptr = loadPNG(filename, width, height, bytes_per_pixel);

    if (data_ptr != NULL) return 0;
    else return -1;

	return -1;
}

int SimpleImage::writeToFile(const char *filename, FileType file_type)
{
    if (file_type == FileType::FT_AUTO)
    {
        file_type = recognizeFileType(filename);
        if (file_type == FT_UNKNOWN)
            return -1;
    }

	if (file_type==FileType::FT_PGM)
	{
        if (bytes_per_pixel == 3)
        {
            SimpleImage grayscale = *this;
            grayscale.convertToGrayscale();
            writePGM(filename, width, height, grayscale.data_ptr);
        } else
    		writePGM(filename, width, height, data_ptr);
	}

	else if (file_type==FileType::FT_JPG || file_type==FileType::FT_PNG || file_type==FileType::FT_UNKNOWN)
	{
		if (bytes_per_pixel==1)
		{
			unsigned char *rgb_dest = new unsigned char[width*height*3];
			copyGrayscaleToRGB(rgb_dest, data_ptr, width, height, 0);
			saveJPG(filename, rgb_dest, width, height);
			delete []rgb_dest;
		}
		else
			saveJPG(filename, data_ptr, width, height);
	}


	return -1;
}

int SimpleImage::getRegionSum(int x1, int y1, int w, int h) const
{
	int iw = width + 1;

	return integral_image[x1 + y1*iw] + integral_image[x1 + w + (y1+h)*iw] 
			- integral_image[x1 + (y1+h)*iw] - integral_image[(x1+w) + y1*iw];
}

int SimpleImage::scaleImage(double scale)
{
	if (scale <= 0.0 || scale == 1.0)
		return -1;
	// nearest neighbour algorithm
	unsigned char *new_data;
	int new_width = static_cast<int>(width / scale);
	int new_height = static_cast<int>(height / scale);
	new_data = new unsigned char[new_width * new_height * bytes_per_pixel];

	for (int x = 0; x < new_width; ++x)
	{
		int src_x = min(width - 1, static_cast<int>(x * scale + 0.5));
		for (int y = 0; y < new_height; ++y)
		{
			int src_y = min(height - 1, static_cast<int>(y * scale + 0.5));
			memcpy(new_data + (x + y*new_width)*bytes_per_pixel, data_ptr + (src_x + src_y*width)*bytes_per_pixel, bytes_per_pixel);
		}
	}

	delete[]data_ptr;

    data_ptr = new_data; 
    width = new_width; 
    height = new_height;
    return 0;
}

int SimpleImage::getSqRegionSum(int x1, int y1, int w, int h) const
{
	int iw = width + 1;
	return sq_integral_image[x1 + y1*iw] + sq_integral_image[x1 + w + (y1+h)*iw] 
			- sq_integral_image[x1 + (y1+h)*iw] - sq_integral_image[(x1+w) + y1*iw];
}

int SimpleImage::getRotatedRegionSum(int x1, int y1, int w, int h) const
{
	int iw = width + 2;

	return rotated_integral_image[x1 + 2 + w + h + (y1 + 1 + w)*iw] +
		rotated_integral_image[x1 + 2 + (y1 + 1 + h)*iw] - 
		rotated_integral_image[x1 + h + 2 + (y1 + 1)*iw] -
		rotated_integral_image[x1 + 2 + w + (y1 + 1 + w + h)*iw];
}

int SimpleImage::getSqRotatedRegionSum(int x1, int y1, int w, int h) const
{
	int iw = width + 2;
	return sq_rotated_integral_image[x1 + 2 + w + h + (y1 + 1 + w)*iw] +
		sq_rotated_integral_image[x1 + 2 + (y1 + 1 + h)*iw] - 
		sq_rotated_integral_image[x1 + h + 2 + (y1 + 1)*iw] -
		sq_rotated_integral_image[x1 + 2 + w + (y1 + 1 + w + h)*iw];
}

void SimpleImage::zeroVariables()
{
	bip = NULL;
	data_ptr = NULL;
	bytes_per_pixel = 1;
}

int* SimpleImage::getIntegralImagePtr()
{
	if (integral_image.size()!=((width+1)*(height+1)))
		updateIntegralImage();

	return integral_image.data();
}

int* SimpleImage::getRotatedIntegralImagePtr()
{
	if (rotated_integral_image.size()!=((width+2)*(height+1)))
		updateRotatedIntegralImage();

	return rotated_integral_image.data();
}

void SimpleImage::prepareBitmapInfo()
{
	if (width==-1 || height==-1) return;

	if (bip!=NULL) delete bip;

    bip = (BITMAPINFO*) new unsigned char[sizeof(BITMAPINFOHEADER)];
    bip->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bip->bmiHeader.biWidth = width;
    bip->bmiHeader.biHeight = height;
    bip->bmiHeader.biPlanes = 1;
	bip->bmiHeader.biBitCount = bytes_per_pixel*8;
    bip->bmiHeader.biCompression = BI_RGB;
    bip->bmiHeader.biSizeImage = 0;
    bip->bmiHeader.biXPelsPerMeter = 0;
    bip->bmiHeader.biYPelsPerMeter = 0;
    bip->bmiHeader.biClrUsed = 0;
    bip->bmiHeader.biClrImportant = 0;
}


BITMAPINFO *SimpleImage::getBip()
{
	if (bip==NULL) prepareBitmapInfo();
	return bip;
}
	
unsigned char* SimpleImage::operator[](int row)
{
	return data_ptr + row*width*bytes_per_pixel;
}


SimpleImage& SimpleImage::operator=(const SimpleImage &si)
{
	if (data_ptr!=NULL) delete []data_ptr;
	if (bip != NULL)
	{
		delete bip;
		bip = NULL;
	}
	width = si.width;
	height = si.height;
	bytes_per_pixel = si.bytes_per_pixel;
	integral_image = si.integral_image;
	sq_integral_image = si.sq_integral_image;
	rotated_integral_image = si.rotated_integral_image;
	sq_rotated_integral_image = si.sq_rotated_integral_image;

	if (width!=-1 && height!=-1)
	{
		data_ptr = new unsigned char[width*height*bytes_per_pixel];
		memcpy(data_ptr, si.data_ptr, width*height*bytes_per_pixel);
	}

	prepareBitmapInfo();
	return *this;
}

void SimpleImage::setSize(int _width, int _height)
{
	width = _width; 
	height = _height; 
	prepareBitmapInfo();
}

void SimpleImage::freeMemory()
{
	if (data_ptr!=NULL) 
		delete []data_ptr;
	
	if (bip!=NULL)
		delete bip;
	sq_integral_image.clear();
	integral_image.clear();
	rotated_integral_image.clear();
	sq_rotated_integral_image.clear();

	zeroVariables();
}

SimpleImage::~SimpleImage()
{
	freeMemory();
}