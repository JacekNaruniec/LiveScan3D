#include "fileutils.h"
#include "windows.h"
#include "simplegraphic.h"
#include <atlimage.h>
#include "jpeglib.h"
#include "pnglib\png.h"
#include <string>
#include <filesystem>

using namespace std;
using namespace std::tr2::sys;

void saveJPG(const char * filename, unsigned char *data, int width, int height, int bpp)
{
	CImage im;
	bool delete_data = false;

	if ((width%8)!=0)
	{
		int new_w = width + (8 - width%8);
		unsigned char *new_data = new unsigned char[new_w*height*3];
		memset(new_data, 0, new_w*height*3);
		for (int i=0; i<height; i++)
			memcpy(new_data + i*new_w*3, data + i*width*3, width*3);
		
		data = new_data;
		width = new_w;
		delete_data = true;
	}

    // convert to bottom to up BGR
    unsigned char *tmp_data = new unsigned char[width * height * bpp];
    for (int i = 0; i < width; ++i)
        for (int j = 0; j < height; ++j)
        {
            unsigned char *ptr_src = data + (i + j*width)*bpp;
            unsigned char *ptr_dst = tmp_data + (i + (height - j - 1)*width)*bpp;
            *ptr_dst = *(ptr_src + 2);
            *(ptr_dst + 1) = *(ptr_src + 1);
            *(ptr_dst + 2) = *(ptr_src + 0);
        }

	BITMAPINFO *bip = prepareBitmapInfo(width, height);

	im.Create(width, height, 24);
	HDC hdc = im.GetDC();

	StretchDIBits(hdc, 0, 0, width, height, 0, 0, width, height, 
        tmp_data, bip, DIB_RGB_COLORS, SRCCOPY);

	im.Save(filename);
	im.ReleaseDC();
	if (delete_data) delete[] data;
    delete[]tmp_data;
	delete bip;
}

void abort_(const char * s, ...)
{
    va_list args;
    va_start(args, s);
    vfprintf(stderr, s, args);
    fprintf(stderr, "\n");
    va_end(args);
    abort();
}

unsigned char *loadPNG(const char *filename, int &iWidth, int &iHeight, int &bpp)
{
    FILE *fp = fopen(filename, "rb");
    png_byte color_type, bit_depth;

    png_structp png = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    if (!png) abort();
    
    png_infop info = png_create_info_struct(png);
    if (!info) abort();

    if (setjmp(png_jmpbuf(png))) abort();

    png_init_io(png, fp);

    png_read_info(png, info);

    iWidth = png_get_image_width(png, info);
    iHeight = png_get_image_height(png, info);
    color_type = png_get_color_type(png, info);
    bit_depth = png_get_bit_depth(png, info);
    
    // Read any color_type into 8bit depth, RGBA format.
    // See http://www.libpng.org/pub/png/libpng-manual.txt

    if (bit_depth == 16)
        png_set_strip_16(png);

    png_set_strip_alpha(png);

    if (color_type == PNG_COLOR_TYPE_PALETTE)
        png_set_palette_to_rgb(png);

    // PNG_COLOR_TYPE_GRAY_ALPHA is always 8 or 16bit depth.
    if (color_type == PNG_COLOR_TYPE_GRAY && bit_depth < 8)
        png_set_expand_gray_1_2_4_to_8(png);

    if (png_get_valid(png, info, PNG_INFO_tRNS))
        png_set_tRNS_to_alpha(png);

    png_read_update_info(png, info);

    int row_bytes = static_cast<int>(png_get_rowbytes(png, info));
    bpp = row_bytes / iWidth;

    png_bytep data = (png_bytep)malloc(sizeof(png_bytep) * iHeight * row_bytes);
    png_bytep *row_pointers = (png_bytep*)malloc(sizeof(png_bytep) * iHeight);
    for (int i = 0; i < iHeight; ++i)
        row_pointers[i] = data + i * row_bytes;
    png_read_image(png, row_pointers);

    fclose(fp);
    free(row_pointers);
    return data;
}

unsigned char *loadJPG(const char *filename, int &iWidth, int &iHeight, int &bpp)
{
	struct jpeg_decompress_struct cinfo;
	struct jpeg_error_mgr jerr;

	//PrintFullPath("..\\");

	cinfo.err = jpeg_std_error(&jerr);
	jpeg_create_decompress(&cinfo);

	FILE * infile;

	if ((infile = fopen(filename, "rb")) == NULL) {
	    fprintf(stderr, "can't open %s\n", filename);
	    return NULL;
	}
	jpeg_stdio_src(&cinfo, infile);

	boolean require_image = true;
	jpeg_read_header(&cinfo, require_image);

	jpeg_start_decompress(&cinfo);

	iWidth = cinfo.image_width;
	iHeight = cinfo.image_height;
	
	if (cinfo.out_color_components == JCS_GRAYSCALE)
		bpp = 1;
	else if (cinfo.out_color_components == JCS_RGB)
		bpp = 3;
	else if (cinfo.out_color_components == 3)
		bpp = 3;

	unsigned char *out_array = new unsigned char[iWidth*iHeight*bpp];
	JSAMPARRAY image = new JSAMPROW[iHeight];
	for (int i=0; i<iHeight; i++)
		image[i] = out_array + i*iWidth*bpp;

	int read_lines=0;
	while (cinfo.output_scanline < cinfo.output_height)
	{
		read_lines += jpeg_read_scanlines(&cinfo, image + read_lines, 1); 
	}
	jpeg_finish_decompress(&cinfo);
	fclose(infile);
	jpeg_destroy_decompress(&cinfo);
	delete []image;
	return out_array;
}


void skipLine(FILE *f)
{
	int c = -1;

	while (c!='\n')
		c = getc(f);
}

bool checkFileExistence(char *filename)
{
	FILE *f;

	if (fopen_s(&f, filename, "rt")!=0)
		return false;

	fclose(f);
	return true;
}

std::string* loadAllFileNames(char *path, char *filter, int &n_files, bool full_name)
{
	std::string *sFileNames;
	WIN32_FIND_DATA fdata;
	HANDLE hnd;
	bool found = true;

	char cwd[_MAX_PATH];
	GetCurrentDirectory(_MAX_PATH, cwd);

	SetCurrentDirectory(path);

	n_files=0;

	// finding number of files
	hnd = FindFirstFile(filter, &fdata);
	if (hnd == INVALID_HANDLE_VALUE){ 
		SetCurrentDirectory(cwd);
		return NULL; }

	while (found)
	{
		n_files++;
		found = (FindNextFile(hnd, &fdata)!=0);
	}

	if (n_files==0) 
	{
		SetCurrentDirectory(cwd);
		return NULL;
	}

	found = true;

	sFileNames = new std::string[n_files];

	FindClose(hnd);

	// loading filenames 
	n_files = 0;
	hnd = FindFirstFile(filter, &fdata);

	while (found)
	{
		if (full_name) sFileNames[n_files] = path; else sFileNames[n_files]= "";
		sFileNames[n_files].append(fdata.cFileName);
		n_files++;
		found = (FindNextFile(hnd, &fdata)!=0);
	}

	FindClose(hnd);
	SetCurrentDirectory(cwd);

	return sFileNames;
}

vector<string> getFilenamesInSubdirectories(string path, string extension, vector<string> &rawnames)
{
	vector<string> filenames; 
/*	rawnames.clear();

	for (recursive_directory_iterator i(path), end; i != end; ++i)
		if (!is_directory(i->path()))
		{
			string ext = i->path().extension();
			std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);
			if (ext == extension)
			{
				string parent_path;				
				filenames.push_back(i->path().file_string());
				rawnames.push_back(i->path().filename());
			}
		}
		*/
	return filenames; 
}
