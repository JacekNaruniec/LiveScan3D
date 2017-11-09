#ifndef FILEUTILS
#define FILEUTILS

#include <string>
#include <vector>

void saveJPG(const char *filename, unsigned char *data, int width, int height, int bpp=3);
unsigned char * loadJPG(const char *filename, int &iWidth, int &iHeight, int &bpp);
unsigned char *loadPNG(const char *filename, int &iWidth, int &iHeight, int &bpp);
void skipLine(FILE *f);
bool checkFileExistence(char *filename);
std::string* loadAllFileNames(char *path, char *filter, int &n_files, bool full_name);
std::vector<std::string> getFilenamesInSubdirectories(std::string path, std::string extension, std::vector<std::string> &rawnames);


#endif
