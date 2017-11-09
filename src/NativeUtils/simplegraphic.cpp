#include "math.h"
#include "simplegraphic.h"

#ifndef min
#define min(a, b)  (((a) < (b)) ? (a) : (b))
#define max(a, b)  (((a) > (b)) ? (a) : (b))
#define abs(a)	   (((a) < 0) ? -(a) : (a))
#endif
void drawCross(int _x, int _y, int thickness, int size, int w, int h, byte *RGBPixels, int r, int g, int b)
{
	thickness-=1;

	for (int x=_x-size; x<=_x+size; x++)
		for (int y=_y - thickness; y<=_y+thickness; y++)
			drawPixel(x, y, w, h, RGBPixels, r, g, b);

	for (int y=_y-size; y<=_y+size; y++)
		for (int x=_x - thickness; x<=_x+thickness; x++)
			drawPixel(x, y, w, h, RGBPixels, r, g, b);

}

void erode(unsigned char *image, int w, int h)
{
	unsigned char *temp = new unsigned char[w*h];
	memset(temp, 0, w*h);
	int pos;

	for (int y=1; y<(h-1); y++)
	{
		pos = y*w+1;
		for (int x=1; x<(w-1); x++)
		{
			temp[pos] = min(min(min(min(image[pos], image[pos-1]), image[pos+1]), image[pos+w]), image[pos-w]);
			temp[pos] = min(min(min(min(temp[pos], image[pos-1-w]), image[pos+1-w]), image[pos-1+w]), image[pos+1+w]);
			pos++;
		}
	}
	memcpy(image, temp, w*h);

	delete []temp;	
}

void dilate(unsigned char *image, int w, int h)
{
	unsigned char *temp = new unsigned char[w*h];
	memset(temp, 0, w*h);
	int pos;

	for (int y=1; y<(h-1); y++)
	{
		pos = y*w+1;
		for (int x=1; x<(w-1); x++)
		{
			temp[pos] = max(max(max(max(image[pos], image[pos-1]), image[pos+1]), image[pos+w]), image[pos-w]);
			temp[pos] = max(max(max(max(temp[pos], image[pos-1-w]), image[pos+1-w]), image[pos-1+w]), image[pos+1+w]);
			pos++;
		}
	}
	memcpy(image, temp, w*h);

	delete []temp;	
}


SimplePoint<double> rotate2DPoint(SimplePoint<double> &p, float angle)
{
	SimplePoint<double> p2;
	float sina, cosa;
	sina = sin(angle);
	cosa = cos(angle);
	p2.x = p.x * cosa - p.y * sina;
	p2.y = p.y * cosa + p.x * sina;

	return p2;
}

void smoothImage(unsigned char *image, int w, int h)
{
    int wmax = w-2;
    int hmax = h-2;
    int w2 = w*2;
    unsigned char *temp = new unsigned char[w*h];
    unsigned char *pos_dest, *pos_source;
	
	memcpy(temp, image, w*h);

    for (int y=2; y<hmax; y++)
    {
        pos_source = image + y*w;
        pos_dest = temp + y*w;
        for (int x=2; x<wmax; x++)
            pos_dest[x] = (int)((pos_source[x-2] + 2*pos_source[x-1] + 4*pos_source[x] +
                          2*pos_source[x+1] + pos_source[x+2])*0.1f);
    }

    for (int y=2; y<hmax; y++)
    {
        pos_source = temp + y*w;
        pos_dest = image + y*w;
        for (int x=2; x<wmax; x++)
            pos_dest[x] = (unsigned char)((pos_source[x-w2] + 2*pos_source[x-w] + 4*pos_source[x] +
                          2*pos_source[x+w] + pos_source[x+w2])*0.1f);
    }

    delete []temp;
}


void copyRegion(unsigned char *dst, unsigned char *src, int w, int h, SimpleRect<int> region)
{
	int y_min = region.getTop();
	int y_max = region.getBottom();
	int x_min = region.getLeft();
	int rw = region.getRight() - region.getLeft();
	int pos;
	for (int y=y_min; y<y_max; y++)
	{
		pos = y*w + x_min;
		memcpy(dst + pos, src + pos, rw);
	}
}


void getPixelRGB(int x, int y, unsigned char *RGBPixels, int w, int h, unsigned char &R, 
	unsigned char &G, unsigned char &B)
{
	int l;
	if (x<w && y<h && x>0 && y>0)
	{
		l = 3*(h-y-1)*w + x*3;
		R = RGBPixels[l];
		G = RGBPixels[l+1];
		B = RGBPixels[l+2];
	}
}


void drawRectangleRGB(SimpleRect<int> rect, int x, int y, byte *pixelsRGB, int w, int h, int r, int g, int b,
					  int thickness)
{
	int start_p = thickness/2;
	SimpleRect<int>nrect;

	for (int i=0; i<thickness; i++)
	{
		nrect.set(rect.getLeft() - start_p + i,
			rect.getRight() - start_p - i,
			rect.getTop() - start_p + i,
			rect.getBottom() - start_p - i);
		drawRectangleRGB(nrect, x, y, pixelsRGB, w, h, r, g, b);
	}
}

void drawRectangleRGB(RECT rect, int x, int y, byte *pixelsRGB, int w, int h, int r, int g, int b)
{
	int i;

	for (i=max(0, (rect.left + x)); i<min((rect.right+x+1), w); i++)
	{
		drawPixel(i, y+rect.top, w, h, pixelsRGB, r, g, b);
		drawPixel(i, y+rect.bottom, w, h, pixelsRGB, r, g, b);
	}

	for (i=max(0, (rect.top + y)); i<min((rect.bottom+y+1), h); i++)
	{
		drawPixel(rect.left + x, i, w, h, pixelsRGB, r, g, b);
		drawPixel(rect.right + x, i, w, h, pixelsRGB, r, g, b);
	}
}


unsigned char *scaleImageToSize(unsigned char *data, int dest_w, int
 dest_h, int src_w, int src_h)
 {
   unsigned char *temp = new unsigned char[dest_w * dest_h];
   
   int x, y, sum, i, j, n, m;
   float nel;
   float scaleX = max(1.0f, (float)src_w/(float)dest_w);
   float scaleY = max(1.0f, (float)src_h/(float)dest_h);

   unsigned char *pos;

   for (x=0; x<dest_w; x++)
     for (y=0; y<dest_h; y++)
     {
       sum = 0;
       nel = 0;

       for (j=0; j<scaleY; j++)
       {
         n = (int)(y*scaleY)+j;
         m = (int)(x*scaleX);

         if (n>=src_h)
            n = src_h-1;

         pos = data + n*src_w;

         for (i=0; i<scaleX; i++)
         {
           if (m>=src_w) m = src_w-1;
           sum+=(*(pos+m));
           m++;
           nel++;

         }
       }
       temp[x + y*dest_w] = (unsigned char)((float)sum/nel);
     }
   
   return temp;
 }
 
BITMAPINFO* prepareBitmapInfo(int im_width, int im_height)
{
	BITMAPINFO *bip;
	bip = (BITMAPINFO*) new unsigned char[sizeof(BITMAPINFOHEADER)];
    bip->bmiHeader.biSize = sizeof(BITMAPINFOHEADER);
    bip->bmiHeader.biWidth = im_width;
    bip->bmiHeader.biHeight = im_height;
    bip->bmiHeader.biPlanes = 1;
    bip->bmiHeader.biBitCount = 24;
    bip->bmiHeader.biCompression = BI_RGB;
    bip->bmiHeader.biSizeImage = 0;
    bip->bmiHeader.biXPelsPerMeter = 0;
    bip->bmiHeader.biYPelsPerMeter = 0;
    bip->bmiHeader.biClrUsed = 0;
    bip->bmiHeader.biClrImportant = 0;
	return bip;
}

void changeBrightness(unsigned char *image, int w, int h, int val)
{
	int new_val;
	for (int i=0; i<(w*h); i++)
	{
		new_val = min(max((*image)+val, 0), 255);
		*image = (unsigned char)new_val;
		image++;
	}
}

void drawBigCrossC(int x, int y, int w, int h, byte *pixels, unsigned char val, int size, int thickness)
{
	SimpleRect<int>rect;
	rect.set(x - thickness, x + thickness, y - size, y + size);
	drawRectangleG(rect, 0, 0, pixels, w, h, val);

	rect.set(x - size, x + size, y - thickness, y + thickness);
	drawRectangleG(rect, 0, 0, pixels, w, h, val);

}

void drawBigCross(int x, int y, int w, int h, byte *RGBPixels, int r, int g, int b)
{
	(x, y, w, h, RGBPixels, r, g, b);
	drawSmallCross(x-1, y-1, w, h, RGBPixels, r, g, b);
	drawSmallCross(x+1, y+1, w, h, RGBPixels, r, g, b);
	drawSmallCross(x, y-1, w, h, RGBPixels, r, g, b);
	drawSmallCross(x, y-2, w, h, RGBPixels, r, g, b);
	drawSmallCross(x, y+1, w, h, RGBPixels, r, g, b);
	drawSmallCross(x, y+2, w, h, RGBPixels, r, g, b);
	drawSmallCross(x-1, y, w, h, RGBPixels, r, g, b);
	drawSmallCross(x-2, y, w, h, RGBPixels, r, g, b);
	drawSmallCross(x+1, y, w, h, RGBPixels, r, g, b);
	drawSmallCross(x+2, y, w, h, RGBPixels, r, g, b);
}

void drawRectangleRGB(SimpleRect<int> rect, int x, int y, byte *pixelsRGB, int w, int h, int r, int g, int b)
{
	int i;

	for (i=max(0, (rect.getLeft() + x)); i<min((rect.getRight()+x+1), w); i++)
	{
		drawPixel(i, y+rect.getTop(), w, h, pixelsRGB, r, g, b);
		drawPixel(i, y+rect.getBottom(), w, h, pixelsRGB, r, g, b);
	}

	for (i=max(0, (rect.getTop() + y)); i<min((rect.getBottom()+y+1), h); i++)
	{
		drawPixel(rect.getLeft() + x, i, w, h, pixelsRGB, r, g, b);
		drawPixel(rect.getRight() + x, i, w, h, pixelsRGB, r, g, b);
	}
}


void drawRectangleG(SimpleRect<int> rect, int x, int y, byte *pixels, int w, int h, unsigned char val)
{
	int i;


	for (i=max(0, (rect.getLeft() + x)); i<min((rect.getRight()+x), w-1); i++)
	{
		drawPixel(i, y+rect.getTop(), w, h, pixels, val);
		drawPixel(i, y+rect.getBottom(), w, h, pixels, val);
	}

	for (i=max(0, (rect.getTop() + y)); i<min((rect.getBottom()+y), h-1); i++)
	{
		drawPixel(rect.getLeft() + x, i, w, h, pixels, val);
		drawPixel(rect.getRight() + x, i, w, h, pixels, val);
	}
}


void drawPixel(int x, int y, int w, int h, byte *RGBPixels, int r, int g, int b)
{
	int l;

	if (x<w && y<h && x>0 && y>0)
	{
		l = 3*y*w + x*3;
		RGBPixels[l]   = (unsigned char)r;
		RGBPixels[l+1] = (unsigned char)g;
		RGBPixels[l+2] = (unsigned char)b;
	}
}

void drawPixel(int x, int y, int w, int h, byte *pixels, int val)
{
	if (x<w && y<h && x>0 && y>0)
		pixels[x+y*w]   = (byte)val;
}

void drawLine(int xa, int ya, int xb, int yb, int w, int h, byte *pixels, int val)
{
	double arc=0;
	double wx, wy, div;
	double l;
	int x1, x2, y1, y2, sgnx=1, sgny=1, sgn=1;

	xa = max(0, xa);
	xb = max(0, xb);
	ya = max(0, ya);
	yb = max(0, yb);

	xa = min(w-1, xa);
	xb = min(w-1, xb);
	ya = min(h-1, ya);
	yb = min(h-1, yb);


	x1 = min(xa, xb);
	x2 = max(xa, xb);
	y1 = min(ya, yb);
	y2 = max(ya, yb);
	
	wx = x1 - x2;
	wy = y1 - y2;
	
	if (wx==0) return;
	div = wy/wx;

	if (xb>xa) sgnx = -1;
	if (yb>ya) sgny = -1;

	arc = atan(div); 

	if ((sgnx==1 && sgny==1) ||
	   (sgnx==-1 && sgny==-1)) ;
	 else sgn = -1;
		 
	
	for (l=x1; l<x2; l++)
	{
		if (sgn==-1) wy = (x2-l) * tan(arc); 
		  else wy = (l-x1) * tan(arc);

		drawPixel((int)l, y1+(int)wy, w, h, pixels, val);
	}

	
	for (l=y1; l<y2; l++)
	{
		if (sgn==-1) wx = (y2-l) / tan(arc);
		  else wx = (l-y1) / tan(arc);
		
		drawPixel(x1+(int)wx, (int)l, w, h, pixels, val);
	}

}


void drawLine(int xa, int ya, int xb, int yb, int w, int h, byte *pixels, unsigned char r, unsigned char g,
				 unsigned char b)
{
	double arc=0;
	double wx, wy, div;
	double l;
	int x1, x2, y1, y2, sgnx=1, sgny=1, sgn=1;

	x1 = min(xa, xb);
	x2 = max(xa, xb);
	y1 = min(ya, yb);
	y2 = max(ya, yb);
	
	wx = x1 - x2;
	wy = y1 - y2;
	
	div = wy/wx;

	if (xb>xa) sgnx = -1;
	if (yb>ya) sgny = -1;

	arc = atan(div); 

	if ((sgnx==1 && sgny==1) ||
	   (sgnx==-1 && sgny==-1)) ;
	 else sgn = -1;
		 
	
	for (l=x1; l<x2; l++)
	{
		if (sgn==-1) wy = (x2-l) * tan(arc); 
		  else wy = (l-x1) * tan(arc);

		drawPixel((int)l, y1+(int)wy, w, h, pixels, r, g, b);
	}

	
	for (l=y1; l<y2; l++)
	{
		if (sgn==-1) wx = (y2-l) / tan(arc);
		  else wx = (l-y1) / tan(arc);
		
		drawPixel(x1+(int)wx, (int)l, w, h, pixels, r, g, b);
	}

}

void drawSmallCross(int x, int y, int w, int h, byte *RGBPixels, int r, int g, int b)
{
	drawPixel(x, y, w, h, RGBPixels, r, g, b);
	drawPixel(x+1, y, w, h, RGBPixels, r, g, b);
	drawPixel(x+2, y, w, h, RGBPixels, r, g, b);
	drawPixel(x+3, y, w, h, RGBPixels, r, g, b);
	drawPixel(x, y-1, w, h, RGBPixels, r, g, b);
	drawPixel(x, y-2, w, h, RGBPixels, r, g, b);
	drawPixel(x, y-3, w, h, RGBPixels, r, g, b);
	drawPixel(x-1, y, w, h, RGBPixels, r, g, b);
	drawPixel(x-2, y, w, h, RGBPixels, r, g, b);
	drawPixel(x-3, y, w, h, RGBPixels, r, g, b);
	drawPixel(x, y+1, w, h, RGBPixels, r, g, b);
	drawPixel(x, y+2, w, h, RGBPixels, r, g, b);
	drawPixel(x, y+3, w, h, RGBPixels, r, g, b);
}

void drawCross(int x, int y, int w, int h, byte *RGBPixels, int r, int g, int b)
{
	drawSmallCross(x, y, w, h, RGBPixels, r, g, b);
	drawSmallCross(x-1, y-1, w, h, RGBPixels, r, g, b);
	drawSmallCross(x-2, y-2, w, h, RGBPixels, r, g, b);
	drawSmallCross(x, y-1, w, h, RGBPixels, r, g, b);
	drawSmallCross(x, y-2, w, h, RGBPixels, r, g, b);
	drawSmallCross(x-1, y, w, h, RGBPixels, r, g, b);
	drawSmallCross(x-2, y, w, h, RGBPixels, r, g, b);
}



void drawCrossSimple(int x, int y, int w, int h, byte* pixels,byte val)
{
	drawPixel(x, y, w, h, pixels, val);
	drawPixel(x-1, y, w, h, pixels, val);
	drawPixel(x-2, y, w, h, pixels, val);
	drawPixel(x-3, y, w, h, pixels, val);
	drawPixel(x+1, y, w, h, pixels, val);
	drawPixel(x+2, y, w, h, pixels, val);
	drawPixel(x+3, y, w, h, pixels, val);
	drawPixel(x, y-1, w, h, pixels, val);
	drawPixel(x, y-2, w, h, pixels, val);
	drawPixel(x, y-3, w, h, pixels, val);
	drawPixel(x, y+1, w, h, pixels, val);
	drawPixel(x, y+2, w, h, pixels, val);
	drawPixel(x, y+3, w, h, pixels, val);

}

void copyRGBToGrayscale(byte *dest, byte *RGBSource, int w, int h)
{
	int x, y, diff;
	byte *ptrs, *ptrd;
	for (y=0; y<h; y++)
	{
	  diff = h-y-1;
	  
	  ptrs = RGBSource + y*w*3;
	  ptrd = dest      + diff*w;

	  for (x=0; x<w; x++)
 	  {
//		*ptrd = (0.11*(*ptrs)+0.30*(*(ptrs+1))+.59*(*(ptrs+2)));
		*ptrd = ((*ptrs)+*(ptrs+1)+*(ptrs+2))/3;
		ptrs+=3;
		ptrd+=1;
	  }
	}

}

void copyRGBToGrayscaleWithoutFlip(byte *dest, byte *RGBSource, int w, int h)
{
	int x, y, diff;
	byte *ptrs, *ptrd;
	for (y=0; y<h; y++)
	{
	  diff = y;
	  
	  ptrs = RGBSource + y*w*3;
	  ptrd = dest      + diff*w;

	  for (x=0; x<w; x++)
 	  {
//		*ptrd = (0.11*(*ptrs)+0.30*(*(ptrs+1))+.59*(*(ptrs+2)));
		*ptrd = ((*ptrs)+*(ptrs+1)+*(ptrs+2))/3;
		ptrs+=3;
		ptrd+=1;
	  }
	}

}

void copyRGBToGrayscale(byte *dest, byte *RGBSource, int w, int h, int extra)
{
	int x, y;
	byte *ptrs, *ptrd;
	for (y=0; y<h; y++)
	{ 
	  ptrs = RGBSource + y*(w+extra)*3;
	  ptrd = dest      + y*w;

	  for (x=0; x<w; x++)
 	  {
//		*ptrd = (0.11*(*ptrs)+0.30*(*(ptrs+1))+.59*(*(ptrs+2)));
		*ptrd = ((*ptrs)+*(ptrs+1)+*(ptrs+2))/3;
		ptrs+=3;
		ptrd+=1;
	  }
	}

}

void copyRGBToGrayscale(float *dest, byte *RGBSource, int w, int h)
{
	int x, y, diff;
	byte *ptrs;
	float *ptrd;
	for (y=0; y<h; y++)
	{
	  diff = h-y-1;
	  
	  ptrs = RGBSource + y*w*3;
	  ptrd = dest      + diff*w;

	  for (x=0; x<w; x++)
 	  {
   
//		*ptrd = 0.114*(*ptrs)+0.587*(*(ptrs+1))+0.299*(*(ptrs+2));
		*ptrd = (float)(((*ptrs)+*(ptrs+1)+*(ptrs+2))*0.33);
		ptrs+=3;
		ptrd+=1;
	  }
	}

}

// extraWidth must be difference between w and closest number that can be divided by 8
void copyGrayscaleToRGB(byte *RGBdest, byte *source, int w, int h, int extraWidth)
{
	int x, y;
	byte *ptrs, *ptrd;
	for (y=0; y<h; y++)
	{
	  ptrs = source + y*w;
	  ptrd = RGBdest + y*(w+extraWidth)*3;
	  for (x=0; x<w; x++)
 	  {
		*ptrd = *ptrs;
		*(ptrd+1) = *ptrs;
		*(ptrd+2) = *ptrs;
		ptrd+=3;
		ptrs+=1;
	  }
	
	  for (x=0; x<extraWidth; x++)
	  {
		  *ptrd = 0;
		  *(ptrd+1) = 0;
		  *(ptrd+2) = 0;
		  ptrd+=3;
	  }

	}


}

void flipRGBImageVertical(unsigned char *rgb_image, int w, int h)
{
	unsigned char *tmp = new unsigned char[w*h*3];
	memcpy(tmp, rgb_image, w*h*3);

	for (int y=0; y<h; y++)
		memcpy(rgb_image + y*w*3, tmp + (h-1-y)*w*3, w*3); 

	delete []tmp;
}

void convertRGBToBR(unsigned char *br_image, unsigned char *rgb_image, int w, int h)
{
	int wh = w*h;
	int R, G, B ,sum;

	memset(br_image, 0, w*h*2);
	for (int i=0; i<wh; i++)
	{
		R = *rgb_image;
		G = *(rgb_image+1);
		B = *(rgb_image+2);

		sum = R+G+B;

		if (sum==0)
		{
			rgb_image+=3;
			br_image+=2;
			continue;
		}

		*br_image = (G*255)/(R+G+B);
		*(br_image+1) = (B*255)/(R+G+B);

		rgb_image+=3;
		br_image+=2;
	}
}

void convertRGBToBRY(unsigned char *bry_image, unsigned char *rgb_image, int w, int h)
{
	int wh = w*h;
	int R, G, B ,sum;

	memset(bry_image, 0, w*h*3);
	for (int i=0; i<wh; i++)
	{
		R = *rgb_image;
		G = *(rgb_image+1);
		B = *(rgb_image+2);

		sum = R+G+B;

		if (sum==0)
		{
			rgb_image+=3;
			bry_image+=3;
			continue;
		}

		*bry_image = (G*255)/(R+G+B);
		*(bry_image+1) = (B*255)/(R+G+B);
		*(bry_image+2) = (unsigned char)((R+G+B)*0.33);

		rgb_image+=3;
		bry_image+=3;
	}
}

float rectangleIntersectionRatio(const SimpleRect<int> &r1, const SimpleRect<int> &r2)
{
	float intersectionRatio1 = 0, intersectionRatio2 = 0;
	SimpleRect<int> intersectingRect;
	int field1, field2, intersectionField;

	if (!rectangleIntersection(r1, r2)) return 0.0f;

	field1 = (r1.getRight() - r1.getLeft()) * (r1.getBottom() - r1.getTop());
	field2 = (r2.getRight() - r2.getLeft()) * (r2.getBottom() - r2.getTop());
	
	intersectingRect.set(max(r1.getLeft(), r2.getLeft()), min(r1.getRight(), r2.getRight()), max(r1.getTop(), r2.getTop()), min(r1.getBottom(), r2.getBottom()));
	
	intersectionField = (intersectingRect.getRight() - intersectingRect.getLeft()) * (intersectingRect.getBottom() - intersectingRect.getTop());
	
	intersectionRatio1 = min((float)intersectionField/(float)field1, field1/(float)intersectionField);
	intersectionRatio2 = min((float)intersectionField/(float)field2, field2/(float)intersectionField);

	return min(intersectionRatio1, intersectionRatio2);
}


bool rectangleIntersection(const SimpleRect<int> &r1, const SimpleRect<int> &r2)
{
     return !((r1.getLeft()) > (r2.getRight()) ||
         (r1.getBottom()) < (r2.getTop()) ||
         (r1.getRight()) < (r2.getLeft()) ||
         (r1.getTop()) > (r2.getBottom()));

}



void rotatePoint(float angle, int x0, int y0, int &x, int &y)
{
	int x2, y2;
	x2 = (int)(cos(angle)*(float)(x-x0) - sin(angle)*(float)(y-y0)) + x0;
	y2 = (int)(sin(angle)*(float)(x-x0) + cos(angle)*(float)(y-y0)) + y0;

	x = x2;
	y = y2;
	     	
}

void rotateImage(float angle, int x0, int y0, int iWidth, int iHeight, unsigned char *pbData, 
				 unsigned char **pbDataRows)
{
   byte *pbRotatedImage = new byte[iWidth*iHeight];
   byte **pbRotatedImageRows = new byte*[iHeight];

   memset(pbRotatedImage, 0, iWidth*iHeight);

   int i;
   int x2, y2;

   for (i=0; i<iHeight; i++)
	   pbRotatedImageRows[i] = pbRotatedImage + i*iWidth;
                   
   for (int x1 = 0; x1 < iWidth; x1++)
      for (int y1=0; y1 < iHeight; y1++)
      {
         x2 = (int)(cos(angle)*(x1-x0) - sin(angle)*(y1-y0)+x0);
         y2 = (int)(sin(angle)*(x1-x0) + cos(angle)*(y1-y0)+y0);	 
    	      
         if (x2>-1 && x2<iWidth && y2>-1 && y2<iHeight)
    	        pbRotatedImageRows[y1][x1] = pbDataRows[y2][x2];
      }

   memcpy(pbData, pbRotatedImage, iWidth*iHeight);
   delete []pbRotatedImage;
   delete []pbRotatedImageRows;
}

// returns specified rectangle area for the image
// x, y - anchoring point
// w, h - size of the new image 
// 
unsigned char* cutImage(int x, int y, int w, int h, unsigned char *data, 
				   int &iWidth, int &iHeight, bool deleteData)
{
   int i,j;
   byte *newData = new byte[w*h];

   memset(newData, 0, w*h);

   byte *ptr1, *ptr2;

   ptr1 = newData;

   for (i = 0; i < h; i++)
   {
     if ((y+i)>=iHeight || (y+i)<=0) continue;
     ptr2 = data + x + (i+y)*iWidth;

	 if ((i+y)<iHeight && (i+y)>=0)
       for (j = 0; j < w; j++)
	   {
		 if ((j+x)>=0 && (j+x)<iWidth) *ptr1 = *ptr2;
		 ptr1++;
		 ptr2++;
	   }
   }

   iWidth = w;
   iHeight = h;
   if (deleteData) delete []data;
   return newData;
}

unsigned char *scaleImage2(double scale, unsigned char *data, int &iWidth, int &iHeight)
{
   int iWOrg = iWidth;
   iWidth = (int)((double)iWidth/scale);
   iHeight = (int)((double)iHeight/scale);

   double countX=0, countY=0;
   
   byte *newData = new byte[iWidth*iHeight];
   byte *ptr1, *ptr2;    

   ptr1 = newData;
   ptr2 = data;
   for (int j=0; j<iHeight; j++)
   {
	   for (int i=0; i<iWidth; i++)
	   {
			*ptr1 = *ptr2;
			ptr1++;
			ptr2 = data + int(countX) + (int(countY)) * iWOrg;
			countX+=scale;
	   }
	 countY+=scale;
	 countX=0;
   }
   	 

 //  delete []data;

   return newData;
}

void scaleImage(double scale, unsigned char *data, int &iWidth, int &iHeight, unsigned char *newData)
{
   int iWOrg = iWidth;
   iWidth = (int)((double)iWidth/scale);
   iHeight = (int)((double)iHeight/scale);

   double countX=0, countY=0;
   
// byte *newData = new byte[iWidth*iHeight];
   byte *ptr1, *ptr2;    

   ptr1 = newData;
   ptr2 = data;
   for (int j=0; j<iHeight; j++)
   {
	   for (int i=0; i<iWidth; i++)
	   {
			*ptr1 = *ptr2;
			ptr1++;
			ptr2 = data + int(countX) + (int(countY)) * iWOrg;
			countX+=scale;
	   }
	 countY+=scale;
	 countX=0;
   }
   	 

   delete []data;

// return newData;
}

int* buildHistogram(byte *image, int w, int h)
{
	int *histogram = new int[256];
	int i, j;

	for (i=0; i<256; i++)
		histogram[i] = 0;

	for (i=0; i<w; i++)
		for (j=0; j<h; j++)
			histogram[image[i + j*w]]++;

	return histogram;

}

void equalizeHistogram(byte *image, int w, int h)
{
	int *histogram, *cumulative;
	int i, sum, tempval;
	byte *levels;
	int min_val;

	cumulative = new int[256];
	levels = new byte[256];
	histogram = buildHistogram(image, w, h);
	min_val = 0;
	for (i=0; i<=255; i++)
		if (histogram[i]>0) 
		{
			min_val = histogram[i];
			break;
		}
	sum = 0;
	for (i=0; i<=255; i++)
	{
		sum+=histogram[i];
		tempval = (int)(((sum - min_val)/(double)(w*h - min_val)) * 255.0); 
		tempval = min(tempval, 255);
		tempval = max(tempval, 0);

		levels[i] = (byte)tempval;
	}	

	for (i=0; i<w*h; i++)
		image[i] = levels[image[i]];
 
	delete []cumulative;
	delete []levels;
	delete []histogram;
	return;
}

int distance(int x1, int y1, int x2, int y2)
{
    return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
}

float distanceExact(int x1, int y1, int x2, int y2)
{
    return sqrt((float)((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)));
}

float distanceExact(float x1, float y1, float x2, float y2)
{
    return sqrt(((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2)));
}

float distance(float x1, float y1, float x2, float y2)
{
    return (x1-x2)*(x1-x2) + (y1-y2)*(y1-y2);
}

void findIntegralImage(unsigned char *img,int xres,int yres,int **iimage)
{		
    int y_pos, y;

    for(int x=0;x<=xres;x++) iimage[0][x]=0;
    for(y=0;y<=yres;y++) iimage[y][0]=0;		
    for(y=1;y<=yres;y++) {
        y_pos=(y-1)*xres;			
        for(int x=1;x<=xres;x++) iimage[y][x]=iimage[y][x-1]+iimage[y-1][x]-iimage[y-1][x-1]+img[x-1+y_pos];
    }		

}

void findIntegralImage(unsigned char *img,int xres,int yres,int *iimage)
{		
	int y_pos, y;
	int *iiy, *iiym1;
	int iiw = xres+1;
	int iih = yres+1;

	for(int x=0;x<=xres;x++) iimage[x]=0;
	for(y=0;y<=yres;y++)  	 iimage[y*iiw]=0;

	iiy = iimage + xres + 1;
	iiym1 = iiy - iiw;

	for(y=1;y<=yres;y++) {
		y_pos=(y-1)*xres;	

		for(int x=1;x<=xres;x++) 
			iiy[x]=iiy[x-1]+iiym1[x]-iiym1[x-1]+img[x-1+y_pos];

		iiy+=iiw;
		iiym1+=iiw;	
	}

}

void findIntegralImage(int *img,int xres,int yres,int *iimage)
{		
	int y_pos, y;
	int *iiy, *iiym1;
	int iiw = xres+1;
	int iih = yres+1;

	for(int x=0;x<=xres;x++) iimage[x]=0;
	for(y=0;y<=yres;y++)  	 iimage[y*iiw]=0;

	iiy = iimage + xres + 1;
	iiym1 = iiy - iiw;

	for(y=1;y<=yres;y++) {
		y_pos=(y-1)*xres;	

		for(int x=1;x<=xres;x++) 
			iiy[x]=iiy[x-1]+iiym1[x]-iiym1[x-1]+img[x-1+y_pos];

		iiy+=iiw;
		iiym1+=iiw;	
	}

}

void findRotatedIntegralImage(unsigned char *img,int xres,int yres,int *iimage)
{		
	int y;
	int iiw = xres+2;
	int iih = yres+1;

	for(int x=0;x<=(xres+1);x++) iimage[x]=0;
	for(y=0;y<=yres;y++)  	
	{
		iimage[y*iiw]=0;
		iimage[y*iiw+1]=0;
	}

	for (int y=1; y<iih; y++)
		for (int x=2; x<iiw; x++)
		{
			iimage[x + y*iiw] = iimage[x-1 + (y-1)*iiw] + iimage[x-1 + y*iiw] +
				img[x-2 + (y-1) * xres] - iimage[x-2 + (y-1) * iiw];
		}

	for (int y=(iih-2); y>0; y--)
		for (int x=(iiw-1); x>1; x--)
		{
			iimage[x + y*iiw]+=iimage[x-1 + (y+1)*iiw] - iimage[x-2 + y*iiw];
		}
}

void findRotatedIntegralImage(int *img,int xres,int yres,int *iimage)
{		
	int y;
	int iiw = xres+2;
	int iih = yres+1;

	for(int x=0;x<=(xres+1);x++) iimage[x]=0;
	for(y=0;y<=yres;y++)  	
	{
		iimage[y*iiw]=0;
		iimage[y*iiw+1]=0;
	}

	for (int y=1; y<iih; y++)
		for (int x=2; x<iiw; x++)
		{
			iimage[x + y*iiw] = iimage[x-1 + (y-1)*iiw] + iimage[x-1 + y*iiw] +
				img[x-2 + (y-1) * xres] - iimage[x-2 + (y-1) * iiw];
		}

	for (int y=(iih-2); y>0; y--)
		for (int x=(iiw-1); x>1; x--)
		{
			iimage[x + y*iiw]+=iimage[x-1 + (y+1)*iiw] - iimage[x-2 + y*iiw];
		}
}

void findIntegralImage(unsigned char *img,int xres,int yres,unsigned int *iimage)
{		
	int y_pos, y;
	unsigned int *iiy, *iiym1;
	int iiw = xres+1;
	int iih = yres+1;

	for(int x=0;x<=xres;x++) iimage[x]=0;
	for(y=0;y<=yres;y++)  	 iimage[y*iiw]=0;

	iiy = iimage + xres + 1;
	iiym1 = iiy - iiw;

	for(y=1;y<=yres;y++) {
		y_pos=(y-1)*xres;	

		for(int x=1;x<=xres;x++) 
			iiy[x]=iiy[x-1]+iiym1[x]-iiym1[x-1]+img[x-1+y_pos];

		iiy+=iiw;
		iiym1+=iiw;	
	}

}

void findIntegralImage(float *img,int xres,int yres,float *iimage)
{		
	int y_pos, y;
	float *iiy, *iiym1;
	int iiw = xres+1;
	int iih = yres+1;

	for(int x=0;x<=xres;x++) iimage[x]=0.0;
	for(y=0;y<=yres;y++)  	 iimage[y*iiw]=0.0;

	iiy = iimage + xres + 1;
	iiym1 = iiy - iiw;

	for(y=1;y<=yres;y++) {
		y_pos=(y-1)*xres;	

		for(int x=1;x<=xres;x++) 
			iiy[x]=iiy[x-1]+iiym1[x]-iiym1[x-1]+img[x-1+y_pos];

		iiy+=iiw;
		iiym1+=iiw;	
	}

}
