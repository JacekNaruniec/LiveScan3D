#pragma once

#include "simplepoint.h"
#include <algorithm>

template <class T>
class SimpleRect
{
public:
	SimpleRect() {}

	SimpleRect(T _left, T _top, T _width, T _height)
	{
		width = _width;
		height = _height;
		left = _left;
		right = _left + _width;
		top = _top;
		bottom = _top + height;
	}

	T getLeft() const  { return left; }
	T getRight() const  { return right; }
	T getWidth() const  { return width; }
	T getHeight() const  { return height; }
	T getTop() const  { return top; }
	T getBottom() const  { return bottom; }

	void set2(T l, T t, T width, T height)
	{
		left = l;
		right = l + width;
		this->width = width; 
		this->height = height;
		top = t;
		bottom = t + height;
	}

	void set(T l, T r, T t, T b)
	{
		left = l;
		right = r;
		top = t;
		bottom = b;
		width = r - l;
		height = b - t;
	}

	void crop(T l, T r, T t, T b)
	{
		left = max(l, left);  right = max(l, right);
		left = min(r, left);  right = min(r, right);
		top = max(t, top);    bottom = max(t, bottom);
		top = min(b, top);    bottom = min(b, bottom);
		width = right - left;
		height = bottom - top;
	}

	void shiftXY(const T val_x, const T val_y)
	{
		left += val_x;
		right += val_x;
		top += val_y;
		bottom += val_y;
	}


	bool load(const char *filename)
	{
		FILE *f = fopen(filename, "rt");
		if (f == NULL) return false; 
		int l, t, w, h;

		fscanf(f, "%*s %d %*s %d %*s %d %*s %d", &l, &t, &w, &h);
		set2(l, t, w, h);
		fclose(f);

		return true;
	}

	bool save(const char *filename)
	{
		FILE *f = fopen(filename, "wt");
		
		if (f == NULL)
			return false;

		fprintf(f, "x= %d y= %d w= %d h= %d, ", getLeft(), getTop(), getWidth(), getHeight());
		fclose(f);

		return true; 
	}


	SimpleRect& operator*=(const float val)
	{
		left = (T)(left * val);
		right = (T)(right * val);
		top = (T)(top * val);
		bottom = (T)(bottom * val);
		width = right - left; 
		height = bottom - top;
		return *this; 
	}

	bool operator==(const SimpleRect &r)
	{
		if (r.left != left || r.right != right || r.top != top || r.bottom != bottom)
			return false;
		return true;
	}

	bool containPoT(SimplePoint<T> p)
	{
		if (p.x>left && p.x<right && p.y>top && p.y<bottom)
			return true;
		else
			return false;
	}

	void scale(float factor)
	{
		int newWidth = factor * width;
		int newHeight = factor * height;

		left = left - (newWidth - width) / 2;
		top = top - (newHeight - height) / 2;

		width = newWidth;
		height = newHeight;

		right = left + width;
		bottom = top + height;
	}

private:
	T left;
	T right;
	T top;
	T bottom;
	T width;
	T height;

};
