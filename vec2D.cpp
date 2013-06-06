#include "vec2D.h"

vec2D::vec2D(double ix, double iy)
{
	mX=ix;
	mY=iy;
}

vec2D& vec2D::operator+(const vec2D &rhs)
{
	return vec2D(mX+rhs.mX,mY+rhs.mY);
}

vec2D& vec2D::operator-(const vec2D &rhs)
{
	return vec2D(mX-rhs.mX,mY-rhs.mY);
}

vec2D& vec2D::operator=(const vec2D &rhs)
{
	mX = rhs.mX;
	mY = rhs.mY;
	
	return *this;
}

vec2D& vec2D::operator+=(const vec2D &rhs)
{
	mX += rhs.mX;
	mY += rhs.mY;
	
	return *this;
}

vec2D& vec2D::operator*(const double &rhs)
{
	return vec2D(mX*rhs,mY*rhs);
}

vec2D& vec2D::operator/(const double &rhs)
{
	return vec2D(mX/rhs,mY/rhs);
}