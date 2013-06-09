#pragma once

#include "math.h"
class vec2D
{
public:

	// Constructor
	vec2D(double ix=0, double iy=0);

	inline double &x() { return mX; }
	inline double &y() { return mY; }

	 inline vec2D operator+(const vec2D &rhs)
		 {
			vec2D result = vec2D(mX+rhs.mX,mY+rhs.mY);
			return result;
		}
	 inline vec2D operator-(const vec2D &rhs)
		 {
			vec2D result = vec2D(mX-rhs.mX,mY-rhs.mY);
			return result;
		}
	 inline vec2D operator*(const double &rhs)
		 {
			vec2D result = vec2D(mX*rhs,mY*rhs);
			return result;
		}
	 inline vec2D operator/(const double &rhs)
		 {
			vec2D result = vec2D(mX/rhs,mY/rhs);
			return result;
		}
	 vec2D& operator=(const vec2D &rhs); 
	 vec2D& operator+=(const vec2D &rhs);

	 void normalize()
	 {
		 float length = sqrt(mX*mX + mY*mY);
		 mX = mX/length;
		 mY = mY/length;
	 }

	 float dot(vec2D v)
	 {
		 return (mX*v.mX + mY*v.mY);
	 }


private:
	double mX;
	double mY;
};
