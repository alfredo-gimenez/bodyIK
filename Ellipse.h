#pragma once

#include "phyobject.h"
#include <vector>

class EllipseObject :	public PhyObject
{
public:
	EllipseObject();
	EllipseObject(vec2D center, 
		    double length, 
			double width, 
			double rotation = 0);
	~EllipseObject();

	void drawGL();
	bool collide(EllipseObject *other);
	bool inContact;
private:
	double mLength;
	double mWidth;
	vec2D width_Vector;
	vec2D length_Vector;

	void calculate_Ellipse_Vertices();
	bool isVerticeInside(vec2D);
	bool isLeft(vec2D a, vec2D b, vec2D c);
	std::vector<vec2D> ellipse_Vertices;
};

