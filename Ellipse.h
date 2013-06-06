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
	bool collide(EllipseObject &other);

private:
	double mLength;
	double mWidth;
	void calculate_Ellipse_Vertices();
	std::vector<vec2D> ellipse_Vertices;
};

