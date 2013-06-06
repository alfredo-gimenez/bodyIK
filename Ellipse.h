#pragma once

#include "phyobject.h"

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
	bool collide(EllipseObject &other, vec2D &newVec);

private:
	double mLength;
	double mWidth;
};

