#pragma once

#include "phyobject.h"
#include <vector>

class EllipseObject :	public PhyObject
{
public:
	EllipseObject();
	EllipseObject(EllipseObject *ellipse);
	EllipseObject(vec2D center, 
		    double length, 
			double width, 
			double damageWeight = 1.0,
			double rotation = 0);
	~EllipseObject();

	void drawGL();
	bool collide(EllipseObject *other);

	int inContact;
	bool immobile;

private:
	double mLength;
	double mWidth;
	// get rid of these?
	vec2D width_Vector;
	vec2D length_Vector;

	void calculate_Ellipse_Vertices();
	bool isVerticeInside(vec2D);
	bool isLeft(vec2D a, vec2D b, vec2D c);
	std::vector<vec2D> ellipse_Vertices;
};

