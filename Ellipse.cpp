#define _USE_MATH_DEFINES
#include <math.h>
#include <Windows.h>

#include "Ellipse.h"

#include "glut.h"

EllipseObject::EllipseObject()
{
	mPos = vec2D();
	mVel = 0;
	mAcc = 0;
	mRot = 0;
	mRotVel = 0;

	mLength = 5.0;
	mWidth = 5.0;
}

EllipseObject::EllipseObject(vec2D center, 
				 double length, 
                 double width, 
				 double rotation)
{
	mPos = center;
	mVel = 0;
	mAcc = 0;
	mRot = rotation;
	mRotVel = 0;

	mLength = length;
	mWidth = width;
}


EllipseObject::~EllipseObject()
{
}

void EllipseObject::drawGL()
{
	double xlength = mLength*cos(mRot);
	double ylength = mLength*sin(mRot);

	vec2D lengthVec(xlength,ylength);

	vec2D beginPoint = mPos - (lengthVec/2.0);
	vec2D endPoint = mPos + (lengthVec/2.0);

	glLineWidth(5.0);
	glBegin(GL_LINES);
	{
		glColor3f(0,1,0);
		glVertex3f(beginPoint.x(),beginPoint.y(),0);
		glVertex3f(endPoint.x(),endPoint.y(),0);
	}
	glEnd();
	
	OutputDebugStringA("blah!\n");
}

bool EllipseObject::collide(EllipseObject &other, vec2D &newVec)
{
	return false;
}

