#define _USE_MATH_DEFINES
#include <math.h>
#include <Windows.h>

#include "Ellipse.h"
#include <vector>
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

	calculate_Ellipse_Vertices();
	
	glBegin(GL_TRIANGLES);
	{
		glColor3f(1,1,0);
		for(int x=0;x<ellipse_Vertices.size()-1;x++)
		{
			glVertex3f(mPos.x(),mPos.y(),0);
			glVertex3f(ellipse_Vertices[x].x(),ellipse_Vertices[x].y(),0);
			glVertex3f(ellipse_Vertices[x+1].x(),ellipse_Vertices[x+1].y(),0);
		}
		//The last triangle
		glVertex3f(mPos.x(),mPos.y(),0);
		glVertex3f(ellipse_Vertices[ellipse_Vertices.size()-1].x(),ellipse_Vertices[ellipse_Vertices.size()-1].y(),0);
		glVertex3f(ellipse_Vertices[0].x(),ellipse_Vertices[0].y(),0);

	}
	glEnd();


	glBegin(GL_LINES);
	{
		glColor3f(1,0,1);
		for(int x=0;x<ellipse_Vertices.size()-1;x++)
		{
			glVertex3f(ellipse_Vertices[x].x(),ellipse_Vertices[x].y(),0);
			glVertex3f(ellipse_Vertices[x+1].x(),ellipse_Vertices[x+1].y(),0);
		}
		glVertex3f(ellipse_Vertices[ellipse_Vertices.size()-1].x(),ellipse_Vertices[ellipse_Vertices.size()-1].y(),0);
		glVertex3f(ellipse_Vertices[0].x(),ellipse_Vertices[0].y(),0);

	}
	glEnd();


	OutputDebugStringA("blah!\n");
}

void EllipseObject::calculate_Ellipse_Vertices()
{

	ellipse_Vertices.clear();

	int number_Of_Triangles = 50;
	vec2D width_Vector =  vec2D(cos(mRot+M_PI/2.0),sin(mRot+M_PI/2.0))*mWidth;
	vec2D length_Vector = vec2D(cos(mRot+M_PI/2.0+M_PI/2.0),sin(mRot+M_PI/2.0+M_PI/2.0))*mLength*0.5;
	//Sweep vector around ellipse, vector is combo of V1 and V2;
	float val=mRot;
	for(int x=0;x<number_Of_Triangles;x++)
	{
		val =  (float)x/(number_Of_Triangles)*M_PI*2.0;
		ellipse_Vertices.push_back(mPos+(width_Vector)*cos(val) + (length_Vector*sin(val)));
	}

}

bool EllipseObject::collide( EllipseObject &other )
{
	
	
	
	
	
	return false;
}

