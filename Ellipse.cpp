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
	inContact=false;
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
	inContact=false;
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
	
	/*
	vec2D tVec(0,59);
	glColor3f(1,0.5,0.5);
	glBegin(GL_POINTS);
	glVertex3f(tVec.x(),tVec.y(),0);
	glEnd();
	bool inside = isVerticeInside(tVec);
	*/
	glBegin(GL_TRIANGLES);
	{
		if(inContact==true)
			glColor3f(1,0,0);
		else
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

	glLineWidth(1.0);
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

	int number_Of_Triangles = 10;
	width_Vector =  vec2D(cos(mRot+M_PI/2),sin(mRot+M_PI/2))*mWidth;
	length_Vector = vec2D(-sin(mRot+M_PI/2),cos(mRot+M_PI/2))*mLength*0.5;
	//Sweep vector around ellipse, vector is combo of V1 and V2;
	float val=mRot;
	for(int x=0;x<number_Of_Triangles;x++)
	{
		val =  (float)x/(number_Of_Triangles)*M_PI*2.0;
		ellipse_Vertices.push_back(mPos+(width_Vector)*cos(val) + (length_Vector*sin(val)));
	}
}

bool EllipseObject::collide( EllipseObject *other )
{
	calculate_Ellipse_Vertices();
	other->calculate_Ellipse_Vertices();
	bool collision=false;
	vec2D insideVertice;
	int closestVertice;
	for(int x=0;x<other->ellipse_Vertices.size();x++)
		if(isVerticeInside(other->ellipse_Vertices[x]))
		{
			collision=true;
			insideVertice=other->ellipse_Vertices[x];
		}	
		
		float distance =99999999999999.9;
		//Going to try getting normal a different way
		//First find closest vertice to the intersection vertice in object
		for(int x=0;x<ellipse_Vertices.size();x++)
		{
			vec2D dist = insideVertice - ellipse_Vertices[x];
			float tempDist = sqrt(dist.x()*dist.x() + dist.y()*dist.y());
			if(tempDist < distance)
			{
				closestVertice = x;
				distance=tempDist;
			}
		}
		

		if(collision && !inContact)
		{
			//Now can calculate normal
			vec2D line;
			if(closestVertice != ellipse_Vertices.size()-1)
				line = ellipse_Vertices[closestVertice] - ellipse_Vertices[closestVertice+1];
			else
				line = ellipse_Vertices[closestVertice] - ellipse_Vertices[0];

			line.normalize();
			vec2D normal;
			normal.x() = line.x()*cos(M_PI/2.0) - line.y()*sin(M_PI/2.0);
			normal.y() = line.x()*sin(M_PI/2.0) + line.y()*cos(M_PI/2.0);

			vec2D incomingVector(vec2D(other->mVel.x(),other->mVel.y()));
			//float mag = sqrt(other->mVel.x()*other->mVel.x() + other->mVel.y()*other->mVel.y());
			//float x = insideVertice.x();
			//float y = insideVertice.y();
			//float xtans = ((x-mPos.x())*-cos(mRot +M_PI/2.0) - (y-mPos.y())*-sin(mRot +M_PI/2.0))+mPos.x();
			//float ytrans = ((x-mPos.x())*-sin(mRot +M_PI/2.0) + (y-mPos.y())*-cos(mRot +M_PI/2.0))+mPos.y();

			//vec2D normal(xtans -mPos.x(),ytrans -mPos.y());
			
			//reflectionVec.normalize();
			//normal.normalize();
		/*	normal.x() = normal.x()*mLength*0.5;
			normal.y() = normal.y()*mWidth;

			normal.x() = x;
			normal.y() = y;*/
			vec2D reflectionVector = incomingVector - (normal*(2.0*(normal.dot(incomingVector))));
			
			other->speedupTo(reflectionVector*1.0);
			other->accelerateTo(vec2D(0,G_ACC*0.01));
		}

	//if(!other->inContact)
		inContact=collision;

	return collision;
}

bool EllipseObject::isVerticeInside(vec2D testVert)
{
	bool isInside=true;

	for(int x=0;x<ellipse_Vertices.size()-1;x++)
		{
			if(!isLeft(ellipse_Vertices[x],ellipse_Vertices[x+1],testVert))
				isInside=false;
		}

		if(!isLeft(ellipse_Vertices[ellipse_Vertices.size()-1],ellipse_Vertices[0],testVert))
			isInside=false;

		return isInside;
}

bool EllipseObject::isLeft(vec2D a, vec2D b, vec2D c)
{
     return ((b.x() - a.x())*(c.y() - a.y()) - (b.y() - a.y())*(c.x() - a.x())) >= 0;
}
