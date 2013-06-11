#pragma once

#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

#include "vec2D.h"
#include "Ellipse.h"

#define MAX_IK_DISTANCE 0.01
#define MAX_IK_ITERS 100
#define MAX_ERROR 0.01
#define DEFAULT_MAG 0.06

class IKchain;

struct IKsegment
{
	IKsegment(double l=10, double w=10, double t=0,
		      double mint=-2.0*M_PI, double maxt=2.0*M_PI)
	{
		mLength=l;
		mWidth=w;
		mTheta=t;
		mMinTheta=mint;
		mMaxTheta=maxt;
	}
	double mLength;
	double mWidth;
	double mTheta;
	double mMinTheta;
	double mMaxTheta;
};

class IKchain
{
	friend class Body;

public:
	IKchain();
	IKchain(vec2D *origin);
	~IKchain();

	inline void update() { calcIK(); }
	void drawGL();
	bool collide(EllipseObject*);
	inline double getTotalDamage()
	{
		double total = 0;
		for(int i=0; i<numSegments(); i++)
		{
			total += mBodyParts[i].getDamage();
		}
		return total;
	}

private:
	inline unsigned int numSegments() { return mSegments.size(); }
	inline void setGoal(vec2D goal) { mGoal=goal; }
	inline void moveGoal(vec2D delta) { mGoal+=delta; }
	inline void addSegment(double length, 
						   double width, 
						   double theta, 
						   double minTheta=-2.0*M_PI,
						   double maxTheta=2.0*M_PI,
						   double damageWeight = 1.0)
	{ 
		IKsegment seg(length,width,theta,minTheta,maxTheta); 
		EllipseObject part(vec2D(),length,width,damageWeight);

		mSegments.push_back(seg); 
		mPositions.push_back(vec2D());
		mBodyParts.push_back(part);
	}
	inline void resetIK()
	{
		calcFK();
		moveBodyParts();
		mGoal = mPositions[numSegments()];
	}
	inline double totalLength()
	{
		double total = 0;
		for(int i=0; i<numSegments(); i++)
		{
			total += mSegments[i].mLength;
		}
		return total;
	}


private:

	std::vector<IKsegment> mSegments;
	std::vector<EllipseObject> mBodyParts;
	std::vector<vec2D> mPositions;

	vec2D *mOrigin;
	vec2D mGoal;

	void calcSinesCosines(double *sines, double *cosines);
	void calcFK();
	double calcIK();
	void calcJacobian(double *out);
	void calcPseudoInverse(double *J, double *out);
	double calcError(double dpX, double dpY, double *J, double *JPI);
	double doIKStep();

	void moveBodyParts();
};

