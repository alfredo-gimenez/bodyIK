#pragma once

#include <vector>
#include "vec2D.h"
#include "Ellipse.h"

#define MAX_IK_DISTANCE 1.0
#define MAX_IK_ITERS 50
#define MAX_ERROR 1.0
#define DEFAULT_MAG 0.1

class IKchain;

struct IKsegment
{
	IKsegment(double l=10, double t=0, double w=5)
	{
		mLength=l;
		mTheta=t;
		mWidth=w;
	}
	double mLength;
	double mTheta;
	double mWidth;
};

class IKchain
{
	friend class Body;

public:
	IKchain();
	IKchain(vec2D *origin);
	~IKchain();

	inline void recalcPositions() { calcFK(); moveBodyParts(); }
	inline void update() { doIKStep(); }
	inline unsigned int numSegments() { return mSegments.size(); }
	inline void setGoal(vec2D goal) { mGoal=goal; }
	inline void addSegment(double length, double theta, double width)
	{ 
		IKsegment seg(length,theta,width); 
		EllipseObject part(vec2D(),length,width);

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

	void drawGL();
	bool collide(EllipseObject*);

private:

	std::vector<IKsegment> mSegments;
	std::vector<EllipseObject> mBodyParts;
	std::vector<vec2D> mPositions;

	vec2D *mOrigin;
	vec2D mGoal;

	double mMagnitude;

	void calcSinesCosines(double *sines, double *cosines);
	void calcFK();
	void calcIK();
	void calcJacobian(double *out);
	void calcPseudoInverse(double *J, double *out);
	double calcError(double dpX, double dpY, double *J, double *JPI);
	double doIKStep();

	void moveBodyParts();
};

