#include "Body.h"

#define _USE_MATH_DEFINES
#include <math.h>

Body::Body()
{
	// Initialize at 0,0
	mPos = vec2D(0,0);
	mVel = vec2D(0,0);
	mAcc = vec2D(0,0);
	
	// Initialize body parts vector
	mBodyParts = std::vector<IKchain>(NUM_BODY_PARTS);

	// Create skeleton!
	mBodyParts[SPINE] = IKchain(&mPos);
	mBodyParts[SPINE].addSegment(20.0, 20.0, M_PI_2, M_PI_4/2.0, M_PI_2+M_PI_4/2.0, 48.0);
	mBodyParts[SPINE].addSegment(25.0, 30.0, 0.0, -M_PI_4/2.0, M_PI_4/2.0, 48.0);
	mBodyParts[SPINE].addSegment(25.0, 30.0, 0.0, -M_PI_4/2.0, M_PI_4/2.0, 48.0);
	mBodyParts[SPINE].addSegment(10.0, 10.0, 0.0, -M_PI_4/2.0, M_PI_4/2.0, 48.0);
	mBodyParts[SPINE].addSegment(40.0, 20.0, 0.0, -M_PI_4/2.0, M_PI_4/2.0, 80.0);
	mBodyParts[SPINE].resetIK();

	// Arms are children of the 3rd spine link
	mBodyParts[LEFT_ARM] = IKchain(&mBodyParts[SPINE].mPositions[3]);
	mBodyParts[LEFT_ARM].addSegment(40.0, 10.0, -M_PI, -M_PI-M_PI_4, -M_PI+M_PI_4, 20.0);
	mBodyParts[LEFT_ARM].addSegment(60.0, 10.0, M_PI_2, M_PI_2-M_PI_2, M_PI_2+M_PI_2, 1.0);
	mBodyParts[LEFT_ARM].addSegment(60.0, 10.0, -M_PI/6.0, -2.0*M_PI, 2.0*M_PI, 1.0);
	mBodyParts[LEFT_ARM].addSegment(50.0, 25.0, -M_PI/6.0, -2.0*M_PI, 2.0*M_PI, 1.0);
	mBodyParts[LEFT_ARM].resetIK();

	mBodyParts[RIGHT_ARM] = IKchain(&mBodyParts[SPINE].mPositions[3]);
	mBodyParts[RIGHT_ARM].addSegment(40.0, 10.0, 0.0,      -M_PI_4, M_PI_4, 20.0);
	mBodyParts[RIGHT_ARM].addSegment(60.0, 10.0, -M_PI_2,  -M_PI_2, M_PI_2, 1.0);
	mBodyParts[RIGHT_ARM].addSegment(60.0, 10.0, M_PI/6.0, -2.0*M_PI, 2.0*M_PI, 1.0);
	mBodyParts[RIGHT_ARM].addSegment(50.0, 25.0, M_PI/6.0, -2.0*M_PI, 2.0*M_PI, 0.0);
	mBodyParts[RIGHT_ARM].resetIK();

	mBodyParts[LEFT_LEG] = IKchain(&mPos);
	mBodyParts[LEFT_LEG].addSegment(25.0, 15.0, (-M_PI_2 - M_PI_4), -M_PI, (-M_PI_2 - M_PI_4), 60.0);
	mBodyParts[LEFT_LEG].addSegment(70.0, 15.0, M_PI_4, 0, M_PI_4, 10.0);
	mBodyParts[LEFT_LEG].addSegment(70.0, 10.0, 0.0,    0.0, M_PI_2, 5.0);
	mBodyParts[LEFT_LEG].addSegment(10.0, 10.0, -M_PI_4, -2.0*M_PI, 2.0*M_PI, 0.0);
	mBodyParts[LEFT_LEG].resetIK();

	mBodyParts[RIGHT_LEG] = IKchain(&mPos);
	mBodyParts[RIGHT_LEG].addSegment(25.0,15.0,-M_PI_4,-M_PI_4,0, 60.0);
	mBodyParts[RIGHT_LEG].addSegment(70.0,15.0,-M_PI_4,-M_PI_2,0, 10.0);
	mBodyParts[RIGHT_LEG].addSegment(70.0,10.0,0.0,-M_PI_2,0.0, 5.0);
	mBodyParts[RIGHT_LEG].addSegment(10.0,10.0,M_PI_4, -2.0*M_PI, 2.0*M_PI, 0.0);
	mBodyParts[RIGHT_LEG].resetIK();

	// Initialize IK destinations vector
	mIKDest = std::vector<vec2D>(NUM_BODY_PARTS);

	// Initialize IK destinations
	for(int i=0; i<mBodyParts.size(); i++)
	{
		mIKDest[i] = mBodyParts[i].mGoal;
	}
}

Body::Body(Body *body)
{
	// Copy body parts
	mBodyParts = std::vector<IKchain>(NUM_BODY_PARTS);
	std::copy(body->mBodyParts.begin(),
			  body->mBodyParts.end(),
			  mBodyParts.begin());

	// Copy IK destinations
	mIKDest = std::vector<vec2D>(NUM_BODY_PARTS);
	std::copy(body->mIKDest.begin(),
			  body->mIKDest.end(),
			  mIKDest.begin());
}

Body::~Body()
{
}

void Body::drawGL()
{
	for(int i=0; i<mBodyParts.size(); i++)
	{
		mBodyParts[i].drawGL();
	}
}

void Body::update()
{
	// Move IK controller 
	vec2D ikdelta;
	for(int i=0; i<mBodyParts.size(); i++)
	{
		ikdelta = mIKDest[i] - mBodyParts[i].mGoal;
		ikdelta = ikdelta / double(MOTION_ITERS);
		mBodyParts[i].moveGoal(ikdelta);
	}

	// Calculate IK controls
	double ikerr;
	for(int i=0; i<mBodyParts.size(); i++)
	{
		ikerr = mBodyParts[i].calcIK();
		mBodyParts[i].moveBodyParts();

		// IK not close enough, reset IK position
		if(ikerr > 20.0)
		{
			vec2D endPos = mBodyParts[i].mPositions[mBodyParts[i].numSegments()];
			mIKDest[i] = endPos;
		}
	}
}

void Body::collide(EllipseObject *obj)
{
	for(int i=0; i<mBodyParts.size(); i++)
	{
		mBodyParts[i].collide(obj);
	}
}

double Body::assessDamage()
{
	double totalDamage = 0;
	for(int i=0; i<mBodyParts.size(); i++)
	{
		totalDamage += mBodyParts[i].getTotalDamage();
	}
	return totalDamage;
}

void Body::makeDecision(Decision *dec)
{
	// Change the IK destinations
	for(int i=0; i<dec->mNumVectors; i++)
	{
		vec2D goal = mBodyParts[i].mGoal;
		vec2D delta = dec->mDeltaVectors[i];

		mIKDest[i] = mBodyParts[i].mGoal + dec->mDeltaVectors[i];
		
		// Constrain to farthest point
		vec2D destVec = mIKDest[i] - *mBodyParts[i].mOrigin;
		double partLength = mBodyParts[i].totalLength();
		if(destVec.length() > partLength)
		{
			destVec.normalize();
			mIKDest[i] = *mBodyParts[i].mOrigin + (destVec*partLength);
		}
	}
}
