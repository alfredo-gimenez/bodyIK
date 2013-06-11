#include "Body.h"

#define _USE_MATH_DEFINES
#include <math.h>

#define FOR_ALL_LIMBS(f) { \
	for(int xkcd=0; xkcd<NUM_BODY_PARTS; xkcd++) \
	{ \
		mBodyParts[xkcd]->f; \
	} \
}

Body::Body()
{
	// Initialize at 0,0
	mPos = vec2D(0,0);
	mVel = vec2D(0,0);
	mAcc = vec2D(0,0);

	// Create skeleton!
	mBodyParts[SPINE] = new IKchain(&mPos);
	mBodyParts[SPINE]->addSegment(20.0, 20.0, M_PI_2, M_PI_4/2.0, M_PI_2+M_PI_4/2.0, 48.0);
	mBodyParts[SPINE]->addSegment(25.0, 30.0, 0.0, -M_PI_4/2.0, M_PI_4/2.0, 48.0);
	mBodyParts[SPINE]->addSegment(25.0, 30.0, 0.0, -M_PI_4/2.0, M_PI_4/2.0, 48.0);
	mBodyParts[SPINE]->addSegment(10.0, 10.0, 0.0, -M_PI_4/2.0, M_PI_4/2.0, 48.0);
	mBodyParts[SPINE]->addSegment(40.0, 20.0, 0.0, -M_PI_4/2.0, M_PI_4/2.0, 80.0);
	mBodyParts[SPINE]->resetIK();

	// Arms are children of the 3rd spine link
	mBodyParts[LEFT_ARM] = new IKchain(&mBodyParts[SPINE]->mPositions[3]);
	mBodyParts[LEFT_ARM]->addSegment(40.0, 10.0, -M_PI, -M_PI-M_PI_4, -M_PI+M_PI_4, 20.0);
	mBodyParts[LEFT_ARM]->addSegment(60.0, 10.0, M_PI_2, M_PI_2-M_PI_2, M_PI_2+M_PI_2, 1.0);
	mBodyParts[LEFT_ARM]->addSegment(60.0, 10.0, -M_PI/6.0, 1.0);
	mBodyParts[LEFT_ARM]->addSegment(15.0, 10.0, -M_PI/6.0, 0.0);
	mBodyParts[LEFT_ARM]->resetIK();

	mBodyParts[RIGHT_ARM] = new IKchain(&mBodyParts[SPINE]->mPositions[3]);
	mBodyParts[RIGHT_ARM]->addSegment(40.0, 10.0, 0.0, -M_PI_4, M_PI_4, 20.0);
	mBodyParts[RIGHT_ARM]->addSegment(60.0, 10.0, -M_PI_2, -M_PI_2, M_PI_2, 1.0);
	mBodyParts[RIGHT_ARM]->addSegment(60.0, 10.0, M_PI/6.0, 1.0);
	mBodyParts[RIGHT_ARM]->addSegment(15.0, 10.0, M_PI/6.0, 0.0);
	mBodyParts[RIGHT_ARM]->resetIK();

	mBodyParts[LEFT_LEG] = new IKchain(&mPos);
	mBodyParts[LEFT_LEG]->addSegment(25.0, 15.0, (-M_PI_2 - M_PI_4), -M_PI, (-M_PI_2 - M_PI_4), 60.0);
	mBodyParts[LEFT_LEG]->addSegment(70.0, 15.0, M_PI_4, 0, M_PI_4, 10.0);
	mBodyParts[LEFT_LEG]->addSegment(70.0, 10.0, 0.0, 0.0, M_PI_2, 5.0);
	mBodyParts[LEFT_LEG]->addSegment(10.0, 10.0, -M_PI_4, 0.0);
	mBodyParts[LEFT_LEG]->resetIK();

	mBodyParts[RIGHT_LEG] = new IKchain(&mPos);
	mBodyParts[RIGHT_LEG]->addSegment(25.0,15.0,-M_PI_4,-M_PI_4,0, 60.0);
	mBodyParts[RIGHT_LEG]->addSegment(70.0,15.0,-M_PI_4,-M_PI_2,0, 10.0);
	mBodyParts[RIGHT_LEG]->addSegment(70.0,10.0,0.0,-M_PI_2,0.0, 5.0);
	mBodyParts[RIGHT_LEG]->addSegment(10.0,10.0,M_PI_4, 0.0);
	mBodyParts[RIGHT_LEG]->resetIK();

	for(int i=0; i<NUM_BODY_PARTS; i++)
	{
		mIKDest[i] = mBodyParts[i]->mGoal;
	}
}

Body::~Body()
{
	for(int p=0; p<NUM_BODY_PARTS; p++)
	{
		delete mBodyParts[p];
	}
	delete mBodyParts;
}

void Body::drawGL()
{
	FOR_ALL_LIMBS(drawGL());
}

void Body::update()
{
	// Move IK controller 
	vec2D ikdelta;
	for(int i=0; i<NUM_BODY_PARTS; i++)
	{
		ikdelta = mIKDest[i] - mBodyParts[i]->mGoal;
		ikdelta = ikdelta / double(IK_DELTA_STEPS);
		mBodyParts[i]->moveGoal(ikdelta);
	}

	// Calculate IK controls
	double ikerr;
	for(int i=0; i<NUM_BODY_PARTS; i++)
	{
		ikerr = mBodyParts[i]->calcIK();
		mBodyParts[i]->moveBodyParts();

		// IK not close enough, reset IK position
		if(ikerr > 1.0)
		{
			vec2D endPos = mBodyParts[i]->mPositions[mBodyParts[i]->numSegments()];
			//mBodyParts[i]->setGoal(endPos);
			mIKDest[i] = endPos;
		}
	}
}

void Body::collide(EllipseObject *obj)
{
	FOR_ALL_LIMBS(collide(obj));
}

double Body::assessDamage()
{
	double totalDamage = 0;
	for(int i=0; i<NUM_BODY_PARTS; i++)
	{
		totalDamage += mBodyParts[i]->getTotalDamage();
	}
	return totalDamage;
}

void Body::makeDecision(Decision *dec)
{
	// Change the IK destinations
	for(int p=0; p<NUM_BODY_PARTS; p++)
	{
		mIKDest[p] = mBodyParts[p]->mGoal + dec->mDeltaVectors[p];
		
		// Constrain to farthest point
		vec2D destVec = mIKDest[p] - *mBodyParts[p]->mOrigin;
		double partLength = mBodyParts[p]->totalLength();
		if(destVec.length() > partLength)
		{
			destVec.normalize();
			mIKDest[p] = *mBodyParts[p]->mOrigin + (destVec*partLength);
		}
	}
}
