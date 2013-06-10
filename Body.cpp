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
	mBodyParts[SPINE]->addSegment(20.0,M_PI/2.0,10.0);
	mBodyParts[SPINE]->addSegment(20.0,0,10.0);
	mBodyParts[SPINE]->addSegment(20.0,0,10.0);
	mBodyParts[SPINE]->addSegment(30.0,0,10.0);
	mBodyParts[SPINE]->resetIK();

	// Arms are children of the 3rd spine link
	mBodyParts[LEFT_ARM] = new IKchain(&mBodyParts[SPINE]->mPositions[3]);
	mBodyParts[LEFT_ARM]->addSegment(40.0,-M_PI,10.0);
	mBodyParts[LEFT_ARM]->addSegment(70.0,M_PI/2.0,10.0);
	mBodyParts[LEFT_ARM]->addSegment(70.0,-M_PI/6.0,10.0);
	mBodyParts[LEFT_ARM]->addSegment(10.0,-M_PI/6.0,10.0);
	mBodyParts[LEFT_ARM]->resetIK();

	mBodyParts[RIGHT_ARM] = new IKchain(&mBodyParts[SPINE]->mPositions[3]);
	mBodyParts[RIGHT_ARM]->addSegment(40.0,0.0,10.0);
	mBodyParts[RIGHT_ARM]->addSegment(70.0,-M_PI/2.0,10.0);
	mBodyParts[RIGHT_ARM]->addSegment(70.0,M_PI/6.0,10.0);
	mBodyParts[RIGHT_ARM]->addSegment(10.0,M_PI/6.0,10.0);
	mBodyParts[RIGHT_ARM]->resetIK();

	mBodyParts[LEFT_LEG] = new IKchain(&mPos);
	mBodyParts[LEFT_LEG]->addSegment(20.0,-M_PI/4.0,10.0);
	mBodyParts[LEFT_LEG]->addSegment(70.0,-M_PI/4.0,10.0);
	mBodyParts[LEFT_LEG]->addSegment(70.0,0.0,10.0);
	mBodyParts[LEFT_LEG]->addSegment(10.0,M_PI/4.0,10.0);
	mBodyParts[LEFT_LEG]->resetIK();

	mBodyParts[RIGHT_LEG] = new IKchain(&mPos);
	mBodyParts[RIGHT_LEG]->addSegment(20.0,-M_PI/2.0 - M_PI/4.0,10.0);
	mBodyParts[RIGHT_LEG]->addSegment(70.0,M_PI/4.0,10.0);
	mBodyParts[RIGHT_LEG]->addSegment(70.0,0.0,10.0);
	mBodyParts[RIGHT_LEG]->addSegment(10.0,-M_PI/4.0,10.0);
	mBodyParts[RIGHT_LEG]->resetIK();
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
	// Move
	//translate(mVel);
	//mVel += mAcc;

	// Calculate IK controls
	FOR_ALL_LIMBS(calcIK());
	FOR_ALL_LIMBS(moveBodyParts());
}

void Body::collide(EllipseObject *obj)
{
	FOR_ALL_LIMBS(collide(obj));
}

void Body::translate(vec2D v)
{
	// Move origin
	mPos += v;

	// Move all IK controllers to match
	FOR_ALL_LIMBS(mGoal += v);

	// Calculate IK
	FOR_ALL_LIMBS(calcFK());
	FOR_ALL_LIMBS(moveBodyParts());
}

double Body::maxDamage()
{
	//TODO: this
	return 0;
}

double Body::assessDamage()
{
	//TODO: this
	return 0;
}

void Body::makeDecision(Decision *dec)
{
	for(int p=0; p<NUM_BODY_PARTS; p++)
	{
		mBodyParts[p]->moveGoal(dec->mDeltaVectors[p]);
	}
}
