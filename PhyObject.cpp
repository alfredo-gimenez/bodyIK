#include "PhyObject.h"

PhyObject::PhyObject()
{
	mRot = 0;
	mRotVel = 0;
	mMass = 0;
	mDamage = 0;
}

PhyObject::~PhyObject()
{
}

void PhyObject::update()
{
	mPos += mVel;
	mVel += mAcc;
	mRot += mRotVel;
}