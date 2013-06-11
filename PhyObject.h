#pragma once

#include "vec2D.h"

#define G_ACC -1.0

class PhyObject
{
public:
	PhyObject();
	~PhyObject();

	virtual void update();
	virtual void drawGL()=0;

	inline void accelerate(vec2D acc) { mAcc += acc; }
	inline void speedup(vec2D vel) { mVel += vel; }
	inline void move(vec2D pos) { mPos += pos; }

	inline void speedupRotation(double rot) { mRotVel += rot; }
	inline void rotate(double rot) { mRot += rot; }

	inline void accelerateTo(vec2D acc) { mAcc = acc; }
	inline void speedupTo(vec2D vel) { mVel = vel; }
	inline void moveTo(vec2D pos) { mPos = pos; }

	inline void speedupRotationTo(double rot) { mRotVel = rot; }
	inline void rotateTo(double rot) { mRot = rot; }

	inline void damage(unsigned int dmg) { mDamage += dmg; }
	inline double getDamage() const { return (double)mDamage*mDamageWeight; }

protected:
	vec2D mPos;
	vec2D mVel;
	vec2D mAcc;

	double mRot;
	double mRotVel;

	double mMass;

	unsigned int mDamage;
	double mDamageWeight;
};

