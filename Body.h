#pragma once

#include "PhyObject.h"
#include "IKchain.h"

class Body : public PhyObject
{
public:
	Body();
	~Body();

	void drawGL();
	void update();
	void collide(EllipseObject*);

	double maxDamage();
	double assessDamage();

private:
	IKchain *mSpine;
	IKchain *mLeftArm;
	IKchain *mRightArm;
	IKchain *mLeftLeg;
	IKchain *mRightLeg;

	void translate(vec2D v);
};

