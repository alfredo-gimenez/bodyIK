#pragma once

#include "PhyObject.h"
#include "IKchain.h"
#include "DecisionTree.h"

#define NUM_BODY_PARTS 5
#define IK_DELTA_STEPS 10

enum BodyPartIndex
{
	SPINE = 0,
	LEFT_ARM,
	RIGHT_ARM,
	LEFT_LEG,
	RIGHT_LEG,
//	NUM_BODY_PARTS
};

class Decision;

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
	void makeDecision(Decision *dec);

private:
	IKchain *mBodyParts[NUM_BODY_PARTS];
	vec2D mIKDest[NUM_BODY_PARTS];
};

