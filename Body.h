#pragma once

#include "PhyObject.h"
#include "IKchain.h"
#include "DecisionTree.h"

#define NUM_BODY_PARTS 5

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

	IKchain *mBodyParts[NUM_BODY_PARTS];
private:

	void translate(vec2D v);
};

