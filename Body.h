#pragma once

#include "PhyObject.h"
#include "IKchain.h"
#include "DecisionTree.h"
#include "Scene.h"

enum BodyPartIndex
{
	SPINE = 0,
	LEFT_ARM,
	RIGHT_ARM,
	LEFT_LEG,
	RIGHT_LEG,
	NUM_BODY_PARTS
};

class Decision;

class Body : public PhyObject
{
public:
	Body();
	Body(Body *body);
	~Body();

	void drawGL();
	void update();
	void collide(EllipseObject*);

	double assessDamage();
	void makeDecision(Decision *dec);

private:
	std::vector<IKchain> mBodyParts;
	std::vector<vec2D> mIKDest;
};

