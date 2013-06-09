#pragma once

#include <vector>

#include "vec2D.h"
#include "PhyObject.h"
#include "Body.h"

#define MAGNITUDE 5.0
#define NUM_LIMBS 5
#define POSSIBLE_DECISIONS 1024

#define MONTECARLO_MODE 0
#define COMBO_MODE		1

class Decision
{
public:
	friend class DecisionTree;

	Decision();
	~Decision();

private:
	vec2D mDeltaVectors[NUM_LIMBS];
	double mDecisionWeights[POSSIBLE_DECISIONS];
	Decision* mNextDecisions[POSSIBLE_DECISIONS];

	std::vector<PhyObject*> mScene;
	Body *mBody;

	void populateNextDecisions(int mode,int depth);
	void calculateDecisionWeights();
	void pruneDecisions();
};

class DecisionTree
{

public:
	// Every "decision" is a delta vector for all limbs
	DecisionTree();
	~DecisionTree();

	void makeNextDecision();

private:
	Decision *mRoot;

	std::vector<PhyObject*> mScene;
	Body *mBody;

};

