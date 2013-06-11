#pragma once

#include <vector>

#include "vec2D.h"
#include "PhyObject.h"
#include "Body.h"

#define MAGNITUDE 100.0
#define POSSIBLE_DECISIONS 512

#define NUM_BODY_PARTS 5

class Decision
{
public:
	friend class DecisionTree;
	friend class Body;

	Decision();
	~Decision();

private:
	vec2D mDeltaVectors[NUM_BODY_PARTS];
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

	Decision* makeNextDecision();
	inline void createDecisions(int mode, int depth) 
		{ mRoot->populateNextDecisions(mode, depth); }

private:
	Decision *mRoot;

	std::vector<PhyObject*> mScene;
	Body *mBody;

};

