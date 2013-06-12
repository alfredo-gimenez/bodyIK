#pragma once

#include <vector>

#include "vec2D.h"
#include "Scene.h"
#include "Body.h"

#define MAGNITUDE 800.0
#define SIMULATION_ITERATIONS LOOKAHEAD

class Scene;

class Decision
{
public:
	friend class DecisionTree;
	friend class Body;

	Decision();
	~Decision();

private:
	// How many vecs/decision?
	// how many next decisions?
	unsigned int mNumVectors;
	unsigned int mNumDecisions;

	// Depth of decisions below
	int mDepth;

	// The actual decision (bunch of vectors)
	std::vector<vec2D> mDeltaVectors;

	// The weights and pointers to next decisions
	std::vector<double> mDecisionWeights;
	std::vector<Decision*> mNextDecisions;

	void populateNextDecisions(int mode, int depth);
	void calculateDecisionWeights(Scene *scene, int depth);
	int getNextBestDecision();
};

class DecisionTree
{

public:
	// Every "decision" is a delta vector for all limbs
	DecisionTree();
	~DecisionTree();

	void populateDecisionTree(int mode, int depth = 1);
	void calculateDecisionWeights(Scene *scene);
	Decision* makeNextDecision();

private:
	int mDepth;
	Decision *mRoot;
};

