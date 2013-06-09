#include "DecisionTree.h"
#include "stdlib.h"

Decision::Decision()
{

}

Decision::~Decision()
{

}

void Decision::populateNextDecisions(int mode, int depth)
{
	if(depth == 0)
		return;

	double xval;
	double yval;

	for(int i=0; i<POSSIBLE_DECISIONS; i++)
	{
		if(mNextDecisions[i])
			delete mNextDecisions[i];

		mNextDecisions[i] = new Decision();
		for(int j=0; j<NUM_LIMBS; j++)
		{
			xval = 2.0*((rand()/RAND_MAX) - 0.5)*MAGNITUDE;
			yval = 2.0*((rand()/RAND_MAX) - 0.5)*MAGNITUDE;

			mNextDecisions[i]->mDeltaVectors[j] = vec2D(xval,yval);
		}

		mNextDecisions[i]->populateNextDecisions(mode,depth-1);
	}
}

void Decision::calculateDecisionWeights()
{
	for(int i=0; i<POSSIBLE_DECISIONS; i++)
	{
		// Create a copy of the scene and simulate it one timestep forward
		for(unsigned int ob=0; ob<mScene.size(); ob++)
		{
			mScene[ob]->update();
		}

		mDecisionWeights[i] = mBody->maxDamage() / mBody->assessDamage();
	}

}

void Decision::pruneDecisions()
{

}

DecisionTree::DecisionTree()
{
}

DecisionTree::~DecisionTree()
{
}

void DecisionTree::makeNextDecision()
{

}
