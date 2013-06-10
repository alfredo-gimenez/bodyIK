#include "DecisionTree.h"
#include "stdlib.h"

#define _USE_MATH_DEFINES
#include <math.h>

Decision::Decision()
{
	memset(mNextDecisions,NULL,sizeof(mNextDecisions));
}

Decision::~Decision()
{
	for(int i=0; i<POSSIBLE_DECISIONS; i++)
	{
		if(mNextDecisions[i] != NULL)
			delete mNextDecisions[i];
	}
}

void Decision::populateNextDecisions(int mode, int depth)
{
	if(depth == 0)
		return;

	double theta;
	double mag;
	double xval;
	double yval;

	for(int i=0; i<POSSIBLE_DECISIONS; i++)
	{
		if(mNextDecisions[i])
			delete mNextDecisions[i];

		mNextDecisions[i] = new Decision();
		for(int j=0; j<NUM_BODY_PARTS; j++)
		{
			// Random choice on uniform random circle
			theta = ((double)rand()/(double)RAND_MAX)*2.0*M_PI; 
			mag = ((double)rand()/(double)RAND_MAX)*MAGNITUDE;

			xval = mag*cos(theta);
			yval = mag*sin(theta);

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
	mRoot = new Decision();
}

DecisionTree::~DecisionTree()
{
}

Decision* DecisionTree::makeNextDecision()
{
	int decisionIdx = ((double)rand()/(double)RAND_MAX)*POSSIBLE_DECISIONS;

	Decision *prevRoot = mRoot;
	mRoot = mRoot->mNextDecisions[decisionIdx];
	prevRoot->mNextDecisions[decisionIdx] = NULL;

	delete prevRoot;

	return mRoot;
}
