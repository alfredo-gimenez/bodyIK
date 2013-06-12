#include "DecisionTree.h"
#include "Body.h"
#include "stdlib.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>

Decision::Decision()
	:
	mDepth(0),
	mNumDecisions(NUM_DECISIONS),
	mNumVectors(3)
{
	mDeltaVectors.resize(mNumVectors,vec2D());
	mNextDecisions.resize(mNumDecisions,NULL);
	mDecisionWeights.resize(mNumDecisions,0);
}

Decision::~Decision()
{
	for(int i=0; i<mNumDecisions; i++)
	{
		if(mNextDecisions[i] != NULL)
			delete mNextDecisions[i];
	}
}

void Decision::populateNextDecisions(int mode, int depth)
{
	mDepth = depth;

	if(depth == 0)
		return;

	double theta;
	double mag;
	double xval;
	double yval;

	srand(clock());

	for(int i=0; i<mNumDecisions; i++)
	{
		if(mNextDecisions[i])
			delete mNextDecisions[i];

		mNextDecisions[i] = new Decision();
		for(int j=0; j<mNumVectors; j++)
		{
			// Random choice on uniform random circle
			theta = ((double)rand()/(double)RAND_MAX)*2.0*M_PI; 
			//mag = ((double)rand()/(double)RAND_MAX)*MAGNITUDE;
			mag = MAGNITUDE;

			xval = (i==0) ? 0 : mag*cos(theta);
			yval = (i==0) ? 0 : mag*sin(theta);

			mNextDecisions[i]->mDeltaVectors[j] = vec2D(xval,yval);
		}

		mNextDecisions[i]->populateNextDecisions(mode,depth-1);
	}
}

void Decision::calculateDecisionWeights(Scene *scene, int depth)
{
	if(depth == 0)
		return;
	
#pragma omp parallel
	{
#pragma omp for
		for(int i=0; i<mNumDecisions; i++)
		{
			// Simulate the decision and all next decisions up to depth
			double weight;
			Scene *sceneCopy = new Scene(scene);
			sceneCopy->makeDecision(mNextDecisions[i]);

			int iterations = (depth > 1) ? MOTION_ITERS : SIMULATION_ITERATIONS;
			for(int j=0; j<iterations; j++)
			{
				sceneCopy->update();
			}

			weight = sceneCopy->getSceneScore();
			mDecisionWeights[i] = weight;

			//if(weight - scene->getSceneScore() == 0)
			//{
			//	mDecisionWeights[i] = -1.0;
			//	break;
			//}

			// All following decisions after this
			mNextDecisions[i]->calculateDecisionWeights(sceneCopy, depth-1);
			if(depth > 1)
			{
				int bestSubDecision = mNextDecisions[i]->getNextBestDecision();
				mDecisionWeights[i] = mNextDecisions[i]->mDecisionWeights[bestSubDecision];
			}

			delete sceneCopy;
		}
	}
}

int Decision::getNextBestDecision()
{
	// Choose decision with lowest weight
	int bestDecision = 0;
	double bestWeight = std::numeric_limits<double>::max();

	// TODO: best combination of next decisions
	for(int i=0; i<mNumDecisions; i++)
	{
		if(mDecisionWeights[i] < bestWeight)
		{
			bestDecision = i;
			bestWeight = mDecisionWeights[i];
		}
	}

	// Return index
	return bestDecision;
}

DecisionTree::DecisionTree()
	:
	mDepth(0)
{
	mRoot = new Decision();
}

DecisionTree::~DecisionTree()
{
}

void DecisionTree::populateDecisionTree(int mode, int depth)
{
	mDepth = depth;
	mRoot->populateNextDecisions(mode,depth);
}

void DecisionTree::calculateDecisionWeights(Scene *scene)
{
	mRoot->calculateDecisionWeights(scene, mDepth);
}

Decision* DecisionTree::makeNextDecision()
{
	Decision *prevRoot = mRoot;

	// Get next decision, set as root, and unlink from prev root
	int decision = mRoot->getNextBestDecision();
	mRoot = mRoot->mNextDecisions[decision];
	prevRoot->mNextDecisions[decision] = NULL;

	// Free up unmade decisions (NO REGRETS!!)
	delete prevRoot;

	return mRoot;
}
