#include "DecisionTree.h"
#include "Body.h"
#include "stdlib.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>
#include <omp.h>

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

#define ROWMAJ(xx,yy,zz,n) xx*n*n+yy*n+zz

double round(double number)
{
    return number < 0.0 ? ceil(number - 0.5) : floor(number + 0.5);
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

	int numDirections = round(pow(mNumDecisions,1.0/3.0));
	mag = MAGNITUDE;

	std::vector<vec2D> dirs;
	for(int d=0; d<numDirections; d++)
	{
		theta = ((double)d/(double)numDirections) * (2.0*M_PI) ;
		xval = mag*cos(theta);
		yval = mag*sin(theta);
		dirs.push_back(vec2D(xval,yval));
	}

	int count=0;
	for(int id=0; id<numDirections; id++)
	{
		for(int j=0; j<numDirections; j++)
		{
			for(int k=0; k<numDirections; k++)
			{
				if(mNextDecisions[count])
					delete mNextDecisions[count];
				mNextDecisions[count] = new Decision();

				mNextDecisions[count]->mDeltaVectors[0] = dirs[id];
				mNextDecisions[count]->mDeltaVectors[1] = dirs[j];
				mNextDecisions[count]->mDeltaVectors[2] = dirs[k];
				mNextDecisions[count]->populateNextDecisions(mode,depth-1);
				count++;
			}
		}
	}
}

void Decision::calculateDecisionWeights(Scene *scene, int depth)
{
	if(depth == 0)
		return;
	
	{
		omp_set_num_threads(4);

#pragma omp parallel for
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

			mDecisionWeights[i] = weight;

			// For impact
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
			else
			{
				mDecisionWeights[i] = sceneCopy->getSceneScore();
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
