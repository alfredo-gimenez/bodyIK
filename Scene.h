#pragma once

#include <vector>

#include "DecisionTree.h"
#include "Body.h"

#define NUM_DECISIONS 64
#define DECISION_DEPTH 1
#define MOTION_ITERS 500
#define LOOKAHEAD 1000

class Decision;

class Scene
{
public:
	Scene();
	Scene(Scene *scene);
	~Scene();

	void update();
	void drawGL();

	inline unsigned int getFrameCount() const
		{ return mFrameCount; }

	double getSceneScore();
	void makeDecision(Decision *dec);

private:
	unsigned int mFrameCount;

	// All custom scene functions
	void calcMotion();
	void calcPhysics();

	// All custom scene variables
	Body *mBody;
	std::vector<EllipseObject*> testProjectiles;
};

