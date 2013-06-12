#include "Scene.h"

#include <algorithm>

Scene::Scene()
{
	mFrameCount = 0;

	// Create body
	mBody = new Body();
	//mBody->accelerate(vec2D(0,G_ACC));

	// Create ground and walls
	EllipseObject *newEllipse;
	newEllipse = new EllipseObject(vec2D(0,-195),100000,5,0);
	testProjectiles.push_back(newEllipse);
	testProjectiles.back()->immobile = true;

	newEllipse = new EllipseObject(vec2D(400,650),5,840,0);
	testProjectiles.push_back(newEllipse);
	testProjectiles.back()->immobile = true;

	double y = -20;
	for(int i=0; i<7; i++)
	{
		newEllipse = new EllipseObject(vec2D(-300,y),20,10,5,0);
		testProjectiles.push_back(newEllipse);
		testProjectiles.back()->immobile = true;

		y -= 25;
	}

	// Create projectiles!
	EllipseObject *testObj;
	testObj =new EllipseObject(vec2D(-500,10),50,25,2);
	testObj->accelerate(vec2D(0,G_ACC));
	testObj->speedupTo(vec2D(0.8,0.2));
	testProjectiles.push_back(testObj);

	//testObj =new EllipseObject(vec2D(300,160),100,50,2);
	//testObj->accelerate(vec2D(0,G_ACC));
	//testObj->speedupTo(vec2D(1,0));
	//testProjectiles.push_back(testObj);

	//testObj =new EllipseObject(vec2D(-200,140),30,15,2);
	//testObj->accelerate(vec2D(0,G_ACC));
	//testObj->speedupTo(vec2D(2,0));
	//testProjectiles.push_back(testObj);

	//testObj =new EllipseObject(vec2D(200,140),30,15,2);
	//testObj->accelerate(vec2D(0,G_ACC));
	//testObj->speedupTo(vec2D(-2,0));
	//testProjectiles.push_back(testObj);
}

Scene::Scene(Scene *scene)
{
	// Copy the body
	mBody = new Body(scene->mBody);

	// Copy the projectiles
	testProjectiles.resize(scene->testProjectiles.size());
	for(int i=0; i<testProjectiles.size(); i++)
	{
		testProjectiles[i] = new EllipseObject(scene->testProjectiles[i]);
	}
	
	// Copy the framecount
	mFrameCount = scene->mFrameCount;
}

Scene::~Scene()
{
	delete mBody;
	for(int i=0; i<testProjectiles.size(); i++)
	{
		delete testProjectiles[i];
	}
}

void Scene::update()
{
	calcMotion();
	calcPhysics();

	mFrameCount++;
}

void Scene::drawGL()
{
	// Draw the body
	mBody->drawGL();

	// Draw the projectiles
	for(int x=0;x<testProjectiles.size();x++)
	{
		testProjectiles[x]->drawGL();
	}
}

double Scene::getSceneScore()
{
	//return mBody->assessDamage();
	return testProjectiles.back()->getPos().x();
}

void Scene::makeDecision(Decision *dec)
{
	mBody->makeDecision(dec);
}

void Scene::calcMotion()
{
	// Update movement
	mBody->update();

	for(int x=0;x<testProjectiles.size();x++)
	{
		testProjectiles[x]->update();
	}
}

void Scene::calcPhysics()
{
	// Do collisions and new motion vectors
	for(int x=0;x<testProjectiles.size();x++)
		mBody->collide(testProjectiles[x]);
	
	for(int x=0;x<testProjectiles.size();x++)
		for(int y=0;y<testProjectiles.size();y++)
			if(x !=y )
				testProjectiles[x]->collide(testProjectiles[y]);
}
