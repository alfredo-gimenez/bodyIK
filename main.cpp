#include <Windows.h>

#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

#include "glut.h"
#include "glui.h"

#include "Scene.h"
#include "PhyObject.h"
#include "Body.h"
#include "DecisionTree.h"
#include "Ellipse.h"

int	main_window;

// the camera info  
float eye[3];
float lookat[3];

// walking action variables
//
double gX, gY;

int winWidth,winHeight;     // window (x,y) size

double seye[3] = {0.0,0.0,10};
double at[3]  = {0.0,0.0,0};
double up[3]  = {0.0,1.0,0.0};

Scene *mScene;
DecisionTree *mDecisions;

//////////////////////////////////////////////////////////////////////
//
// Set all variables to their default values.
//
//////////////////////////////////////////////////////////////////////

void reset() 
{
	winWidth = 1024;
	winHeight = 1024;

	// Create Scene
	mScene = new Scene();

	// Create decision tree
	mDecisions = new DecisionTree();
}

void setCamera() 
{
	glViewport(0,0, winWidth,winHeight);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-winWidth/2.0,winWidth/2.0,-winHeight/2.0,winHeight/2.0,0.1,100);
}

//////////////////////////////////////////////////////////////////////////////
//
//  drawArm()
//
//////////////////////////////////////////////////////////////////////////////

void myGlutIdle(void)
{
	// make sure the main window is active
	if (glutGetWindow() != main_window)
		glutSetWindow(main_window);

	// just keep redrawing the scene over and over
	glutPostRedisplay();
}


// mouse handling functions for the main window
// keep track of which button is down and where the last position was
int cur_button = -1;
int last_x;
int last_y;

// catch mouse up/down events
void myGlutMouse(int button, int state, int x, int y)
{
	if (state == GLUT_DOWN)
		cur_button = button;
	else
		if (button == cur_button)
			cur_button = -1;

	last_x = x;
	last_y = y;

	gX = (float)(x) - (winWidth/2.0);
	gY = -((float)(y) - (winHeight/2.0));

	// Leave the following call in place.  It tells GLUT that
	// we've done something, and that the window needs to be
	// re-drawn.  GLUT will call display().
	//
	glutPostRedisplay();
}

// catch mouse move events
void myGlutMotion(int x, int y)
{
	// the change in mouse position
	int dx = x-last_x;
	int dy = y-last_y;

	switch(cur_button)
	{
	case GLUT_LEFT_BUTTON:

		break;

	case GLUT_MIDDLE_BUTTON:
		break;

	case GLUT_RIGHT_BUTTON:

		break;
	}

	last_x = x;
	last_y = y;

	glutPostRedisplay();
}

// you can put keyboard shortcuts in here
void myGlutKeyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
		// quit
	case 27: 
	case 'q':
	case 'Q':
		exit(0);
		break;
	}

	glutPostRedisplay();
}

// the window has changed shapes, fix ourselves up
void myGlutReshape(int	x, int y)
{
	int tx, ty, tw, th;
	GLUI_Master.get_viewport_area(&tx, &ty, &tw, &th);
	glViewport(tx, ty, tw, th);

	glutPostRedisplay();
}

void screenShot(const char *prefix);

// draw the scene
void myGlutDisplay(	void )
{
	setCamera();

	// Set the background colour to dark gray.
	glClearColor(.90f,.90f, .9f,1.f);

	// OK, now clear the screen with the background colour
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Position the camera eye and look-at point.
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(seye[0],seye[1],seye[2],  at[0],at[1],at[2],  up[0],up[1],up[2]);

	// Draw here
	mScene->drawGL();
	
	// Execute any GL functions that are in the queue.
	glFlush();

	// Now, show the frame buffer that we just drew into.
	glutSwapBuffers();

	screenShot("C:/Users/chai/Desktop/frames/screen_");
}

#define DECISION_FREQ LOOKAHEAD

void calcDecisions()
{
	// create new decisions
	if(mScene->getFrameCount() % (DECISION_DEPTH*DECISION_FREQ) == 0)
	{
		mDecisions->populateDecisionTree(0,DECISION_DEPTH);
		mDecisions->calculateDecisionWeights(mScene);
	}

	// Make next decision
	if(mScene->getFrameCount() % DECISION_FREQ == 0)
	{
		Decision *nextDecision = mDecisions->makeNextDecision();
		mScene->makeDecision(nextDecision);
	}
}

#include "FreeImage.h"
#include <sstream>
#include <iomanip>

void screenShot(const char *prefix)
{
	// Make the BYTE array, factor of 3 because it's RBG.
	int size = 3 * winWidth * winHeight;
	BYTE* pixels = new BYTE[size];

	glReadPixels(0, 0, winWidth, winHeight, GL_RGB, GL_UNSIGNED_BYTE, pixels);

	std::ostringstream filenameSS;
	filenameSS << prefix;
	filenameSS << std::setfill('0') << std::setw(4) << mScene->getFrameCount();
	filenameSS << ".bmp";

	std::string filename = filenameSS.str();
	std::cout << filename << std::endl;

	// Convert to FreeImage format & save to file
	FIBITMAP* image = FreeImage_ConvertFromRawBits(
		pixels, winWidth, winHeight, 3 * winWidth, 24, 
		0xFF0000, 0x00FF00, 0x0000FF, false);
	FreeImage_Save(FIF_BMP, image, filename.c_str(), 0);

	// Free resources
	FreeImage_Unload(image);

	delete [] pixels;
}

void myGlutTimer(int t)
{
	// Decide and execute decision
	calcDecisions();

	// Update the scene
	mScene->update();

	// how much damage?
	double score = mScene->getSceneScore();

	// Reset timer
	glutTimerFunc(1, myGlutTimer, 0);
}

// some controls generate a callback when they are changed
void glui_cb(int control)
{
	switch(control)
	{
		break;
	}

	glutPostRedisplay();
}

// entry point
int main(int argc, char* argv[])
{

	// Initialize
	reset();

	// Initialize the GLUT window.  We want a double-buffered window,
	// with R,G,B and alpha per pixel, and the depth buffer (z-buffer)
	// enabled.
	//
	glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	//
	// Put the window at the top left corner of the screen.
	//
	glutInitWindowPosition (0, 0);
	//
	// Specify the window dimensions.
	//
	glutInitWindowSize(winWidth,winHeight);
	//
	// And now create the window.
	main_window = glutCreateWindow("IK Canvas");

	//
	// set callbacks
	//
	glutDisplayFunc(myGlutDisplay);
	glutTimerFunc(100, myGlutTimer, 0);
	GLUI_Master.set_glutReshapeFunc(myGlutReshape);
	GLUI_Master.set_glutIdleFunc(myGlutIdle);
	GLUI_Master.set_glutKeyboardFunc(myGlutKeyboard);
	GLUI_Master.set_glutMouseFunc(myGlutMouse);
	glutMotionFunc(myGlutMotion);

	// initialize gl
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_COLOR_MATERIAL);
	glEnable(GL_POINT_SMOOTH);
	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);
	glEnable(GL_POLYGON_SMOOTH);
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	// give control over to glut
	glutMainLoop();

	return 0;
}























