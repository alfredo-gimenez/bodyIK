#include "IKchain.h"

#include "glut.h"
#include "glui.h"

#define _USE_MATH_DEFINES
#include <math.h>
#include <algorithm>

IKchain::IKchain()
	:
	mOrigin(new vec2D(0,0))
{
	mPositions.push_back(*mOrigin);
}

IKchain::IKchain(vec2D *origin)
	:
	mOrigin(origin)
{
	mPositions.push_back(*mOrigin);
}

IKchain::~IKchain()
{
}

void IKchain::calcSinesCosines(double *sines, double *cosines)
{
	unsigned int M = numSegments();

	for(unsigned int i=0; i<M; i++)
	{
		double sumtheta = 0;
		for(int j=i; j>=0; j--)
		{
			sumtheta += mSegments[j].mTheta;
		}

		cosines[i] = cos(sumtheta);
		sines[i] = sin(sumtheta);
	}
}

void IKchain::calcFK()
{
	unsigned int M = numSegments();

	// Calculate end positions using forward kinematics
	double *costhetas = new double[M];
	double *sinthetas = new double[M];
	calcSinesCosines(sinthetas,costhetas);

	double x = mOrigin->x();
	double y = mOrigin->y();

	for(unsigned int i=0; i<M; i++)
	{
		mPositions[i].x() = x;
		mPositions[i].y() = y;

		x += mSegments[i].mLength * costhetas[i];
		y += mSegments[i].mLength * sinthetas[i];
	}

	mPositions[M].x() = x;
	mPositions[M].y() = y;

	delete [] costhetas;
	delete [] sinthetas;
}

double IKchain::calcIK()
{
	unsigned int M = numSegments();

	// Do we need to do a step?
	vec2D errorVec = mGoal - mPositions[M];
	if(errorVec.length() < MAX_IK_DISTANCE)
	{
		calcFK();
		return errorVec.length();
	}

	// Is the goal past our farthest reach?
	vec2D goalVec = (mGoal - *mOrigin);
	double len = totalLength();
	if(goalVec.length() > len)
	{
		// A dot B = ABcos(theta)
		double tmp = goalVec.x() / (goalVec.length());
		double theta = acos(tmp);

		double sign = (goalVec.y() > 0) ? +1 : -1;

		mSegments[0].mTheta = sign*theta;
		for(int i=1; i<M; i++)
		{
			mSegments[i].mTheta = 0;
		}

		calcFK();
		return 0;
	}

	// Do IK step
	int i=0;
	while(errorVec.length() > MAX_IK_DISTANCE && i < MAX_IK_ITERS)
	{
		doIKStep();
		calcFK();
		errorVec = mGoal - mPositions[M];
		i++;
	}

	return errorVec.length();
}

#define ROW_MAJOR(x,y,w) x*w+y
inline double clamp(double v,double x,double y)
{
	if(v<x)
	{
		return x;
	}
	else if(v>y)
	{
		return y;
	}
	else 
		return v;
}

void IKchain::calcJacobian(double *out)
{
	unsigned int M = numSegments();

	// Get sines and cosines of sums of angles
	double *costhetas = new double[M];
	double *sinthetas = new double[M];
	calcSinesCosines(sinthetas,costhetas);

	// Create the jacobian
	memset(out,0,sizeof(double)*2*M);
	for(unsigned int i=0; i<M; i++)
	{
		out[ROW_MAJOR(0,i,M)] += -1.0 * mSegments[i].mLength * sinthetas[i];
		out[ROW_MAJOR(1,i,M)] += mSegments[i].mLength * costhetas[i];
	}

	delete [] costhetas;
	delete [] sinthetas;
}

void IKchain::calcPseudoInverse(double *J, double *out)
{
	unsigned int M = numSegments();

	// Calculate J * J_transpose
	double JJT[4];
	memset(JJT,0,sizeof(double)*4);
	for(unsigned int i=0; i<M; i++)
	{
		// Dot products of rows
		JJT[ROW_MAJOR(0,0,2)] += J[ROW_MAJOR(0,i,M)] * J[ROW_MAJOR(0,i,M)];
		JJT[ROW_MAJOR(0,1,2)] += J[ROW_MAJOR(0,i,M)] * J[ROW_MAJOR(1,i,M)];
		JJT[ROW_MAJOR(1,0,2)] += J[ROW_MAJOR(1,i,M)] * J[ROW_MAJOR(0,i,M)];
		JJT[ROW_MAJOR(1,1,2)] += J[ROW_MAJOR(1,i,M)] * J[ROW_MAJOR(1,i,M)];
	}

	// Calculate the inverse of JJT using adjugate
	// For a square matrix A, the inverse is equal to
	// 1/det(A) * adj(A)
	// http://www.cs.rochester.edu/~brown/Crypto/assts/projects/adj.html

	// Determinant 2x2
	double det_JJT = JJT[ROW_MAJOR(0,0,2)] * JJT[ROW_MAJOR(1,1,2)] - 
		JJT[ROW_MAJOR(1,0,2)] * JJT[ROW_MAJOR(0,1,2)];

	// 1/det
	double rcp_det_JJT = 1.0/std::max(det_JJT,0.000000001);

	// Inverse = adj2x2(M) * (1/det)
	double inverse_JJT[4];
	inverse_JJT[ROW_MAJOR(0,0,2)] =  JJT[ROW_MAJOR(1,1,2)] * rcp_det_JJT;
	inverse_JJT[ROW_MAJOR(0,1,2)] = -JJT[ROW_MAJOR(0,1,2)] * rcp_det_JJT;
	inverse_JJT[ROW_MAJOR(1,0,2)] = -JJT[ROW_MAJOR(1,0,2)] * rcp_det_JJT;
	inverse_JJT[ROW_MAJOR(1,1,2)] =  JJT[ROW_MAJOR(0,0,2)] * rcp_det_JJT;

	// Finally, J_Transpose * inverse_JJT
	// NOTE: using J instead of transpose, indexes reversed
	memset(out,0,sizeof(double)*2*M);
	for(unsigned int m=0; m<M; m++)
		for(unsigned int n=0; n<2; n++)
			for(unsigned int k=0; k<2; k++)
				out[ROW_MAJOR(m,n,2)] += J[ROW_MAJOR(k,m,M)] * inverse_JJT[ROW_MAJOR(k,n,2)];
}

double 
	IKchain::calcError(double dpX, double dpY, double *J, double *JPI)
{
	unsigned int M = numSegments();

	// Calculate J*JPI
	double JJPI[4];
	memset(JJPI,0,sizeof(double)*4);
	for(unsigned int m=0; m<2; m++)
		for(unsigned int n=0; n<2; n++)
			for(unsigned int k=0; k<M; k++)
				JJPI[ROW_MAJOR(m,n,2)] += J[ROW_MAJOR(m,k,M)] * JPI[ROW_MAJOR(k,n,2)];

	// Calculate I-JJPI
	double I_JJPI[4];
	I_JJPI[ROW_MAJOR(0,0,2)] = 1.0 - JJPI[ROW_MAJOR(0,0,2)];
	I_JJPI[ROW_MAJOR(0,1,2)] = 0.0 - JJPI[ROW_MAJOR(0,1,2)];
	I_JJPI[ROW_MAJOR(1,0,2)] = 0.0 - JJPI[ROW_MAJOR(1,0,2)];
	I_JJPI[ROW_MAJOR(1,1,2)] = 1.0 - JJPI[ROW_MAJOR(1,1,2)];

	// Multiply by dP
	double DP_I_JJPI[2];
	DP_I_JJPI[ROW_MAJOR(0,0,1)] = I_JJPI[ROW_MAJOR(0,0,2)] * dpX + I_JJPI[ROW_MAJOR(0,1,2)] * dpY;
	DP_I_JJPI[ROW_MAJOR(0,1,1)] = I_JJPI[ROW_MAJOR(1,0,2)] * dpX + I_JJPI[ROW_MAJOR(1,1,2)] * dpY;

	// Calculate 2-norm of I_JJPI
	double error = DP_I_JJPI[0]*DP_I_JJPI[0] + DP_I_JJPI[1]*DP_I_JJPI[1];

	return error;
}

double IKchain::doIKStep()
{
	unsigned int M = numSegments();
	double gX = mGoal.x();
	double gY = mGoal.y();

	// Calculate G - X
	double eX = DEFAULT_MAG * (gX - mPositions[M].x());
	double eY = DEFAULT_MAG * (gY - mPositions[M].y());
	
	// Set up variables
	double *J = new double[2*M];
	double *JPI = new double[2*M];
	double error = MAX_ERROR+1;

	while(1)
	{
		// Calculate Jacobian 
		calcJacobian(J);

		// Calculate Pseudo Inverse of Jacobian 
		calcPseudoInverse(J,JPI);

		// Calculate error
		error = calcError(eX,eY,J,JPI);

		// Return dX length if done
		if(error < MAX_ERROR)
			break;

		// Halve dX to continue
		eX /= 2.0;
		eY /= 2.0;
	}

	double delta1,delta2;
	double newTheta;
	for(unsigned int i=0; i<M; i++)
	{
		delta1 = JPI[ROW_MAJOR(i,0,2)] * eX;
		delta2 = JPI[ROW_MAJOR(i,1,2)] * eY;

		newTheta = mSegments[i].mTheta + delta1 + delta2;

		// NO CONSTRAINTS
		//mSegments[i].mTheta = newTheta;
		mSegments[i].mTheta = clamp(newTheta,
									mSegments[i].mMinTheta,
									mSegments[i].mMaxTheta);
	}

	delete [] J;
	delete [] JPI;

	return error;
}

void IKchain::drawGL() 
{
	unsigned int M = numSegments();

	vec2D ellipseCenter;

	// Draw arm
	for(unsigned int i=0; i<M; i++)
	{
		mBodyParts[i].drawGL();
	}

	// Draw pins
	glPointSize(5.0);
	glBegin(GL_POINTS);
	{
		glColor3f(0,0,1);
		for(unsigned int i=0; i<M; i++)
		{
			glVertex3f(mPositions[i].x(),mPositions[i].y(),0);
		}
	}
	glEnd();

	glPointSize(10);
	glColor3f(1.0,0,0);
	glBegin(GL_POINTS);
		glVertex2f(mGoal.x(),mGoal.y());
	glEnd();
}

bool IKchain::collide(EllipseObject* ellipse)
{
	bool collision=false;
	unsigned int M = numSegments();
	for(unsigned int i=0; i<M; i++)
	{
		collision = mBodyParts[i].collide(ellipse);
	}
	return collision;
}

void IKchain::moveBodyParts()
{
	unsigned int M = numSegments();

	vec2D ellipseCenter;
	double totalRotation = 0;

	for(unsigned int i=0; i<M; i++)
	{
		ellipseCenter = (mPositions[i] + mPositions[i+1]) * 0.5;
		totalRotation += mSegments[i].mTheta;

		mBodyParts[i].moveTo(ellipseCenter);
		mBodyParts[i].rotateTo(totalRotation);
	}
}
