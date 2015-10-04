//
// Copyright (c) 2015 Mahyar Khayatkhoei
// Copyright (c) 2009-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	//================DELETE THIS PART AND THEN START CODING===================
	static bool flag = false;
	if (!flag)
	{
		std::cerr << "ERROR>>>>Member function drawCurve is not implemented!" << std::endl;
		flag = true;
	}
	//=========================================================================

	// Robustness: make sure there is at least two control point: start and end points
	if (checkRobust() == false) {
		return;
	}
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points



	return;
#endif
}

//feed into std::sort to compare ControlPoints
bool compareControlPoints(CurvePoint a, CurvePoint b)
{
	return (a.time < b.time);
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	std::sort(Curve::controlPoints.begin(), Curve::controlPoints.end(), compareControlPoints);
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end point
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;
	float normalTime, intervalTime;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	if (getControPoints().size() < 2) {
		//std::cerr << "ERROR>>>> Curve not robust!" << std::endl;
		return false;
	}
	else {
		//std::cerr << "SUCCESS>>>> Curve is robust!" << std::endl;
		return true;
	}
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	CurvePoint currPoint;
	for (int i = 1; i < controlPoints.size(); i++)
	{
		currPoint = controlPoints[i];
		if (time < currPoint.time)
		{
			//Found it. return the current index back
			nextPoint = i;
			return true;
		}
	}
	//Not found
	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	// Calculate time interval, and normal time required for later curve calculations
	// Should findTimeInterval be used here? error with unsigned int and const unsigned int Params
	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time;
	normalTime = (time - controlPoints[nextPoint - 1].time) / intervalTime;

	// Calculate position at t = time on Hermite curve
	/*
	Hermite curve insites from slides:
	f1(t) = 2t^3 - 3t^2 +1
	f2(t) = -2t^3 + 3t^2
	f3(t) = t^3 - 2t^2 +t
	f4(t) = t^3-t^2

	position = p0.pos * f1(t) + p1.pos f2(t) + p0.tan*f3(t) + p1.tan*f4(t)
	*/
	float f1, f2, f3, f4, t2, t3;
	t2 = normalTime * normalTime;
	t3 = normalTime * normalTime * normalTime;

	f1 = (2 * t3) - (3 * t2) + 1;
	f2 = (-2 * t3) + 3 * t2;
	f3 = t3 - 2 * t2 + normalTime;
	f4 = t3 - t2;

	newPosition = f1*controlPoints[nextPoint - 1].position + f2*controlPoints[nextPoint].position
		+ f3*controlPoints[nextPoint - 1].tangent + f4*controlPoints[nextPoint].tangent;

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;

	// Calculate time interval, and normal time required for later curve calculations
	float normalTime, intervalTime;
	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint - 1].time;
	normalTime = (time - controlPoints[nextPoint - 1].time) / intervalTime;

	// Calculate position at t = time on Catmull-Rom curve
	/*
	Hermite curve insites from slides:
	f1(t) = 2t^3 - 3t^2 +1
	f2(t) = -2t^3 + 3t^2
	f3(t) = t^3 - 2t^2 +t
	f4(t) = t^3-t^2

	position = p0.pos * f1(t) + p1.pos f2(t) + p0.tan*f3(t) + p1.tan*f4(t)
	*/
	float f1, f2, f3, f4, t2, t3;
	t2 = normalTime * normalTime;
	t3 = normalTime * normalTime * normalTime;

	f1 = (2 * t3) - (3 * t2) + 1;
	f2 = (-2 * t3) + 3 * t2;
	f3 = t3 - 2 * t2 + normalTime;
	f4 = t3 - t2;

	/*
	Catmull-Rom curves are a subset of cardinal splines, which are a subset of hermite splines
	Cardinal-Roms use
	*/
	/*
	Which to use???
	first order tangent intuition from slides:
	si = yi+1 -yi
	sn-1 = yn-1 -yn-2

	This one I think??
	Second Order accurate tangents intuition from slides:
	s0 = 2*(y1 - y0) - (y2-y0)/2
	si = (yi+1 - yi-1)/(2)

	else:
	Ti = 0.5 * (Pi+1 - Pi-1)
	*/
	Vector s0, s1;

	if (nextPoint == 1)
	{
		//For the first 3 points
		s0 = 2 * (controlPoints[nextPoint].position - controlPoints[nextPoint - 1].position)
			- (controlPoints[nextPoint + 1].position - controlPoints[nextPoint - 1].position) / 2;
		s1 = (controlPoints[nextPoint + 1].position - controlPoints[nextPoint - 1].position) / 2;
	}
	else if (nextPoint == getControPoints().size() - 1)
	{
		//A reverse case of the above edge case to account for the last 3 points
		s0 = (controlPoints[nextPoint].position - controlPoints[nextPoint - 2].position) / 2;
		s1 = (controlPoints[nextPoint].position - controlPoints[nextPoint - 2].position) / 2
			- 2 * (controlPoints[nextPoint - 1].position - controlPoints[nextPoint].position);
	}
	else
	{
		//Any other non edge case set of points
		s0 = (controlPoints[nextPoint].position - controlPoints[nextPoint - 2].position) / 2;
		s1 = (controlPoints[nextPoint + 1].position - controlPoints[nextPoint - 1].position) / 2;
	}

	newPosition = f1*controlPoints[nextPoint - 1].position + f2*controlPoints[nextPoint].position
		+ f3*s0 + f4*s1;
	// Return result
	return newPosition;
}
