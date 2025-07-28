#define _USE_MATH_DEFINES
#include <cmath>
#include "Line.h"
#include "MasterHeader.h"
//Constructor stores a default length for maint. crew
double Line::getLength() { return Length; }

//Calculates the number of steps given a unit distance
//segment the line into small steps
double  Line::getLineSteps(double UnitStep)
{
	return int(Length / UnitStep);
}

//Calculates angle for segment moves
double Line::getAngleFromXY(PointXY Pt1, PointXY Pt2)
{
	double Rads = atan((Pt2.X - Pt1.X) / (Pt2.Y - Pt1.Y));
	return (180 / M_PI) * Rads;  //Convert to degrees
}

//Returns a lenght and comp. for each quadrant.
void   Line::setLength(PointXY Pt1, PointXY Pt2) //see page 153
{
	double XLen;
	double YLen;
	if (Pt1.X < 0)
		if (Pt2.X < 0)
			XLen = Pt1.X - Pt2.X;
		else
			XLen = -Pt1.X + Pt2.X;
	else
		if (Pt2.X < 0)
			XLen = Pt1.X + -Pt2.X;
		else
			XLen = Pt1.X - Pt2.X;

	if (Pt1.Y < 0) //neg

		if (Pt2.Y < 0)
			YLen = Pt1.Y + Pt2.Y;
		else
			YLen = -Pt1.Y - Pt2.Y;
	else
		if (Pt2.Y < 0)
			YLen = Pt1.Y + -Pt2.Y;
		else
			YLen = Pt1.Y - Pt2.Y;
	Length = sqrt(pow((XLen), 2) + pow((YLen), 2));
}

PointXY Line::getNextPt(PointXY Endpt, PointXY OrigXY)
{
	PointXY  OrigXYZ = {0,0};                     //set to 0,0 for simplicity, changed to 9,9

	UnitDistInc += getDistIterator(Endpt, 0.2);  //updates the diestance iterator, changed to 0.2 was originally 0.5
	double Angle = getAngleFromXY(OrigXYZ, Endpt);//sets line angle
	setLength(OrigXYZ, Endpt);                    //Stores the length

	PointXY ReturnXY; //Working point for calculation
	ReturnXY.Y = (UnitDistInc * cos(Angle * (M_PI / 180))) + OrigXY.Y;
	ReturnXY.X = (UnitDistInc * sin(Angle * (M_PI / 180))) + OrigXY.X;
	return ReturnXY;
}

double Line::getDistIterator(PointXY Endpt, double Step)
{
	return ((Endpt.X >= 0 && Endpt.Y >= 0) || (Endpt.X < 0 && Endpt.Y > 0)) ?  Step :  -Step;
}