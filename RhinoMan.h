#ifndef RHINOMAN_H
#define RHINOMAN_H
#include "MasterHeader.h"
struct AnglePair
{
	double Ang1 = 0.0; //After calculations are completed CompAng1(), this var holds angle 1 in degrees
	double Ang2 = 0.0; //After calculations are completed CompAng2(), this var holds angle 2 in degrees
};

class RhinoMath
{
private:
	double ArmA; //Arm A  9.0 default
	double ArmB; //Arm B  9.0 default
	double CompAng2(double x, double y);             //Should take in point pair
	double CompAng1(double x, double y, double ang2);//Should take in point pair and angle 2
public:
	RhinoMath(double = 9.0, double = 9.0);
	AnglePair getAngpair(PointXY);
};
#endif
