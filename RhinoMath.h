#ifndef RHINOMAN_H
#define RHINOMAN_H

#include "MasterHeader.h"
struct AnglePair
{
	double Ang1 = 0.0; //After calculations are completed CompAng1(), this var holds angle 1 in degrees
	double Ang2 = 0.0; //After calculations are completed CompAng2(), this var holds angle 2 in degrees
};

class MasterHeader
{
private:
	double ArmA;
	double ArmB;
    double CompAng2(double, double);          //make private add point pair
	double CompAng1(double, double, double);  //make private

public:
	double eMotorDifference = 0;
	double eMotorDifference2 = 0;
	//MasterHeader(double = 9.0, double = 9.0);
	AngPair getAngpair(PointXY);           //make public// was PointXY
};
#endif