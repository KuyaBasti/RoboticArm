#include "MoveToPoint.h"
#include "RhinoMath.h"

bool MoveToPoint::MoveToXY(PointXY XYPointNew)
{
	bool Didit;
	AnglePair CurrentAngs;
	CurrentAngs = RoboCalc.getAngpair(XYPointNew);  //Result in degrees
	
	Didit = RoboMotion.MotoMoveServo('E', int(CurrentAngs.Ang1 / .12)); // was not an int previously
	Didit = RoboMotion.MotoMoveServo('D', int(CurrentAngs.Ang2 / .12));
	return true;
}


//Hard coded Quil on/off
bool MoveToPoint::MotoMoveServoFout()
{
	RoboMotion.MotoMoveServo('F', -20);
	return true;
}

bool MoveToPoint::MotoMoveServoFin()
{
	RoboMotion.MotoMoveServo('F', 20);
	return true;
}