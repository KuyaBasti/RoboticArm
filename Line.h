#ifndef LINE_H
#define LINE_H 

#include "MasterHeader.h"
#include "MoveToPoint.h"

class Line {
private:
	MoveToPoint* RoboMovetoPoint = nullptr;
	double Length;                                     // Holds the length of travel
    void   setLength(PointXY Pt1, PointXY Pt2);        // Set length of line
    double UnitDistInc = 0.0;                          // Save step from last motion
    double  getDistIterator(PointXY Endpt, double Step);
	double  getAngleFromXY(PointXY Pt1, PointXY Pt2);  // Get angle from X
    double  getLength();                               // Return current length needed??

public:  // Once we get everything working, we will only have one method
	Line(float Length = 1){this->Length = Length;}     // Provide a default for new users
	void LineReset(){UnitDistInc = 0.0;}               // Reset UnitDistInc after line complete (modify??) changed to 0.2 from 0.0
	double  getLineSteps(double UnitStep);             // Get Number of steps for loop
	PointXY getNextPt(PointXY Endpt, PointXY OrigXY);                 // Get next point
	void setMovetoPoint(MoveToPoint* RoboMove) { RoboMovetoPoint = RoboMove; }
};

#endif