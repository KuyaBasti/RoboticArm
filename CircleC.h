#include <string>
#include "MasterHeader.h"
#include "MoveToPoint.h"
//struct PointXY
//{
//   double X,Y;
//};

class circle
{   //Note: C trig functions return radians.  I have converted to degrees for demonstration
	private:
      //tserial instance.
      MoveToPoint* RoboMovetoPoint = nullptr;
      double Angle1 = 0.0, Angle2 = 0.0, AngleDist = 0.0, Radius = 0.0;
      double Steps,Step, UnitDist;
	  double getAngleDist(){return AngleDist;}        //Return Ang Dist. between points
	  double PointAngle   (const  PointXY &P1);       //Return Angle based on XY point
      double getCirc(double Radius);                  //Return Circumf.
      double getAngleSteps(double UnitStep,  double Radius); //Get Num Steps around arc.
      PointXY PlotPoint{ 0.0 , 0.0 };
      char textch = ' ';
      double test = 0.0;
      int counter = 0;
      PointXY WorkingPtMv;
public:
	  double getAngle1(){return Angle1;}              //Return Angle1
      double getAngle2(){return Angle2;}              //Return Angle2
      double getStep(){return Step;}                  //Return UnitStep
      double AngleBWPoints(double Ang1, double Ang2); //Return Angular Distance
      void  initArc(const  PointXY &P1, const  PointXY &P2, const double UnitDistance); //Initialize Circle params
      PointXY  getXYAngleHyp(double Angle);             //Return XYPoint given an angle.
      void  DoitC(  PointXY &Pt1,   PointXY &Pt2, std::string dir, const double unitStep, PointXY &Cpt);
         //Add Center point param
      void setMovetoPoint(MoveToPoint* RoboMove) { RoboMovetoPoint = RoboMove; }
};
