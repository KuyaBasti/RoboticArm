#ifndef MOVETOPOINT_H
#define MOVETOPOINT_H

#include "RhrinoSpecific.h"
#include "RhinoMan.h"
//#include "MasterHeader.h"

class MoveToPoint {
private:
    RhrinoSpecific RoboMotion;
    RhinoMath      RoboCalc;
    //MasterHeader      RoboCalc;

public:
    char commandType;
    bool MoveToXY(PointXY);
    bool MotoMoveServoFout();  //Move waist out (Quil off) Did not get 3-D model in 2022
    bool MotoMoveServoFin();   //Move waist in  (Quil on)
    //void setCommandTypeFromParse(char c);
    //double command_type_unit_step; // variable to hold unitstep based on command type 
};

#endif
