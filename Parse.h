#include "MoveToPoint.h"
#include "circleC.h"
#include "line.h"  //add your line code
#include <fstream>
using namespace std;

class Parse
{
private:
    //tserial *robo;Moveto lives in class movetoclass
    //Make instance of moveto class
    MoveToPoint RoboMovetoPoint; // new object of MoveToPoint Class
    MoveToPoint* RMPtr = &RoboMovetoPoint; // pointer to MoveToPoint class 
    PointXY Current = { 0,0 };  // struct in MasterHeader.h being called through the circle class, used for holding current point
    PointXY Target = { 0,0 };  // struct in MasterHeader.h being called through the circle class, used for holding target point
    PointXY Center = { 0,0 }; // struct in MasterHeader.h being called through the circle class, used for holding center point
    PointXY  WorkingPtMv = { 0,0 }; // struct in MasterHeader.h being called through the circle class, used for holding working point(s)
    PointXY WorkingPt = { 0,0 };
    PointXY WorkingCur = { 0,0 };
    ifstream GCODEin;                   //GCode input file
    char     GCODE = ' ';               //Store Current G command
    double  CurrentX = 9.0, CurrentY = 9.0; //Store currentbines. Changed currents to 0.0
    double   X = 0.0, Y = 0.0, I = 0.0, J = 0.0, Z = 0.0;//Store new target position
    unsigned index = 0;     //Index to individual command ln.
    circle TestCircle;  //Instance of circle class ,Pass robot tserial to TestCircle by reference
    Line    LineTest;
    //line   TestLine;    //Make instance of your line code pass robot tseral by refernce
    bool QuilOutFlag = false; //Assume pen on white board
    string CommandLine = "";  //Store individual command lines.
    string Direction;  
    char textch;
public:
    Parse(string nameFile){
        GCODEin.open(nameFile);
        TestCircle.setMovetoPoint(RMPtr);
}

   // Parse(string nameFile){GCODEin.open(nameFile);}  //Use new to create tserial, then Add connect
                                                     //Open the tserail and make instance
    
    void parseLine();
    void getGODE();
    void getX();
    void getY();
    void getI();
    void getJ();
    void getZ();

    ~Parse() { GCODEin.close(); } //Close tseral, destructor 
};