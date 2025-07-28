#include <iostream>
#include <cmath>
#include <string>
//#include "RhinoMan.h"
#include <windows.h>
#include "RhrinoSpecific.h"
#include "MasterHeader.h"
using namespace std;

//                      Passing in('E'   ,       693.82)
bool RhrinoSpecific::MotoMoveServo(char c, int Distance)//  passing in(ex:10) gives 742 instead of 693

{

	MasterHeader MotorDistance;
	//Lines 17 - 34 are from work explained in class with Prof and Rebecca
	// 57    =    750   -    693

	if (c == 'E')
		Distance = TickAng1 - Distance;
	else
		Distance = TickAng2 - Distance;

	cout << "\nDistance for " << c << ": " << Distance << endl;

	double Distance1 = 0; //Initializing the tempvariable that holds the difference
	//double Distance2 = 0;
	double Difference = Distance;
	double dMotorDistance = 0;

	if (c == 'E')
	{
		Distance1 = Distance;//57 on first iteration
		TempDistanceE = Distance1;
		Difference = TickAng1 - Distance1;//693 on first
		//TickAng1 = Distance1;//57 on first
		Distance = Difference - Distance1;//693 on first
		TickAng1 = Difference;
		MotorDistance.eMotorDifference = Difference;//693 on first
	}
	//double eMotorDifference2 = MotorDistance.eMotorDifference;//693 on first

	if (c == 'D')
	{
		cout << "Start of D motor: " << endl;
		Distance1 = Distance;//-579 on first//should be 56
		Difference = TickAng2 - Distance1;
		TickAng2 = Difference;
		dMotorDistance = Difference - MotorDistance.eMotorDifference2;
		Distance = dMotorDistance;
		Distance1 -= TempDistanceE;
		//dMotorDifference = Difference;
	}


	//E motor = 57
	//D motor = 56 

	//113 - 57 = 56
	cout << "\nDistance for motor " << c << ": " << Distance1 << endl;

	//Add take distance and current tick count TickAng1 and TickAng2
	//and move only the difference betweeb TickAngs and current TickAngs
	//          1. Calc diffrence between where I am and where goint
	//             move only that difference TickAng1 and TickAng2
	//          2. Store the complete value as read to
	unitstepVal = 1;
	int count = Distance1 / unitstepVal;	/* hard code*/														//Rebecca changed this to a 2. from 50 for PTP// I changed to 1 for line
	int What = int(unitstepVal);
	int cntRem = abs(int(Distance1) % What);												    //Rebecca changed this to a 2.from 50 for PTP// I changed to 1 for line
	char Number[3];                     //Two characters plus the NULL terminator
	char direction;
	char* RobCmd = new char[80];       //C string cmd for Rhino
	string RoboCommand = "";            //Blank build string
	bool complete = false;

	//Assign the servo motor letter and direction
	RoboCommand += c;        // Assign motor
	direction = (Distance1 > 0) ? '+' : '-';
	RoboCommand += direction; // Assign directiom
	RoboCommand += "1";	//Needs to be 1 for line but 50 for point-to-point //Variable needed															//Rebecca changed this to a 2. from 50// I changed to 1
							   //strcpy(RobCmd, RoboCommand.c_str()); // Older Version
	strcpy_s(RobCmd, RoboCommand.length() + 1, RoboCommand.c_str());

	for (int cnt = 0; cnt < abs(count); cnt++)
	{ //Move by fixed number loop.
		Sleep(250);
		cout << " Move of  " << unitstepVal << RobCmd << endl;	//Needs to be 1 for line but 50 for point-to-point //Variable needed									//Rebecca changed this to a 2. form 50// I changed to 1
		robo << RobCmd;
		// Insert your call to the robot controller
	}

	//Create string to handle remainder
	RoboCommand = ""; //Clear string
	RoboCommand += c; //Assign motor
	RoboCommand += direction;//Assign direction
	//itoa(cntRem, Number, 10);//Convert remainder to char string ***Convert to
	_itoa_s(cntRem, Number, 3, 10);

	RoboCommand += Number; //Assign remainder as characters
						   //strcpy(RobCmd, RoboCommand.c_str()); // Older Version
	strcpy_s(RobCmd, RoboCommand.length() + 1, RoboCommand.c_str());
	Sleep(250);
	// Insert your call to the robot controller
	cout << "Move Remainder " << RobCmd << endl << endl;
	Sleep(250);
	return complete;
}