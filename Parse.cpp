#include "Parse.h"
#include <iostream>
#include <iomanip>
using namespace std;

void Parse::parseLine()
{
	unsigned LineLength;;     //Hold lengtgh of current line.
	PointXY Ptret;			 //Working point return
	char CharsIn[80] = {0};  //holds char array
 	CommandLine = "";        //blank out the string
	if(!GCODEin.eof() && GCODE != '8')    //Read if not end of file
	{
       GCODEin.getline(CharsIn, 80);
       for(unsigned i = 0; i < strlen(CharsIn); i++)
          CommandLine += CharsIn[i];
    }
	else
	  return;
	LineLength = CommandLine.length();

	//Collect G XYIJ values in each line
	for(index = 0; index < LineLength; index++)
	{
  	   //swtich statement to appropriate case, to get the 'G', 'X', 'Y', 'I', 'J', 'Z' 
  	   switch (CommandLine[index])
  	   {
		   case 'G' : getGODE();
		              break;

		   case 'X' : getX();
		              break;

		   case 'Y' : getY();
		              break;

		   case 'I' : getI();
		              break;

		   case 'J' : getJ();
		              break;

		   case 'Z' : getZ();
		              break;
	   }
    }


    //Display current G XYIJ values for next move Demo only
	cout << "G0" << GCODE << " X" << setw(7) << X << " Y" <<  setw(7) << Y
	                      << " I" << setw(7) << I << " J" <<  setw(7) << J ;
	//call line or circle given current and new XYIJ Values
	switch(GCODE)
	{
		case '0': //call line class and populate the Gcommand in the class
			/* 
				- calling MoveToPoint class to move motor out
				
			*/
  			RoboMovetoPoint.MotoMoveServoFout();
			std::cout << "Line:" << std::fixed << std::showpoint << std::setprecision(1) //FINAL STEP
				<< setw(7) << right << X
				<< " " << setw(7) << right << Y << std::endl;
			WorkingPtMv.X = X;     //Working variable based on current position X and Y
			WorkingPtMv.Y = Y;     //Locations stored in class
			RoboMovetoPoint.MoveToXY(WorkingPtMv); // call to move motor D and E

			/*
				- call to move to point clas,
				- function to move motor inwards
			*/
			RoboMovetoPoint.MotoMoveServoFin();
			CurrentX = X;
			CurrentY = Y;
			Current.X = CurrentX;  //Used by line.  Need to consolidate.
			Current.Y = CurrentY;  //Used by line.  Need to consolidate.
			break;
		case '1': //call line class and populate the Gcommand in the class
		          //call line method
			      LineTest.LineReset();
				  WorkingPt.X = X - Current.X;    /* <-- updating working point of struct PointXy in cirle clas		updating working Y cordinate in circle class ->>*/     WorkingPt.Y = Y - Current.Y;
				  for (unsigned traverse = 0; traverse < (LineTest.getLineSteps(0.2)); traverse++)
				  {
					  Ptret = LineTest.getNextPt(WorkingPt, Current); //Adjusted to 0.0 added heere
					  RoboMovetoPoint.MoveToXY(Ptret); // calling function to move 
					  std::cout <<"Line:" << showpos <<std::fixed << std::showpoint << std::setprecision(1)
						  << setw(7) << right <<Ptret.X  
						  << " " << setw(7) << right << Ptret.Y << std::endl;
				  }   //Validate and adjust for forward and backward
				  //Just goe to the final step.
				  //NOTE: !!!!!!!!!!!Create string to pass to movetoxy
				  WorkingPtMv.X = X;     //Final X and Y Target Destination
				  WorkingPtMv.Y = Y;				  
				  RoboMovetoPoint.MoveToXY(WorkingPtMv); // call to move motor d or e
				  CurrentX = WorkingPt.X + Current.X; // updating 
		          CurrentY = WorkingPt.Y + Current.Y;
				  Current.X = CurrentX;  //Used by line.  Need to consolidate.
				  Current.Y = CurrentY;  //Used by line.  Need to consolidate.
		          break;

		case '2':
		case '3': //call circle CW  and populate the Gcommand in the class
		          //Account for arc center position
                  //add feature to allow Center (i,j)
		          Current = {CurrentX, CurrentY};
		          cout << "Current X: "<< X << "  " << "Current Y: " << Y << endl;
		          Target  = {X, Y};
		          Center  = {I, J};
		          Direction = (GCODE == '2')? "CW" : "CCW";

				  TestCircle.DoitC(Current,Target,Direction,0.2, Center); //Add Center point param add tserail pointer
		          CurrentX = Target.X + Center.X; //Update current point  of X
		          CurrentY = Target.Y + Center.Y; // Update current point of Y
				  Current.X = CurrentX;  //Used by line.  Need to consolidate.
				  Current.Y = CurrentY;  //Used by line.  Need to consolidate.
		          break;
	}
}

//reading in I values, inrements to the next I value
void Parse::getI()
{
	string Temp = "";
	while(isdigit(CommandLine[++index])|| CommandLine[index] == '.'||CommandLine[index] == '-')
       Temp.append(1, CommandLine[index]); //Append characters to a temp string
	I = atof(Temp.c_str());
	--index; // decrement index 
}

//reading in J values, inrements to the next J value
void Parse::getJ()
{
	string Temp = ""; // string to hold temperary 
	// while loop to check for digits, if a digit is found store the command line to string Temp
	while (isdigit(CommandLine[++index]) || CommandLine[index] == '.' || CommandLine[index] == '-') {
		Temp.append(1, CommandLine[index]);
	}
       
	J = atof(Temp.c_str());  // convert string to a float data type 
	--index; // decrement index 
}

//reading in Y values, inrements to the next Y value
void Parse::getY()
{
	string Temp = "";
	while (isdigit(CommandLine[++index]) || CommandLine[index] == '.' || CommandLine[index] == '-') {
		Temp.append(1, CommandLine[index]);
	}
      
	Y = atof(Temp.c_str());
	--index; // decrement index 
}

//reading in X values, inrements to the next X value
void Parse::getX()
{
	string Temp = "";
	while(isdigit(CommandLine[++index])|| CommandLine[index] == '.'||CommandLine[index] == '-')
       Temp.append(1, CommandLine[index]);
	X = atof(Temp.c_str());
	--index; // decrement index 
}

void Parse::getZ()
{
	string Temp = "";
	while(isdigit(CommandLine[++index])|| CommandLine[index] == '.'||CommandLine[index] == '-')
       Temp.append(1, CommandLine[index]);
	Z = atof(Temp.c_str());
	--index; // decrement index 
}

// to get GCode
void Parse::getGODE()
{
	GCODE = CommandLine[2]; //Grab the actual charcter for the G0,1,2,or 3
	index = 2;  // setting index value to 2
}

