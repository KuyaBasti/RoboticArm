#ifndef RHRINOSPECIFIC_H
#define RHRINOSPECIFIC_H

#include "tserial.h"

class RhrinoSpecific
{
	Tserial robo;
	double EmotorDistance2 = 0.0;
	double EmotorDistance = 0.0; 
	//Tserial -- TBD to communicate with arm
	//bool SwitchStat -- TBD used for home
	//bool ServoWait -- TBD
	double TickAng1 = 750;   //Represent the number of clicks to get from 0deg to 90deg
	double TickAng2 = 750;   //Represent the number of clicks to get from 0deg to 90deg
	double TempDistanceE = 0;
	//Todo -convert angle pair to ticks dividing by .12 pass to MotoMoveServo
public:
	RhrinoSpecific() {
		char Ports[] = "COM1";
		robo.connect(Ports, 9600, spEVEN);
	}
	bool MotoMoveServo(char c, int Distance); //  Distance in clicks or servo divisions
	//This method will store the complete distance for servo E, the shoulder and Servo D,
	//the elbo in TickAng1 and TickAng2.  We will only move the difference between the stored
	//value in TickAng1 and 2, but store the complete number once we are done

	double unitstepVal = 0.2; // was 0.0
	void setUnitstepVal(double a) { unitstepVal = a; }


};

#endif

