//============================================================================
// Name        : Fireman.cpp
// Author      : Team11
// Version     : 3
// Copyright   : CSE-4360-001
// Description : Project02
//============================================================================


#include <ev3.h>
#include <string>
#include "Fireman.h"


int main(){
	InitEV3();

	// Set the sensors to the input ports
	setAllSensorMode(TOUCH_PRESS,COL_COLOR,US_DIST_MM,GYRO_ANG);

	int flag=0; // wall not found
	int count; // count for looping times in Adjustments

	goto Mode; // got the the original check-point

/*
 *	Original check-point, check all the sensors and flag, choose Mode
*/
Mode:
	if(readSensor(readSensor(IN_1)==1))
	{
		goto Adjust1;
	}
	// If fire region detect, end
	else if(readSensor(IN_2)==2)
	{
		goto EndLoop;
	}
	// If the flag is 0, wander mode
	if(flag == 0)
	{
		goto Wander;
	}
	// If the flag is not 0, wall follow mode
	else if(flag !=0)
	{
		goto WallFollow;
	}
	// Other situations
	goto EndLoop;

/*
 *	Wander mode
 */
Wander:
	if (flag == 0)
	{
		// Wander function called, makes the Robot to
		// find a wall->turn to face at that wall
		Wander();
	}
	// After wall found, change mode
	else if(flag!=0)
	{
		LcdPrintf(1, "Wall Found!\n");
		goto Mode;
	}


	// Move to the wall
	do{
		OnFwdReg(OUT_BC,15);
		Wait(100);
	}while(readSensor(IN_1)==0 && readSensor(IN_2)!=2);
	Off(OUT_BC);
	// Detect the wall
	if(readSensor(readSensor(IN_1)==1))
	{
		goto Adjust1;
	}
	// If fire region detected, end
	else if(readSensor(IN_2)==2)
	{
		goto EndLoop;
	}
	// Other situations
	goto EndLoop;

/*
 *	Wall follow mode
 */
WallFollow:
	// Follow the wall, stop when touched/blue/out of range
	do{
		OnFwdReg(OUT_BC,25);
		Wait(100);
	}while(readSensor(IN_1)==0 && readSensor(IN_2)!=2 && (readSensor(IN_3)<110 && readSensor(IN_3)>40));
	Off(OUT_BC);

	// If touched, backup and counter turn
	if(readSensor(readSensor(IN_1)==1))
	{
		goto Adjust1;
	}
	// If fire region detect, end
	else if(readSensor(IN_2)==2)
	{
		goto EndLoop;
	}

	do
	{
		OnFwdReg(OUT_BC,15); // Forward
		Wait(10);
		count++;
	}
	while(count!=50 && readSensor(IN_1)==0 &&  readSensor(IN_2)!=2 && readSensor(IN_4)!=-1);
	Off(OUT_BC);
	count = 0;
	// If touched, backup and counter turn
	if(readSensor(readSensor(IN_1)==1))
	{
		goto Adjust1;
	}
	// If fire region detect, end
	else if(readSensor(IN_2)==2)
	{
		goto EndLoop;
	}
	// If lost the wall, dist>150mm, clockwise turn
	if(readSensor(IN_3)>150)
	{
		ClockTurn_1w();
		goto WallFollow;
	}
	// If off the wall, 150>=dist>110,
	else if(readSensor(IN_3)<=150 && readSensor(IN_3)>110)
	{
		goto Adjust2;
	}
	// If close to the wall, dist<=40
	else if(readSensor(IN_3)<=40)
	{
		goto Adjust3;
	}
	// Other situations
	goto EndLoop;


/*
 * Wall in front, counter clockwise turn
 */
Adjust1:
	// Firstly move backward a little bit
	OnRevReg(OUT_BC,15);
	Wait(1000);
	Off(OUT_BC);
	// Counter turn 90, use 87 to compensate with the gyro sensor
	CounterTurn_2w(87);
	// Wall found, change mode
	if(flag==0){
		flag=1;
		goto Mode;
	}
	// Otherwise, always:
	// Keep following the wall
	goto WallFollow;


/*
 * Away from the wall, make it closer and parallel
 */
Adjust2:
	// Firstly, decrease the distance to the wall on right
	DecreaseDist();
	// If touch the wall in front, go Adjust1
	if(readSensor(IN_1)==1)
	{
		goto Adjust1;
	}
	// If fire found, go EndLoop
	else if(readSensor(IN_2)==2)
	{
		goto EndLoop;
	}
	// Parallel with the wall, then keep wall following
	Parallel();
	goto WallFollow;


/*
 * Close to the wall, make it away and parallel
 */
Adjust3:
	// Firstly, increase the distance
	IncreaseDist();
	// If touch the wall, go Adjust1
	if(readSensor(IN_1)==1)
	{
		goto Adjust1;
	}
	// If fire found, go EndLoop
	else if(readSensor(IN_2)==2)
	{
		goto EndLoop;
	}
	// Parallel with the wall, then keep wall following
	Parallel();
	goto WallFollow;


/*
 *	End of the behaviors, fire region found.
*/
EndLoop:
	LcdClean();
	LcdPrintf(1, "   -===FIRE ALERT===-\n");
	LcdPrintf(1, "   -===FIRE ALERT===-\n");
	LcdPrintf(1, "   -===FIRE ALERT===-\n");
	LcdPrintf(1, "   -===FIRE ALERT===-\n");
	LcdPrintf(1, "   -===FIRE ALERT===-\n");
	Wait(3000);
	FreeEV3();
	exit(0);

/*
 *	Test use only
 *
 *	Check all the sensors whether they are functioning correctly or not
 *
CheckTouch:
	do{
		OnFwdReg(OUT_BC,15);
		Wait(100);
	}while(readSensor(IN_1)==0);
	Off(OUT_BC);
	LcdClean();
	LcdPrintf(1, "Touched\n");
	Wait(2000);
	goto EndLoop;

CheckColor:
	do{
		OnFwdReg(OUT_BC,15);
		Wait(100);
	}while(readSensor(IN_2)!=2);
	Off(OUT_BC);
	LcdClean();
	LcdPrintf(1, "Region detect\n");
	Wait(2000);
	goto EndLoop;

CheckDist:
	do{
		OnFwdReg(OUT_BC,15);
		Wait(5);
	}while(readSensor(IN_3)<110 && readSensor(IN_3)>40);
	Off(OUT_BC);
	LcdClean();
	LcdPrintf(1, "Out of range: %d\n",readSensor(IN_3));
	Wait(2000);
	goto EndLoop;
*/


	FreeEV3();
}
