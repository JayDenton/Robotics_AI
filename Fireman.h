/*
 * Fireman.h
 *
 *  Created on: Nov 28, 2018
 *      Author: Team11
 */

#ifndef FIREMAN_H_
#define FIREMAN_H_


#include <ev3.h>
#include <cmath>

void CounterTurn_2w(int angle){
	int ang=readSensor(IN_4);
	if(angle > 0)
	{
		goto situation1;
	}
	if(angle < 0)
	{
		goto situation2;
	}
	else
		goto end;

situation1:
	do
	{
		OnRevReg(OUT_B,5); // Reverse left wheel
		OnFwdReg(OUT_C,5); // Forward right wheel
		Wait(5);
	}while((ang-readSensor(IN_4))<angle && readSensor(IN_2)!=2 && readSensor(IN_4)!=-1);
	Off(OUT_BC);
	goto end;

situation2:
	angle = angle*(-1);
	do
	{
		OnRevReg(OUT_C,5); // Reverse right wheel
		OnFwdReg(OUT_B,5); // Forward left wheel
		Wait(5);
	}while((readSensor(IN_4)-ang)<angle && readSensor(IN_2)!=2 && readSensor(IN_4)!=-1);
	Off(OUT_BC);
	goto end;

end:
	return;
}

void ClockTurn_1w(){
	int ang=readSensor(IN_4);
	int time=0;

	// Firstly, clockwise turn
	do
	{
		OnFwdReg(OUT_B,10); // Forward left wheel
		Wait(5);
	}while((readSensor(IN_4)-ang)<89 && readSensor(IN_2)!=2 && readSensor(IN_4)!=-1);
	Off(OUT_BC);

	// Secondly, forward again to track wall
	time = 0;
	do
	{
		OnFwdReg(OUT_BC,15); // Forward 4 sec
		Wait(10);
		time++;
	}
	while(time!=400 && readSensor(IN_2)!=2 && readSensor(IN_4)!=-1);
	Off(OUT_BC);
}

void Parallel(){
	int count = 0;
	int a = 60; // a is the adjacent edge of the angle , which is 60mm
	int b; // Store wall distance before moving
	int c; // Store wall distance after moving
	int turnAngle=0;

	// Wall distance before moving
	b = readSensor(IN_3);

	// Go forward to get 'a'
	do{
		OnFwdReg(OUT_BC,15); // Forward
		Wait(8); // According to the experiment, average 60 mm
		count++;
	}while(count<100 && readSensor(IN_1)==0 && readSensor(IN_2)!=2 && readSensor(IN_4)!=-1);
	Off(OUT_BC);

	// Check wall and fire region
	if(readSensor(IN_1)==1 || readSensor(IN_2)==2)
		return;

	// Wall distance after moving
	c = readSensor(IN_3);

	// Calculate the angle to turn
	if(b > c){
		turnAngle = (int) floor(atan((double (b)- double(c))/(double (a)))/3.1415926*180.0+0.5);
		CounterTurn_2w(turnAngle);
	}
	else if(b < c){
		turnAngle = -1 * (int) floor(atan((double (c)- double(b))/(double (a)))/3.1415926*180.0+0.5);
		CounterTurn_2w(turnAngle);
	}

	// Now the robot is parallel with the wall
	return;
}

void Wander(){
	int count = 0;
	int a = 60; // a is the adjacent edge of the angle , which is 60mm
	int b; // Store wall distance before moving
	int c; // Store wall distance after moving
	int turnAngle=0;

	// Wall distance before moving
	b = readSensor(IN_3);

	// Go forward to get 'a'
	do{
		OnFwdReg(OUT_BC,15); // Forward
		Wait(8); // According to the experiment, average 60 mm
		count++;
	}while(count<100 && readSensor(IN_1)==0 && readSensor(IN_2)!=2 && readSensor(IN_4)!=-1);
	Off(OUT_BC);

	// Check wall and fire region
	if(readSensor(IN_1)==1 || readSensor(IN_2)==2)
		return;

	// Wall distance after moving
	c = readSensor(IN_3);

	// Calculate the angle to turn
	if(b > c){
		turnAngle = (int) floor(atan((double (b)- double(c))/(double (a)))/3.1415926*180.0+0.5);
		CounterTurn_2w(turnAngle-88);
	}
	else if(b < c){
		turnAngle = -1 * (int) floor(atan((double (c)- double(b))/(double (a)))/3.1415926*180.0+0.5);
		CounterTurn_2w(turnAngle-90);
	}

	// Now the robot is facing at the wall
	return;
}

void DecreaseDist(){
	int count = 0;
	do{
		OnFwdReg(OUT_B,15); // Move left wheel 1.2 sec
		Wait(10);
		count++;
	}while(count<120 && readSensor(IN_1)==0 && readSensor(IN_2)!=2);
	Off(OUT_B);
	if(readSensor(IN_1)==1 || readSensor(IN_2)==2)
		return;

	count = 0;
	do{
		OnFwdReg(OUT_BC,15); // Move both wheel .6 sec
		Wait(10);
		count++;
	}while(count<60 && readSensor(IN_1)==0 && readSensor(IN_2)!=2);
	Off(OUT_BC);
	if(readSensor(IN_1)==1 || readSensor(IN_2)==2)
		return;

	count = 0;
	do{
		OnFwdReg(OUT_C,15); // Move left wheel 1 sec
		Wait(10);
		count++;
	}while(count<100 && readSensor(IN_1)==0 && readSensor(IN_2)!=2);
	Off(OUT_C);
	if(readSensor(IN_1)==1 || readSensor(IN_2)==2)
		return;

	return;
}

void IncreaseDist(){
	int count = 0;
	do{
		OnFwdReg(OUT_C,15); // Move right wheel 1.2 sec
		Wait(10);
		count++;
	}while(count<120 && readSensor(IN_1)==0 && readSensor(IN_2)!=2);
	Off(OUT_C);
	if(readSensor(IN_1)==1 || readSensor(IN_2)==2)
		return;

	count = 0;
	do{
		OnFwdReg(OUT_BC,15); // Move both wheel .6 sec
		Wait(10);
		count++;
	}while(count<60 && readSensor(IN_1)==0 && readSensor(IN_2)!=2);
	Off(OUT_BC);
	if(readSensor(IN_1)==1 || readSensor(IN_2)==2)
		return;

	count = 0;
	do{
		OnFwdReg(OUT_B,15); // Move left wheel 1 sec
		Wait(10);
		count++;
	}while(count<100 && readSensor(IN_1)==0 && readSensor(IN_2)!=2);
	Off(OUT_B);
	if(readSensor(IN_1)==1 || readSensor(IN_2)==2)
		return;

	return;
}


#endif /* FIREMAN_H_ */
