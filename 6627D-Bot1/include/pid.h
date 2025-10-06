#include "api.h"
#include "main.h"
#include "robot.h"
//header guards
#ifndef PIDH
#define PIDH

#define STRAIGHT_KP 3.4 
#define STRAIGHT_KI 0.15
#define STRAIGHT_KD 56 
//done. Keep as is 

#define STRAIGHT_INTEGRAL_KI 40//
#define STRAIGHT_MAX_INTEGRAL 14.5//

extern double calcPID(double target, double input, int integralKi, int maxIntergral);
extern double calcPID2(double target2, double input2, int integralKI2, int maxIntegral2);
extern void chasMove(int voltageLF, int voltageLM, int voltageLB, int voltageRF, int voltageRM, int voltageRB);
extern void ColorSort();
extern void driveStraight(int target);
extern void driveStraight2(int target);
extern void driveStraightC(int target);
extern void driveTurn(int target);
extern void driveTurn2(int target);
extern void driveStraightSlow(int target, int speed);
extern void driveIntake(int target, int start, int stop);
extern void driveClamp (int target, int clampDistance);
extern void setConstants( double kp, double ki, double kd);
extern void driveClampS (int target, int clampDistance, int speed);
extern void driveIntakeSlow (int target, int start, int stop, int speed);
extern void driveArcR(double theta, double radius, int timeout, int speed);
extern void driveArcLF(double theta, double radius, int timeout, int speed);
extern void driveArcL(double theta, double radius, int timeout, int speed);
extern void driveArcRF(double theta, double radius, int timeout, int speed);
extern void ColorSenseIntakeRed(int speed);
extern void ColorSenseIntakeBlue(int speed);
extern void driveStraightRush(int target);
extern void driveSortHoldRed(int target, int speed);
extern void driveSortHoldRedC(int target, int speed);
extern void driveSortHoldblue(int target, int speed);
extern void driveSortHoldblueC(int target, int speed);
extern void WallStakePos(int speed, int SlowSense);
extern void RingHold(int speed);
extern void driveTurnT(int target);
extern void wallResetF(int resetTime);
extern void wallResetB(int resetTime);
extern void Counting ();
extern void IntakeStopper(int SecVoltage, int choose);
extern int color;
extern bool InitColor;

extern int time2;
extern float error;
extern int tunetime2;
extern int number;
extern void justIntake (int time);
extern void hooks(int speed);
//extern void stallProt();
extern int viewTime;



#define TURN_KP 4//
#define TURN_KI 0.1// 
#define TURN_KD 32// 

#define TURN_INTRGRAL_KI 30
#define TURN_MAX_INTEGRAL 25

#define TURNT_KP 1//
#define TURNT_KI 0// 
#define TURNT_KD 0// 

#define TURNT_INTRGRAL_KI 30
#define TURNT_MAX_INTEGRAL 25

#define LIFT_KP 1// 
#define LIFT_KI 0// 
#define LIFT_KD 0// 

//straight stuff
#define HEADING_KP 2.1
#define HEADING_KI 0.12
#define HEADING_KD 15
#define HEADING_MAX_INTEGRAL 0
#define HEADING_INTEGRAL_KI 0

//arc turn stuff
#define ARC_HEADING_KP 18 //make it bigger until u can see it correcting along the path 
#define ARC_HEADING_KI 0
#define ARC_HEADING_KD 10 // make it bigger until it is smooth //18
#define ARC_HEADING_MAX_INTEGRAL 0
#define ARC_HEADING_INTEGRAL_KI 0 

#define MACRO_KP 1
#define MACRO_KI 0
#define MACRO_KD 0



#endif