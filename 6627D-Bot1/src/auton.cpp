#include "main.h"
#include "api.h"
#include "pid.h"
#include "robot.h"
#include "auton.h"
 using namespace pros;


  void DriveStraightAuto(){
    driveStraightSlow(250, 50);
  }

 void Standard_AWP_red(){
  driveStraight2(1200);
  driveTurn2(-90);
  Intake_Piston.set_value(true);
  Intake1.move(-127);
	Intake2.move(127);
	MainIntake.move(-127);
  //MiddleHood down
  driveStraightSlow(200, 50);
  wallResetF(5);
  //MiddleHood up
  driveStraightSlow(-50, 40);
  driveStraight2(-200);
  driveTurn2(90);
  Intake1.move(0);
	Intake2.move(0);
	MainIntake.move(0);
  Intake_Piston.set_value(false);
  driveStraight2(875);
  //put hood into long goal mode
  //driveStraightSlow(50, 15);
  delay(2000);
  driveStraightC(-300);
  driveTurn2(150);
  driveStraight2(650);
  Intake1.move(-127);
	Intake2.move(127);
	MainIntake.move(-127);
  driveArcLF(60, 500, 1000, 25);
  driveArcRF(35, 550, 1000, 50);
  Intake1.move(0);
	Intake2.move(0);
	MainIntake.move(0);
  driveStraight2(400);
 
  
 }
 void Goal_Rush_red(){ 

 }

 void Ring_Rush_red(){ 


  }

 void goal_safe_side_red(){ 


  }


 void Standard_AWP_blue(){ 


  }


 void Goal_Rush_Blue(){ 


 }


 void goal_safe_side_blue(){
  

  }

  void Ring_Rush_blue(){
    
    
 }
 
 void skill_run(){


 }
 
 void autonomous(){
  if (atn == 0) {
  }else if (atn == 0) {
    DriveStraightAuto();
  }
   else if (atn == 1) {
    Standard_AWP_red();
  } 
  else if (atn ==2) {
   Standard_AWP_blue();
  } 
  else if (atn ==3) {
    Goal_Rush_red();
  } 
  else if (atn ==4) {
    Goal_Rush_Blue();
  } 
  else if (atn ==5) {
    goal_safe_side_blue();
  } 
  else if (atn ==6) {
    goal_safe_side_red();
  } 
  else if (atn ==7) {
    Ring_Rush_red();
  }
  else if (atn ==8) {
    Ring_Rush_blue();
  }
  else if (atn ==9) {
    skill_run();
  }
}