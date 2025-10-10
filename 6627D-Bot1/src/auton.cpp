#include "main.h"
#include "api.h"
#include "pid.h"
#include "robot.h"
#include "auton.h"
 using namespace pros;


  void DriveStraightAuto(){

  }

 void Standard_AWP_red(){ // done
 

 }
 void Goal_Rush_red(){ 
  driveStraightC(300);
  driveTurn2(-45);
  MainIntake.move(-127);
  Intake1.move(127);
  driveStraightSlow(300, 80);
  TongueMech.set_value(true);
  MainIntake.move(127);
  driveStraightC(30);
  MainIntake.move(-127);
  driveStraight2(200);
  driveTurn2(-140);
  driveStraightC(-560);
  Intake2.move(-127);
  driveStraight2(-15);
  delay(250);
  Intake2.move(0);
  driveTurn2(-130);
  MainIntake.move(127);
  driveStraightC(100);
  MainIntake.move(-127);
  driveStraight2(1875);
  driveTurn2(180);
  driveStraight2(625);
  TongueMech.set_value(false);
  //driveTurn2(178);p
  driveStraight2(-500);
  MainIntake.move(127);
  delay(100);
  MainIntake.move(-120);
  driveStraightSlow(100, 80);
  driveTurn2(178);
  driveStraightC(-565);
  MainIntake.move(100);
  driveStraight2(-10);
  Intake2.move(127);
  MainIntake.move(-127);
  Intake1.move(-127);






  

 }

 void Ring_Rush_red(){ 


  }

 void goal_safe_side_red(){ 
  

  }


 void Standard_AWP_blue(){ //fdone

          
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