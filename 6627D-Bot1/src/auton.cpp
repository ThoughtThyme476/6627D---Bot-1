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
  driveStraightC(250);
  driveTurn2(-45);
  MainIntake.move(-127);
  Intake1.move(127);
  driveStraightSlow(300, 80);
  //TongueMech.set_value(true);
  driveStraightSlow(325, 65);
  driveTurn2(-135);
  driveStraight2(-630);
  Intake1.move(-127);
  Intake2.move(-127);
  delay(700);
    MainIntake.move(127);
    Intake2.move(0);
    driveStraightC(100);
    MainIntake.move(-127);
    driveStraight2(1950);
    Intake1.move(127);
    TongueMech.set_value(true);
    driveTurn2(180);
    delay(100);
    driveStraightSlow(660, 95);
    // //driveTurn2(178);
    driveStraight2(10);
    TongueMech.set_value(false);
    // driveStraight2(-920);
    // MainIntake.move(127);
    // Intake2.move(127);
    // MainIntake.move(-127);
    // Intake1.move(-127);
    // driveTurn2(180);
  }

 void Ring_Rush_red(){ 
  MainIntake.move(-127);
  Intake1.move(127);
  driveStraightCslow(400, 90);
  driveTurn2(-90);
  driveStraightCslow(200, 50);
    TongueMech.set_value(true);
    driveStraightCslow(90, 50);
    driveStraight2(10);
  MainIntake.move(127);
  driveArcLF(5, 300, 1000, 50);
  TongueMech.set_value(false);
  MainIntake.move(-127);
  driveArcLF(40, 300, 1000, 50);
  driveArcRF(45, 300, 1500, 80);
  driveStraight2(320);
  driveTurn2(180);
  driveStraightSlow(-245, 40);
  Intake2.move(127);
  Intake1.move(-127);
	MainIntake.move(-107);
  delay(1500);
  TongueMech.set_value(true);
  driveStraight2(875);
  driveStraight2(250);
  delay(200);
  Intake1.move(127);
  MainIntake.move(127);
  TongueMech.set_value(false);
  driveStraightCslow(-150, 50);
  MainIntake.move(-127);
  driveStraightCslow(-150, 75);
  //driveTurn2(-179);
  driveStraight2(-630);
  Intake1.move(-127);
  delay(1000);
  // driveStraightCslow(200, 75);
  // driveTurn2(145);
  // driveStraightCslow(-290, 100);
  // driveTurn2(180);
 
  }

 void goal_safe_side_red(){ 
  

  }


 void Standard_AWP_blue(){ //done

          
  }


 void Goal_Rush_Blue(){ 

 }


 void goal_safe_side_blue(){


  }

  void Ring_Rush_blue(){
    
    
 }
 
 void skill_run(){
  driveStraightCslow(645, 90);
  TongueMech.set_value(true);
  driveTurn2(90);
  MainIntake.move(-127);
  Intake1.move(127);
  driveStraightSlow(600, 60);
  delay(100);
  TongueMech.set_value(false);
  driveStraight2(10);
  MainIntake.move(127);
  driveStraightCslow(-100, 70);
  driveStraightCslow(-875, 70);
  driveTurn2(95);
  //driveStraightSlow(-30, 50);
    MainIntake.move(127);
    Intake2.move(127);
    MainIntake.move(-127);
    Intake1.move(-127);
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