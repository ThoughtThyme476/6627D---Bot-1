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
  eyes.set_led_pwm(100);
  driveStraightC(575);
    //driveStraightC(300);
    driveArcLF(45, 300, 1000, 75);
          TongueMech.set_value(true);
    driveStraightC(95);
    driveArcLF(40, 220, 1000, 75);
      Intake1.move(-127);  
      MainIntake.move(-127);
    driveStraight2(400);
  driveStraight2(-75);
  driveStraight2(100);
    driveStraight2(-75);
  driveStraight2(100);
  driveStraightC(-200);
  TongueMech.set_value(false);
  TopHood.set_value(true);
    Intake1.move(-127);  
  MainIntake.move(-127);
  driveStraight2(-300);
  Intake1.move(0);  
  MainIntake.move(0);
  driveTurn2(89);
  Basket.set_value(true);
  driveStraight2(600);
  driveStraightSlow(75, 25);
  delay(2000);
  TopHood.set_value(false);
  Basket.set_value(false);
  driveStraight2(-500);
  driveTurn2(136);
  // driveStraight2(1500);
  // Intake1.move(0);
  // MainIntake.move(0);
  // TongueMech.set_value(true);
  // driveTurn2(140);
  // driveStraight2(150);
  // Basket.set_value(true);
  // MiddleHood.set_value(true);
  // Intake1.move(-127);  
  // MainIntake.move(-127);
 }
 void Goal_Rush_red(){ 
  eyes.set_led_pwm(100);
  driveStraight2(1000);
  MainIntake.move(-127);
  Intake1.move(-127);
  driveSortHoldblue(1000, 50);
  driveSortHoldblue(-1000, 50);
 }

 void Ring_Rush_red(){ 


  }

 void goal_safe_side_red(){ 


  }


 void Standard_AWP_blue(){ 
    eyes.set_led_pwm(100);
  driveStraightC(540);
    //driveStraightC(300);
    driveArcLF(45, 255, 1000, 75);
               TongueMech.set_value(true);
    driveStraightC(100);
    driveArcLF(35, 260, 1000, 75);
      Intake1.move(-127);  
      MainIntake.move(-127);
  driveTurn2(-90);
  driveStraight2(250);
  driveStraight2(-75);
  driveStraight2(250);
  driveStraight2(-75);
  driveStraight2(100);
  driveStraightC(-200);
  TongueMech.set_value(false);
    Intake1.move(0);  
  MainIntake.move(0);
  TopHood.set_value(true);
  driveStraight2(-300);
  driveTurn2(91);
  Basket.set_value(true);
  driveStraight2(475);
    Intake1.move(-127);  
    MainIntake.move(-127);
  driveStraightSlow(25, 75);
  delay(2000);
  TopHood.set_value(false);
  Basket.set_value(false);
  driveStraight2(-500);
  driveTurn2(134);
  driveStraight2(1500);
  TongueMech.set_value(true); 
  driveTurn2(145);
  Basket.set_value(true);
  driveStraight2(230);
  MiddleHood.set_value(true);
  }


 void Goal_Rush_Blue(){ 


 }


 void goal_safe_side_blue(){
  

  }

  void Ring_Rush_blue(){
    
    
 }
 
 void skill_run(){
  driveStraight2(1525);
  driveTurn2(90);
  TongueMech.set_value(true);
  delay(500);
  Intake1.move(-127);
  MainIntake.move(-127);
  driveStraightSlow(375, 90);
  driveSortHoldblue(-50, 100);
  driveSortHoldblue(75, 100);
  driveSortHoldblue(-50, 100);
  driveSortHoldblue(75, 100);
  driveSortHoldblue(-50, 100);
  driveSortHoldblue(100, 100);
  driveStraightC(-200);
  TongueMech.set_value(false);
  driveStraight2(-300);
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