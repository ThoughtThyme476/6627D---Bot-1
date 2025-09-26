#include "main.h"
#include "api.h"
#include "pid.h"
#include "robot.h"
#include "auton.h"
 using namespace pros;


  void DriveStraightAuto(){
    driveStraightSlow(250, 50);
  }

 void Standard_AWP_red(){ // done
   eyes.set_led_pwm(100);
  driveStraight2(1340);
  driveTurn2(90);
  driveStraightC(550);
  Intake1.move(-127);  
  MainIntake.move(-127);
  TopHood.set_value(true);
  driveStraight2(105);
  delay(500);
  driveStraight2(-400);
  driveTurn2(-101);
  TongueMech.set_value(true);
  TopHood.set_value(false);
  driveStraight2(300);
  driveTurn2(-102);
  driveStraight2(1075);
  driveStraight2(-310);
  driveStraight2(400);
  delay(100);
  driveStraight2(-400);
  TongueMech.set_value(false);
  driveTurn2(135);
  driveStraightC(850);
  driveTurn2(90);
  driveStraightSlow(475, 50);
  driveStraight2(-300);
  driveTurn2(133);
  TopHood.set_value(true);
  Intake1.move(127);
  MainIntake.move(127);
  TongueMech.set_value(true);
  Basket.set_value(true);
  // delay(500);
  MainIntake.move(0);
  driveStraight2(535);
  MainIntake.move(-127);
  Intake1.move(-127);
  MiddleHood.set_value(true);

 }
 void Goal_Rush_red(){ 
  eyes.set_led_pwm(100);
   Intake1.move(-127);
  MainIntake.move(-127); 
  driveStraight2(600);
  driveTurn2(22);
  driveStraightSlow(600, 20);
  driveTurn2(135);
  driveStraight2(1300);
  Intake1.move(127);
  driveTurn2(180);
  TongueMech.set_value(true);
  Intake1.move(-127);
  driveStraight2(800);
  driveStraight2(-250);
  driveStraight2(300);
  delay(300);
  driveStraight2(-400);
  Intake1.move(-127);
  MainIntake.move(-127);
  Basket.set_value(true);
  TongueMech.set_value(false);
  driveTurn2(-1);
  driveStraight2(700);
  driveTurn2(0);
  driveStraightSlow(300, 75);
  TopHood.set_value(true);
  Intake1.move(-127);
  MainIntake.move(-127);


 }

 void Ring_Rush_red(){ 


  }

 void goal_safe_side_red(){ 
    driveStraightSlow(150, 25);

  }


 void Standard_AWP_blue(){ //fdone
    eyes.set_led_pwm(100);
  driveStraight2(1600);
  driveTurn2(90);
  driveStraightC(700);
  Intake1.move(-127);  
  MainIntake.move(-127);
  TopHood.set_value(true);
  driveStraight2(105);
  delay(500);
  driveStraight2(-400);
  driveTurn2(-95);
  TongueMech.set_value(true);
  TopHood.set_value(false);
  driveStraight2(550);
  driveTurn2(-93);
  driveStraight2(600);
  driveStraightSlow(300, 80);
  driveStraight2(-310);
  driveStraight2(400);
  driveStraight2(-350);
  TongueMech.set_value(false);
  driveTurn2(135);
  driveStraightC(1150);
  driveTurn2(90);
  driveStraightSlow(475, 50);
  driveTurn2(133);
  TopHood.set_value(true);
  Intake1.move(127);
  MainIntake.move(127);
  TongueMech.set_value(true);
  Basket.set_value(true);
  delay(500);
  MainIntake.move(0);
  driveStraight2(710);
  driveStraight2(-15);
  MainIntake.move(-127);
  Intake1.move(-127);
  MiddleHood.set_value(true);
          
  }


 void Goal_Rush_Blue(){ 
  eyes.set_led_pwm(100);
   Intake1.move(-127);
  MainIntake.move(-127); 
  driveStraight2(600);
  driveTurn2(22);
  driveStraightSlow(600, 20);
  driveTurn2(135);
  driveStraight2(1350);
  Intake1.move(127);
  driveTurn2(180);
  TongueMech.set_value(true);
  Intake1.move(-127);
  driveStraight2(500);
  driveStraightSlow(300, 40);
  driveStraight2(-250);
  driveStraight2(300);
  delay(300);
  driveStraight2(-400);
  Intake1.move(-127);
  MainIntake.move(-127);
  TongueMech.set_value(false);
  Intake1.move(127);
  MainIntake.move(0);
  driveTurn2(-7);
  driveStraight2(720);
  driveTurn2(-9);
  TopHood.set_value(true);
  Intake1.move(-127);
  MainIntake.move(-127);
  delay(400);
  Basket.set_value(true);
  //driveStraightSlow(300, 75);
 }


 void goal_safe_side_blue(){
  driveStraightSlow(150, 25);

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