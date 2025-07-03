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