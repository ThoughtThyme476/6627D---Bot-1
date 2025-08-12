#include "main.h"


//hi
//header Guards, safety
#ifndef ROBOTH
#define ROBOTH

extern pros::Motor LF;
extern pros::Motor LM;
extern pros::Motor LB;
extern pros::Motor RF;
extern pros::Motor RM;
extern pros::Motor RB;

extern pros::Motor Intake1;
extern pros::Motor Intake2;
extern pros::Motor MainIntake;
extern pros::Imu imu;
extern pros::Optical eyes; 
extern pros::Rotation roto;
extern pros::Distance dist_left;
extern pros::Distance dist_right;
extern pros::Controller con;


extern pros::ADIDigitalOut MiddleHood;
extern pros::ADIDigitalOut Basket;
extern pros::ADIDigitalOut Intake_Piston;
extern pros::ADIDigitalIn selec;

#endif
