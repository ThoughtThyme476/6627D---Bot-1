#include "main.h"
#include "api.h"
#include "robot.h"

//header Guards, safety

#define LF_PORT 14
#define LM_PORT 11 
#define LB_PORT 13 
#define RF_PORT 2
#define RM_PORT 18 
#define RB_PORT 20 
#define IMU_PORT 5
#define Intake1_port 12
#define Intake2_PORT 19
#define MainIntake_PORT 16
#define OPTICAL_PORT 4
#define DISTANCE_PORT 15
#define DISTANCE_PORT2 16
#define ROTATION_PORT 17
#define DISTANCE_PORT3 6

pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, true); //keep
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, false); //keep
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, true); //keep
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, false); //keep
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, true); //keep
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, false); //keep                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           

pros::Motor Intake1 (Intake1_port, pros::E_MOTOR_GEARSET_18, true);
pros::Motor Intake2 (Intake2_PORT, pros::E_MOTOR_GEARSET_18, true);
pros::Motor MainIntake (MainIntake_PORT, pros::E_MOTOR_GEARSET_06, true);

pros::Imu imu(IMU_PORT);
pros::Optical eyes(OPTICAL_PORT);
pros::Rotation roto(ROTATION_PORT);
pros::Distance dist_left (DISTANCE_PORT);
pros::Distance dist_right (DISTANCE_PORT2);
pros::Controller con (pros::E_CONTROLLER_MASTER);
pros::Distance Counter (DISTANCE_PORT3);

pros::ADIDigitalOut Intake_Piston ('D',false);
pros::ADIDigitalOut MiddleHood ('A',false);//out soloniod was stolen so I changed this
pros::ADIDigitalOut Basket('B',false);
pros::ADIDigitalOut TopHood('E', false);
pros::ADIDigitalIn selec ('H');