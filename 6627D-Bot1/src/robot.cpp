#include "main.h"
#include "api.h"
#include "robot.h"


#define LF_PORT 4
#define LM_PORT 5 
#define LB_PORT 8 
#define RF_PORT 1
#define RM_PORT 2 
#define RB_PORT 3 
#define IMU_PORT 7
#define Intake1_port 9
#define Intake2_port 11
#define MainIntake_PORT 10
#define OPTICAL_PORT 12
#define DISTANCE_PORT 13
#define DISTANCE_PORT2 14
#define DISTANCE_PORT3 15
#define DISTANCE_PORT4 16
#define ROTATION_PORT 17

pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, true); //keep
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, true); //keep
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, true); //keep
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, false); //keep
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, false); //keep
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, false); //keep                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           

pros::Motor Intake1 (Intake1_port, pros::E_MOTOR_GEARSET_18, true);
pros::Motor Intake2 (Intake2_port, pros::E_MOTOR_GEARSET_18, true);
pros::Motor MainIntake (MainIntake_PORT, pros::E_MOTOR_GEARSET_06, true);

pros::Imu imu(IMU_PORT);
pros::Optical eyes(OPTICAL_PORT);
pros::Rotation roto(ROTATION_PORT);
pros::Distance dist_left (DISTANCE_PORT);
pros::Distance dist_right (DISTANCE_PORT2);
pros::Controller con (pros::E_CONTROLLER_MASTER);
pros::Distance Counter (DISTANCE_PORT3);

pros::ADIDigitalOut TongueMech ('D',false);
pros::ADIDigitalOut MiddleHood ('A',false);
pros::ADIDigitalOut Basket('B',false);
pros::ADIDigitalOut TopHood('E', false);
pros::ADIDigitalIn selec ('H');