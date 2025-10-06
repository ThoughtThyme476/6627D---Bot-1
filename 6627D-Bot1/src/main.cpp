#include "main.h"
#include "api.h"
#include "pid.h"
#include "robot.h"
#include "auton.h"
#include "field.h"
#include "odom.h"
#include "pure_pursuit.h"

lv_obj_t* Image2;
bool stay_clamp = true;

using namespace pros;
using namespace std;

int atn = 1;
string autstr;
// Task colorSortTask;
// bool colorSortActive = false;

// void on_center_button() {
// 	static bool pressed = false;
// 	pressed = !pressed;
// 	if (pressed) {
// 		pros::lcd::set_text(2, "I was pressed!");
// 	} else {
// 		pros::lcd::clear_line(2);
// 	}
// }

LV_IMG_DECLARE(Image);
void initialize() {

lv_obj_t *img = lv_img_create(lv_scr_act(), NULL);
lv_img_set_src(img, &Image2);
lv_obj_align(img, NULL, LV_ALIGN_CENTER, 0, 0);

}
void disabled() { 
	// imu.tare(); 
	// while(stay_clamp = true){
	// 	MogoMech.set_value(true);
	// }
}

//void autonomous moved into just auton.cpp

void competition_initialize() {
	int pressed = 1;


	
while(true){
	if(selec.get_value() == true){
		pressed ++;
	} else {
		pressed = 0;
	}

	if(pressed ==1){
		atn++;
	}
		if(atn>9){//change number to number of autons
			atn=0;
		}
		if (atn == 0) {
			autstr = "NONE";
			con.print(0,0, "Aut 0:%s", autstr);
		}
		else if (atn ==1) {
			autstr = "RED AWP";
			con.print(0,0, "Aut 1: %s", autstr);
		}
		else if (atn ==2) {
			autstr = "BLUE AWP";
			con.print(0,0, "Aut 2: %s", autstr);
		}
		else if (atn ==3) {
			autstr = "RED GOAL E";
			con.print(0,0, "Aut 3: %s", autstr);
		}
		else if (atn ==4) {
			autstr = "BLUE GOAL E";
			con.print(0,0, "Aut 4: %s", autstr);
		}
		else if (atn ==5) {
			autstr = "Straight";
			con.print(0,0, "Aut 5: %s", autstr);
		}
		else if (atn ==6) {
			autstr = "Straight2";
			con.print(0,0, "Aut 6: %s", autstr);
		}
		else if (atn ==7) {
			autstr = "Red Ring Rush";
			con.print(0,0, "Aut 7: %s", autstr);
		} 
		else if (atn ==8) {
			autstr = "Blue Ring Rush";
			con.print(0,0, "Aut 8: %s", autstr);
		}
		else if (atn ==9) {
			autstr = "SKILLS";
			con.print(0,0, "Aut 9: %s", autstr);
		}
	}

}


void opcontrol() {

// setPosition(0,0,0);
bool arcToggle = true;
bool tankToggle = false;
bool GoalToggle = true;
bool slow = false;
int time = 0;
int tuneDist = 18;
bool Storage = false;
bool DescoreToggle = false;
bool MatchLoad = false;
bool park = false;
eyes.set_led_pwm(100);
int intakeMode = 0; // 0-3 for different intake modes
int number = 0;


while (true) {

//chassis drive 
int power = con.get_analog(ANALOG_LEFT_Y);
int RX = con.get_analog(ANALOG_RIGHT_X);
int turn = int(pow(RX, 3)/ 24193); // change 16129 by whatever to make turning more or less sensetive 
int left= power - turn; 
int right = power + turn;

// if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
// 	arcToggle = !arcToggle;
// 	tankToggle = !tankToggle;
// }

if (tankToggle){
	LF.move(con.get_analog(ANALOG_LEFT_Y));
	LM.move(con.get_analog(ANALOG_LEFT_Y));
	LB.move(con.get_analog(ANALOG_LEFT_Y));
	RF.move(con.get_analog(ANALOG_RIGHT_Y));
	RM.move(con.get_analog(ANALOG_RIGHT_Y));
	RB.move(con.get_analog(ANALOG_RIGHT_Y));
}

if (arcToggle) {
LF.move(right);
LM.move(right);
LB.move(right);
RF.move(left);
RM.move(left);
RB.move(left);
}



const int UPPER_THRESHOLD = tuneDist + 2;
const int LOWER_THRESHOLD = tuneDist - 2;

if ((Toggle.get() < LOWER_THRESHOLD) && (E_CONTROLLER_DIGITAL_R1 || E_CONTROLLER_DIGITAL_R2)){ 
    GoalToggle = false;
} else if((Toggle.get() > LOWER_THRESHOLD) && (E_CONTROLLER_DIGITAL_R1 || E_CONTROLLER_DIGITAL_R2)){
    GoalToggle = true;
} //else {
// 	GoalToggle = NULL;
// }
// Values between LOWER_THRESHOLD and UPPER_THRESHOLD maintain previous state


if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
	// slow = !slow;

	driveTurn2(175);
}


// Mode toggle with A button
if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)) {
    intakeMode = intakeMode + 1;
    if (intakeMode > 2) intakeMode = 0;
    con.print(0, 0, "Mode: %d", intakeMode);
}

// Intake control based on mode
switch(intakeMode) {
    case 0: // Mode 0
        if (con.get_digital(E_CONTROLLER_DIGITAL_R1)) {
            	if(slow == true){
				Intake1.move(100);
				Intake2.move(100);
				MainIntake.move(-100);
				} else {
			Intake1.move(127);
			MainIntake.move(-127);
			Intake2.move(127);
			}
        } else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)) {
            	if(slow == true){
			Intake1.move(-100);
			Intake2.move(-100);
			MainIntake.move(100);
			} else {
			Intake1.move(-127);
			MainIntake.move(127);
			Intake2.move(-127);
			} 
        } else {
            // Stop motors when no buttons pressed
            Intake1.move(0);
            MainIntake.move(0);
            Intake2.move(0);
        }
        break;

    case 1: // Mode 1
        if (con.get_digital(E_CONTROLLER_DIGITAL_R1)) {
              	if(slow == true){
				Intake1.move(-100);
				Intake2.move(100);
				MainIntake.move(-100);
				} else {
			Intake1.move(-127);
			MainIntake.move(-107);
			Intake2.move(127);
			}
        } else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)) {
              	if(slow == true){
			Intake1.move(-100);
			Intake2.move(100);
			MainIntake.move(100);
			} else {
			Intake1.move(-127);
			MainIntake.move(127);
			Intake2.move(127);
			} 
        } else {
            Intake1.move(0);
            MainIntake.move(0);
            Intake2.move(0);
        }
        break;

    case 2: // Mode 2
        if (con.get_digital(E_CONTROLLER_DIGITAL_R1)) {
                 	if(slow == true){
				Intake1.move(-100);
				Intake2.move(-100);
				MainIntake.move(-100);
				} else {
			Intake1.move(-127);
			MainIntake.move(-107);
			Intake2.move(-127);
			}
        } else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)) {
               	if(slow == true){
			Intake1.move(100);
			Intake2.move(100);
			MainIntake.move(100);
			} else {
			Intake1.move(127);
			MainIntake.move(127);
			Intake2.move(127);
			} 
        } else {
            Intake1.move(0);
            MainIntake.move(0);
            Intake2.move(0);
        }
        break;
}

if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
	park = !park;
}
Park.set_value(park); 

if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)){
	MatchLoad = !MatchLoad;
}
TongueMech.set_value(MatchLoad); 

if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)){
	DescoreToggle = !DescoreToggle;
}
Descore.set_value(DescoreToggle); 


double Intake_temp = ((Intake1.get_temperature()));
double  chasstemp = (((RF.get_temperature() + RB.get_temperature() + LF.get_temperature() + LB.get_temperature())/4)*(9/5)+32);
if (time % 50 == 0 && time % 100 !=0 && time % 150 !=0){
    con.print(0,0,"time:%f       ", float(time2));//viewTime
} else if (time% 100 == 0 && time % 150 !=0){
    con.print(1,0,"error%f      ", float(time2));
} else if (time % 150 == 0){
    con.print(2,0,"C:%i MI:%i IM:%i      ",int(chasstemp), int(Intake_temp), int((MainIntake.get_temperature())));
}

//   if (time % 50 == 0 && time % 100 != 0 && time % 150 != 0) {
//         con.print(0,0, "time: %.1f mm     ", float(time2));
//     } else if (time % 100 == 0 && time % 150 != 0) {
//         con.print(1,0, "error: %.1f mm     ", float(error));
//     } else if (time % 150 == 0) {
//         con.print(2,0, "Heading: %.1f deg / %.2f rad ", float(imu_pos), float(imu_pos_radians));
    //}

delay(10);
time += 10;

}


}