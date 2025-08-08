#include "api.h"
#include "main.h"
#include "pid.h"
#include "robot.h"
#include "auton.h"
#include "odom.h"

using namespace pros;
using namespace std;

int turnv = 0;

double absoluteAngleToTarget = 0;
double position = 0;

float deltaX;
float deltaY;

float startingX;
float startingY; 
float startingHeading;

float r0;
float r1;

float delta_left_encoder_pos;
float delta_right_encoder_pos;
float delta_center_encoder_pos;

float prev_left_encoder_pos;
float prev_right_encoder_pos;
float prev_center_encoder_pos;

float left_encoder_pos;
float right_encoder_pos;
float center_encoder_pos;

float localX;
float localY;

float phi;

float prev_imu_pos;
float imu_pos;
float imu_pos_radians;

float x_pos;
float y_pos;

float pi = 3.14159265359; 

int odo_time = 0;

float local_polar_angle;
float local_polar_length; 
float global_polar_angle;

void setPosition(float xcoord, float ycoord, float heading){
startingX = xcoord; 
startingY = ycoord;

startingHeading = heading;


x_pos = startingX;
y_pos = startingY;
}

void Odometry(){


    prev_imu_pos = imu_pos;
    imu_pos = imu.get_rotation() + startingHeading;

    prev_left_encoder_pos = left_encoder_pos;
    prev_right_encoder_pos = right_encoder_pos;  
    prev_center_encoder_pos = center_encoder_pos;

    left_encoder_pos = (LF.get_position()/360.0)*(36.0/48.0)*(2*pi*1.625);
    right_encoder_pos = (RF.get_position()/360.0)*(36.0/48.0)*(2*pi*1.625);
    center_encoder_pos = (roto.get_angle()/36000.0)*(2*pi);

    delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
    delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
    delta_right_encoder_pos = center_encoder_pos - prev_center_encoder_pos;



    phi = imu_pos - prev_imu_pos;
    const double dtheta_rad = (pi * phi) / 180.00;
    const double th_rad = (pi * imu_pos) / 180.00;
    const double EPS = (pi * IMU_THERSHOLD)/ 180.0;


 if (fabs(dtheta_rad) < EPS) {
          // straight-ish: use linearized form
          localX = (delta_left_encoder_pos + delta_right_encoder_pos) / 2.0;
          // NOTE: dtheta now radians
          localY = delta_center_encoder_pos - FORWARD_OFFSET * dtheta_rad;
        } else {
          // turning: use exact arc formulas (use radians!)
          const double avg_lr = (delta_left_encoder_pos + delta_right_encoder_pos) / 2.0;
          r0 = avg_lr / dtheta_rad;                    // forward radius
          r1 =  (delta_center_encoder_pos) / dtheta_rad; // lateral radius

          const double s = sin(dtheta_rad);
          const double c = cos(dtheta_rad);

          localX = r0 * s - r1 * (1.0 - c);
          localY = r1 * s + r0 * (1.0 - c);
        }

        // rotate into global frame using absolute heading (radians)
        deltaY =  localX * cos(th_rad) - localY * sin(th_rad);
        deltaX =  localX * sin(th_rad) + localY * cos(th_rad);

        x_pos += deltaX;
        y_pos += deltaY;

        if (odo_time % 50 == 0 && odo_time % 100 != 0 && odo_time % 150 != 0){
          con.print(0, 0, "x_pos: %f           ", float(x_pos));
        } else if (odo_time % 100 == 0 && odo_time % 150 != 0){
          con.print(1, 0, "y_pos: %f           ", float(y_pos));
        } else if (odo_time % 150 == 0){
          con.print(2, 0, "Pos: %f        ", float(position)); // FYI: 'position' never updates
        }

        odo_time += 10; // assumes caller delays ~10ms per loop
}



void Odometry2(){
    prev_imu_pos = imu_pos;
    imu_pos = imu.get_rotation() + startingHeading; 
    imu_pos_radians = (imu_pos*pi)/180; 

    prev_left_encoder_pos = left_encoder_pos;
    prev_right_encoder_pos = right_encoder_pos;
    prev_center_encoder_pos = center_encoder_pos;

    left_encoder_pos = (LF.get_position()/360.0)*(36.0/48.0)*(2*pi*1.625);
    right_encoder_pos = (RF.get_position()/360.0)*(36.0/48.0)*(2*pi*1.625);
    center_encoder_pos = (roto.get_angle()/36000.0)*(2*pi); // use the side to side tracking wheel and get the position here.
    
//just comment the above lines (left and right encoder setting lines), and uncomment below     

    // left_encoder_pos = (ODOMY.get_position()/36000.0)*(2*pi);
    // right_encoder_pos =(ODOMY.get_position()/36000.0)*(2*pi);
    // center_encoder_pos = (ODOMX.get_position()/36000.0)*(2*pi);
    // use this if you have measure wheels or tracking wheels 

    delta_left_encoder_pos = left_encoder_pos - prev_left_encoder_pos;
    delta_right_encoder_pos = right_encoder_pos - prev_right_encoder_pos;
    delta_center_encoder_pos = center_encoder_pos - prev_center_encoder_pos; 

    phi = imu_pos - prev_imu_pos;
    phi = (pi*phi)/180;

    if(phi == 0){
        localX = delta_center_encoder_pos;
        localY = delta_right_encoder_pos; 
    } else {
        localX = (2*sin(phi/2))*((delta_center_encoder_pos/phi) +FORWARD_OFFSET);
        localY = (2*sin(phi/2))*((delta_right_encoder_pos/phi) +SIDEWAYS_OFFEST);
    }

    if (localX == 0 && localY == 0){
        local_polar_angle = 0;
        local_polar_length = 0;
    } else {
        local_polar_angle = atan2(localY, localX);
        local_polar_length = sqrt(pow(localX, 2) + pow(localY, 2));
    }

    global_polar_angle = local_polar_angle - ((pi*prev_imu_pos)/180) - (phi/2);

    deltaX = local_polar_length*cos(global_polar_angle);
    deltaY = local_polar_length*sin(global_polar_angle);

    x_pos += deltaX;
    y_pos += deltaY;

      if (odo_time % 50 == 0 && odo_time % 100 != 0 && odo_time % 150!= 0) {
        con.print(0,0, "LE pos: %f               ", float(x_pos));
      } else if (odo_time % 100 == 0 && odo_time % 150 != 0) {
        con.print (1,0, "RE pos: %f              ", float(y_pos));
      } else if (odo_time % 150 == 0) {
        con.print(2,0, "Pos: %f                ", float(phi));
      }
    //controller print stuff 


odo_time += 10;

}

void boomerang(double xTarget, double yTarget){ // in encoder units. Move robot to the point and check x and Y tracking wheels 
    double hypot = 0;
    double voltage = 0;
    double heading_correction = 0; 
    int btime = 0;
    int timeout = 30000;
    int count = 0;



    while(true){
        Odometry2();
        hypot = sqrt(pow((x_pos - xTarget),2) + pow((y_pos - yTarget),2 ));
        absoluteAngleToTarget = atan2((xTarget - x_pos),(yTarget - y_pos)) * (180/pi);

        if (absoluteAngleToTarget > 180){
            absoluteAngleToTarget = absoluteAngleToTarget - 360;
        }

        position = imu.get_heading();

        if (position > 180){
            position = position - 360;
        } 

        if ((absoluteAngleToTarget < 0) && (position > 0)){
            if ((position - absoluteAngleToTarget) >= 180){
                absoluteAngleToTarget = absoluteAngleToTarget + 360;
                position = imu.get_heading();
            } else {
                turnv = (abs(position) - abs(absoluteAngleToTarget));
            }
        } else if ((absoluteAngleToTarget > 0) && (position < 0)){

        if(abs(turnv) > 90){
            absoluteAngleToTarget = absoluteAngleToTarget - 360;
            hypot = -hypot; 
        }

        if(absoluteAngleToTarget >= 359){
            absoluteAngleToTarget = absoluteAngleToTarget- 360;
        }

        if((absoluteAngleToTarget < 0) && (position > 0)){
            if((position - absoluteAngleToTarget) >= 180 ){
                absoluteAngleToTarget = absoluteAngleToTarget + 360;
                position = imu.get_heading();
            }

        } else if ((absoluteAngleToTarget > 0) && (position > 0)){
            if((absoluteAngleToTarget - position) >= 180){
                position = imu.get_heading();
            }
        }
        }
        setConstants(TURN_KP,TURN_KI,TURN_KD);
        heading_correction = calcPID(absoluteAngleToTarget, position, TURN_INTRGRAL_KI, TURN_MAX_INTEGRAL);

        setConstants(STRAIGHT_KP*5, STRAIGHT_KI*5, STRAIGHT_KD*5);
        voltage = -calcPID2(0, hypot, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        if(voltage > 127){
            voltage = 127;
        }else if(voltage > -127) { 
            voltage = -127;
        }

        if (abs(hypot) < HEADING_CUTOFF){
            heading_correction = 0;
        }

      chasMove((voltage + heading_correction), (voltage + heading_correction),(voltage +  heading_correction),(voltage + heading_correction),(voltage + heading_correction),(voltage + heading_correction));
        
    }
}