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



void Odometry2() {
    // --- Constants ---
    constexpr double TICKS_PER_REV_MOTOR = 360.0;   // motor reports in degrees
    constexpr double TICKS_PER_REV_ROTO  = 36000.0; // rotation sensor reports centidegrees
    constexpr double WHEEL_RADIUS_X = 1.625;        // drive wheel radius (forward movement) [inches]
    constexpr double WHEEL_RADIUS_Y = 2.0;          // tracking wheel radius (lateral movement) [inches]
    constexpr double GEAR_X = 36.0/48.0;            // motor gear ratio (motor:wheel)
    constexpr double GEAR_Y = 1.0;                  // tracking wheel gear ratio
    constexpr double INCH_TO_MM = 25.4;

    const double DEG2RAD = pi / 180.0;
    const double THRESH  = IMU_THERSHOLD * DEG2RAD;

    // --- Save old IMU state ---
    prev_imu_pos = imu_pos; 
    imu_pos = imu.get_rotation() + startingHeading; // degrees
    imu_pos_radians = imu_pos * DEG2RAD;            // radians 

    // Normalize IMU angle to [-180,180]
    if (imu_pos > 180) imu_pos -= 360;
    if (imu_pos < -180) imu_pos += 360;

    // --- Save old encoder state ---
    prev_left_encoder_pos   = left_encoder_pos;
    prev_right_encoder_pos  = right_encoder_pos;

    // --- Forward/backward distance from drive motor encoders ---
    double left_rev  = (LF.get_position() / TICKS_PER_REV_MOTOR) * GEAR_X;
    double right_rev = (RF.get_position() / TICKS_PER_REV_MOTOR) * GEAR_X;
    double revX      = (left_rev + right_rev) / 2.0;  // average L+R

    left_encoder_pos  = revX * (2.0 * pi * WHEEL_RADIUS_X) * INCH_TO_MM; // mm

    // --- Lateral distance from tracking wheel (roto) ---
    double revY = (roto.get_position() / TICKS_PER_REV_ROTO) * GEAR_Y;
    right_encoder_pos = revY * (2.0 * pi * WHEEL_RADIUS_Y) * INCH_TO_MM; // mm

    // --- Delta wheel displacements ---
    double dXwheel = left_encoder_pos  - prev_left_encoder_pos;
    double dYwheel = right_encoder_pos - prev_right_encoder_pos;

    // --- Heading change ---
    phi = imu_pos - prev_imu_pos;   // Δheading in degrees
    phi = phi * DEG2RAD;            // to radians

    // --- Local displacement (robot frame) ---
    double localX, localY;
    if (fabs(phi) < THRESH) {
        localX = dXwheel + FORWARD_OFFSET  * phi;
        localY = dYwheel + SIDEWAYS_OFFSET * phi;
    } else {
        double s = 2.0 * sin(phi / 2.0);
        localX = s * ((dXwheel / phi) + FORWARD_OFFSET);
        localY = s * ((dYwheel / phi) + SIDEWAYS_OFFSET);
    }

    // --- Transform into global frame ---
    double theta_mid = (prev_imu_pos * DEG2RAD) + (phi / 2.0);
    double dx = localX * cos(theta_mid) - localY * sin(theta_mid);
    double dy = localX * sin(theta_mid) + localY * cos(theta_mid);

    x_pos += dx;
    y_pos += dy;

    // --- Debug printing ---
    if (odo_time % 50 == 0 && odo_time % 100 != 0 && odo_time % 150 != 0) {
        con.print(0,0, "X pos: %.1f mm     ", float(x_pos));
    } else if (odo_time % 100 == 0 && odo_time % 150 != 0) {
        con.print(1,0, "Y pos: %.1f mm     ", float(y_pos));
    } else if (odo_time % 150 == 0) {
        con.print(2,0, "Heading: %.2f rad ", float(imu_pos_radians));
    }

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