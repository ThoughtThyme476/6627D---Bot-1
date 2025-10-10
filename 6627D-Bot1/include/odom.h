#include "api.h"
#include "main.h"

#ifndef ODOH
#define ODOH

// Add TuningParams struct definition before the defines
struct TuningParams {
    double straight_kp;
    double straight_ki;
    double straight_kd;
    double turn_kp;
    double turn_ki;
    double turn_kd;
    double score;
};

#define FORWARD_OFFSET 0
#define IMU_THERSHOLD 2.0 // TUNE
#define HEADING_CUTOFF 0.0001745329
#define SIDEWAYS_OFFSET 5.5 // Remeasure and tune
extern void Odometry();
extern void Odometry2();
extern void driveToPoint(double xTarget, double yTarget, double preferredHeading);
extern bool boomerang(double xTarget, double yTarget);
extern void setPosition(float xcoord, float ycoord, float heading);
extern void autoTune(); // Added function declaration
void applyTuningParams(const TuningParams& p);
extern float x_pos;
extern float y_pos;
extern float imu_pos;
extern float imu_pos_radians;


#endif