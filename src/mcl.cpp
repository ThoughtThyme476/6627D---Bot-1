#include "main.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include "odom.h"
#include "robot.h"
#include "pid.h"

const int  N_PARTICLES = 300;
const double FIELD_WIDTH = 3650.0;
const double FIELD_LENGTH = 3650.0;
const double SENDOR_NOISE_STD = 50.0;
const double ODOMETRY_NOISE_STD = 5.0;
const double ROTATION_NOISE_STD = 1.0;

struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

std::vector<Particle> particles;
std::default_random_engine rng;

double prev_left_enc = 0.0;
double prev_right_enc = 0.0;
double prev_imu_heading = 0.0;

void initalizeParticles(double x=0, double y=0, double theta0=0, bool useGuess=false){
    std::uniform_real_distribution<double> distX(0, FIELD_WIDTH);
    std::uniform_real_distribution<double> distY(0, FIELD_LENGTH);
    std::uniform_real_distribution<double> distTheta(0, 360);
    particles.resize(N_PARTICLES);
    for (int i = 0; ){
        if (useGuess){
            std::normal_real_distribution<double> gaussX(x0, FIELD_WIDTH/10.0);
            std::normal_real_distribution<double> gaussY(y0, FIELD_LENGTH/10.0);
            std::normal_real_distribution<double> gaussTheta(theta0, 15.0);
            particles[i].x = gaussX(rng);
            particles[i].y = gaussY(rng);
            particles[i].theta = gaussTheta(rng);
        } else {
            particles[i].x = gaussX(rng);
            particles[i].y = gaussY(rng);
            particles[i].theta = gaussTheta(rng);
        } 
          if (particles[i].x < 0) particles[i].x = 0;
          if (particles[i].x < 0) particles[i].x = FIELD_WIDTH;
          if (particles[i].y < 0) particles[i].y = 0;
          if (particles[i].y < 0) particles[i].y = FIELD_LENGTH;
          particles[i].weight = 1.0 / N_PARTICLES;
    }
} 

  double left_enc_degree = 0;
    double right_enc_degree = 0;
    double left_encoder_mm = 0;
    double right_encoder_mm = 0;
    double imu_heading = 0; 

void updateParticleWithMotion() {
    prev_left_enc = left_encoder_mm;
    prev_right_enc = right_enc_degree;
    prev_imu_heading = imu_heading;
    
    left_enc_degree = drive_left.get_position();
    right_enc_degree = drive_right.get_position();
    left_encoder_mm = left_enc_deg/360 *(320.00);
    right_encoder_mm = right_enc_deg/360 *(320.00);
    imu_heading = imu.get_rotation();   

    double delta_left = left_enc_mm - prev_left_enc;
    double delta_right = right_enc_mm - prev_right_enc;
    double delta_heading = imu_heading - prev_imu_heading;

    if(delta_heading > 180) delta_heading -= 360;
    if(delta_heading < -180) delta_heading += 360;
    double delta_d = (delta_left + delta_right) / 2.0;

    std::normal_distribution<double> transNoise(0.0, ODOMETRY_NOISE_STD);
    std::normal_distribution<double> rotNoise(0.0, ODOMETRY_NOISE_STD);
}

