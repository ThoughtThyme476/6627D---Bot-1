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
}