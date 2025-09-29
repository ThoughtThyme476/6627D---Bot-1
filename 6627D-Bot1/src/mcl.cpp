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
const double SENSOR_NOISE_STD = 50.0;
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

void initalizeParticles(double x0=0, double y0=0, double theta0=0, bool useGuess=false){
    std::uniform_real_distribution<double> distX(0, FIELD_WIDTH);
    std::uniform_real_distribution<double> distY(0, FIELD_LENGTH);
    std::uniform_real_distribution<double> distTheta(0, 360);
    particles.resize(N_PARTICLES);
        std::normal_distribution<double> gaussX(x0, FIELD_WIDTH/10.0);
        std::normal_distribution<double> gaussY(y0, FIELD_LENGTH/10.0);
        std::normal_distribution<double> gaussTheta(theta0, 15.0);
    for (int i = 0; i < N_PARTICLES; i++){
        if (useGuess){
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
    
    left_enc_degree = LM.get_position();
    right_enc_degree = RM.get_position();
    left_encoder_mm = left_enc_degree/360 *(320.00);
    right_encoder_mm = right_enc_degree/360 *(320.00);
    imu_heading = imu.get_rotation();   

    double delta_left = left_encoder_mm - prev_left_enc;
    double delta_right = right_encoder_mm - prev_right_enc;
    double delta_heading = imu_heading - prev_imu_heading;

    if(delta_heading > 180) delta_heading -= 360;
    if(delta_heading < -180) delta_heading += 360;
    double delta_d = (delta_left + delta_right) / 2.0;

    std::normal_distribution<double> transNoise(0.0, ODOMETRY_NOISE_STD);
    std::normal_distribution<double> rotNoise(0.0, ODOMETRY_NOISE_STD);

    for (Particle& p : particles){
        double noisy_d = delta_d + transNoise(rng);
        double noisy_theta = delta_heading + rotNoise(rng);

        p.theta =+ noisy_theta;
        if(p.theta < 0) p.theta += 360;
        if(p.theta >= 0) p.theta -= 360;

        double theta_rad = p.theta* M_PI /180.00;

        p.x += noisy_d * cos(theta_rad);
        p.y += noisy_d * sin(theta_rad);

        if(p.x < 0) p.x = 0;
        if(p.x > FIELD_WIDTH) p.x = FIELD_WIDTH;
        if(p.x < 0) p.y = 0;
        if(p.x > FIELD_LENGTH) p.y = FIELD_LENGTH;
    }
}

void updateParticlesWithSensor() {
    int32_t z_left = distLeft.get();
    int32_t z_right = distRight.get();
    if(z_left < 0) z_left = FIELD_WIDTH;
    if(z_right < 0) z_right = FIELD_LENGTH;

    double weight_sum = 0.0; 
    double var = SENSOR_NOISE_STD * SENSOR_NOISE_STD;
    for(Particle& p : particles) {

        double pred_left = p.x;
        double pred_right = FIELD_WIDTH - p.x; 

        double err_left = z_left - pred_left;
        double err_right = z_right - pred_right;

        double w_left = exp(-(err_left * err_left) / (2*var));
        double w_right = exp(-(err_right * err_right) / (2*var));
        p.weight = w_left * w_right;
        weight_sum += p.weight;

        
    }
    if (weight_sum > 1e-9){ 
        for (Particle& p : particles) {
            p.weight /= weight_sum;
        }
    } else {
        for (Particle& p : particles){
        p.weight = 1.0 / N_PARTICLES;
        }
    }
}

void resampleParticles() {
    std::vector<Particle> newParticles;

    newParticles.resize(N_PARTICLES);
    std::vector<double> cumw(N_PARTICLES);
    double cum = 0;
    for(int i = 0; i < N_PARTICLES; i++){
        cum += particles[i].weight;
        cumw[i] = cum;
    }

    if (cumw.back() == 0) {

        for (int i = 0; i < N_PARTICLES; i++){
            cumw[i] = (i+1) / (double)N_PARTICLES;
        }
    } 

    std::uniform_real_distribution<double> dist(0.0, 1.0 / N_PARTICLES);
    double start = dist(rng);
    double step = 1.0 / N_PARTICLES;
    double ptr = start;
    int j = 0; 
    for (int i = 0; i < N_PARTICLES; ++i) {
        while (j < N_PARTICLES &&  cumw[j] < ptr) {
            j++;
        }
        if(j == N_PARTICLES) j = N_PARTICLES - 1;
        newParticles[i] = particles[j];
        newParticles[i].weight = 1.0 / N_PARTICLES;
        ptr += step;
    }
    particles = newParticles;

    std::uniform_real_distribution<double> distX(0, FIELD_WIDTH);
    std::uniform_real_distribution<double> distY(0, FIELD_LENGTH);
    std::uniform_real_distribution<double> distTheta(0, 360);
    particles[0].x = distX(rng);
    particles[0].y = distY(rng);
    particles[0].theta = distTheta(rng);
    particles[0].weight = 1.0 / N_PARTICLES;
}

Particle getEstimatedPos() {
    Particle estimate;
    double mean_x = 0, mean_y = 0;
    double mean_sin = 0, mean_cos = 0;
    for (const Particle& p : particles) {
        mean_x += p.x * p.weight;
        mean_y +=  p.y * p.weight;
        mean_cos += cos(p.theta * M_PI / 180.00) * p.weight;
        mean_sin += sin(p.theta * M_PI / 180.00) * p.weight;
    }
    estimate.x = mean_x;
    estimate.y = mean_y;
    estimate.theta = atan2(mean_sin, mean_cos) * 180.0 / M_PI;
    if (estimate.theta < 0) estimate.theta  += 360;
    estimate.weight = 1.0;
    return estimate;
}

//under 200 lines club :}
void IntitializeMCL(){
    
    imu.reset(true);
    LM.tare_position();
    RM.tare_position();
    prev_left_enc = 0;
    prev_right_enc = 0;
    prev_imu_heading = imu.get_rotation();
    
    initalizeParticles(0,0,0, false); // the bool sets the particles to initalize aruond the guess, and since there is no guess, we put false 
}

void RunMCL() {

    while (true) {
    updateParticleWithMotion();
    updateParticlesWithSensor();
    resampleParticles();
    Particle est =  getEstimatedPos();

    }
}