#include "api.h"
#include "main.h"
#include "pid.h"
#include "robot.h"
#include "auton.h"
#include "odom.h"
#include <vector>
#include <algorithm>

using namespace pros;
using namespace std;

// Constants
constexpr double PI = 3.14159265359;
constexpr double TICKS_PER_REV_MOTOR = 360.0;   // VEX motor encoder reports degrees
constexpr double TICKS_PER_REV_ROTO = 36000.0;  // rotation sensor reports centidegrees
constexpr double WHEEL_RADIUS_X_IN = 3.25;      // drive wheel radius (forward movement) [inches]
constexpr double WHEEL_RADIUS_Y_IN = 2.0;       // tracking wheel radius (lateral movement) [inches]
constexpr double GEAR_X = 36.0/48.0;            // motor gear ratio (motor:wheel)
constexpr double GEAR_Y = 1.0;                  // tracking wheel gear ratio
constexpr double INCH_TO_MM = 25.4;
constexpr double POSITION_TOLERANCE = 5.0;       // mm
constexpr double TIMEOUT_MS = 3000;             // 3 second timeout

// Global state variables (minimized)
struct OdomState {
    float x_pos = 0;
    float y_pos = 0;
    float heading = 0;
    uint32_t update_time = 0;
};
OdomState state;

struct TuningParams {
    double straight_kp;
    double straight_ki;
    double straight_kd;
    double turn_kp;
    double turn_ki;
    double turn_kd;
    double score;
};

void setPosition(float xcoord, float ycoord, float heading) {
    state.x_pos = xcoord;
    state.y_pos = ycoord;
    state.heading = heading;
    imu.tare_rotation();  // Zero the IMU at starting position
}

void applyTuningParams(const TuningParams& p) {
    setConstants(p.straight_kp * 5, p.straight_ki * 5, p.straight_kd * 5); // For straight movement
    setConstants(p.turn_kp, p.turn_ki, p.turn_kd);  // For turning
}

void Odometry2() {
    // Convert constants to mm
    const double WHEEL_RADIUS_X_MM = WHEEL_RADIUS_X_IN * INCH_TO_MM;
    const double WHEEL_RADIUS_Y_MM = WHEEL_RADIUS_Y_IN * INCH_TO_MM;
    const double DEG2RAD = PI / 180.0;
    const double THRESH = IMU_THERSHOLD * DEG2RAD;

    // Get current encoder positions
    static double prev_forward_pos = 0;
    static double prev_lateral_pos = 0;
    static double prev_imu_pos = 0;

    // Read current positions
    double left_rev = (LF.get_position() / TICKS_PER_REV_MOTOR) * GEAR_X;
    double right_rev = (RF.get_position() / TICKS_PER_REV_MOTOR) * GEAR_X;
    double forward_pos = ((left_rev + right_rev) / 2.0) * (2.0 * PI * WHEEL_RADIUS_X_MM);
    double lateral_pos = (roto.get_position() / TICKS_PER_REV_ROTO) * GEAR_Y * (2.0 * PI * WHEEL_RADIUS_Y_MM);
    
    // Get IMU heading in radians
    double imu_pos = imu.get_rotation() * DEG2RAD;

    // Calculate deltas
    double dx_wheel = forward_pos - prev_forward_pos;
    double dy_wheel = lateral_pos - prev_lateral_pos;
    double d_theta = imu_pos - prev_imu_pos;

    // Normalize heading change to [-π, π]
    if (d_theta > PI) d_theta -= 2.0 * PI;
    if (d_theta < -PI) d_theta += 2.0 * PI;

    // Calculate local displacement
    double local_x, local_y;
    if (fabs(d_theta) < THRESH) {
        // Straight motion approximation
        local_x = dx_wheel;
        local_y = dy_wheel;
    } else {
        // Arc motion calculation
        double s = 2.0 * sin(d_theta / 2.0);
        local_x = s * (dx_wheel / d_theta);
        local_y = s * (dy_wheel / d_theta);
    }

    // Transform to global coordinates
    double theta_mid = prev_imu_pos + (d_theta / 2.0);
    double dx = local_x * cos(theta_mid) - local_y * sin(theta_mid);
    double dy = local_x * sin(theta_mid) + local_y * cos(theta_mid);

    // Update state
    state.x_pos += dx;
    state.y_pos += dy;
    state.heading = imu_pos;

    // Store previous positions
    prev_forward_pos = forward_pos;
    prev_lateral_pos = lateral_pos;
    prev_imu_pos = imu_pos;

    // Debug output (every 50ms rotation)
    if (state.update_time % 150 == 0) {
        con.print(0, 0, "X: %.1f Y: %.1f", state.x_pos, state.y_pos);
        con.print(1, 0, "H: %.1f deg", state.heading * 180.0 / PI);
    }

    state.update_time += 10;
}

bool boomerang(double xTarget, double yTarget) {
    uint32_t start_time = pros::millis();
    
    while (true) {
        // Check timeout
        if (pros::millis() - start_time > TIMEOUT_MS) {
            return false;
        }

        Odometry2();

        // Calculate distance and angle to target
        double dx = xTarget - state.x_pos;
        double dy = yTarget - state.y_pos;
        double distance = sqrt(dx*dx + dy*dy);
        double angle_to_target = atan2(dy, dx) * 180.0/PI;

        // Normalize angles to [-180, 180]
        while (angle_to_target > 180) angle_to_target -= 360;
        while (angle_to_target < -180) angle_to_target += 360;

        // Check if we've reached the target
        if (distance < POSITION_TOLERANCE) {
            return true;
        }

        // Calculate heading correction
        setConstants(TURN_KP, TURN_KI, TURN_KD);
        double heading_correction = calcPID(angle_to_target, state.heading * 180.0/PI, 
                                         TURN_INTRGRAL_KI, TURN_MAX_INTEGRAL);

        // Calculate drive voltage
        setConstants(STRAIGHT_KP*5, STRAIGHT_KI*5, STRAIGHT_KD*5);
        double voltage = -calcPID2(0, distance, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);

        // Clamp voltage
        voltage = std::clamp(voltage, -127.0, 127.0);

        // Disable heading correction near target
        if (distance < HEADING_CUTOFF) {
            heading_correction = 0;
        }

        // Apply motor movements
        double final_voltage = voltage + heading_correction;
        chasMove(final_voltage, final_voltage, final_voltage, 
                 final_voltage, final_voltage, final_voltage);

        pros::delay(10);
    }
}

void autoTune() {
    std::vector<TuningParams> params;
    
    // Test different PID combinations
    for(double kp = 0.1; kp <= 1.0; kp += 0.1) {
        for(double kd = 0.0; kd <= 0.2; kd += 0.05) {
            TuningParams p = {
                kp, 0.0, kd,     // Straight PID
                kp/2, 0.0, kd/2, // Turn PID
                0.0              // Score
            };
            
            // Reset position
            setPosition(0, 0, 0);
            uint32_t start_time = pros::millis();
            
            // Run square pattern test
            std::vector<std::pair<double, double>> test_points = {
                {1000, 0}, {1000, 1000}, {0, 1000}, {0, 0}
            };
            
            double total_error = 0;
            
            for(const auto& point : test_points) {
                applyTuningParams(p);
                bool success = boomerang(point.first, point.second);
                if(!success) {
                    total_error += 10000; // Penalty for timeout
                    continue;
                }
                
                total_error += sqrt(pow(point.first - state.x_pos, 2) + 
                                  pow(point.second - state.y_pos, 2));
            }
            
            p.score = total_error;
            params.push_back(p);
            
            // Display progress
            pros::lcd::print(0, "Testing KP: %.2f KD: %.2f", kp, kd);
            pros::lcd::print(1, "Error: %.2f", total_error);
        }
    }
    
    // Find best parameters
    auto best = std::min_element(params.begin(), params.end(),
        [](const TuningParams& a, const TuningParams& b) {
            return a.score < b.score;
        });
        
    // Display and save best results
    pros::lcd::print(0, "Best Parameters Found:");
    pros::lcd::print(1, "Straight KP: %.3f", best->straight_kp);
    pros::lcd::print(2, "Straight KD: %.3f", best->straight_kd);
    pros::lcd::print(3, "Turn KP: %.3f", best->turn_kp);
    pros::lcd::print(4, "Turn KD: %.3f", best->turn_kd);
    pros::lcd::print(5, "Score: %.2f", best->score);
}