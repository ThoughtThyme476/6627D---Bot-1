#include "pure_pursuit.h"
#include "main.h"
#include "api.h"
#include "auton.h"
#include "pid.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"
#include "odom.h"
#include <vector>
#include <cmath>

using namespace std;
using namespace pros;

// Debug logging flag
const bool DEBUG_MODE = true;

// Constants
constexpr double LOOKAHEAD_DIST = 200.0;  // mm
constexpr double MAX_SPEED = 100.0;       // motor %
double TURN_GAIN = 2.0;         // steering aggressiveness
constexpr double MIN_DISTANCE_TO_TARGET = 50.0; // mm
constexpr double ACCELERATION_RATE = 5.0;  // Speed change per iteration
constexpr bool AUTO_TUNE = false;  // Enable/disable auto-tuning
constexpr int TUNE_ITERATIONS = 5; // Number of tuning attempts
constexpr double TUNE_INCREMENT = 0.2; // How much to adjust gains

// Point struct with debug output
struct Point {
    double x;
    double y;

    Point() : x(0), y(0) {}
    Point(double _x, double _y) : x(_x), y(_y) {}
    
    void print(const char* label = "") const {
        if (DEBUG_MODE) {
            printf("%s(%.1f, %.1f)\n", label, x, y);
        }
    }
};

// Global variables
std::vector<Point> path;
int pathSelect = 1;
bool stopper = false;
double currentSpeed = 0.0;
double best_turn_gain = TURN_GAIN;
double best_error = 999999.0;
double path_error = 0.0;

// Debug helper functions
void printRobotState() {
    if (!DEBUG_MODE) return;
    printf("Robot State:\n");
    printf("Position: (%.1f, %.1f)\n", x_pos, y_pos);
    printf("Heading: %.1f°\n", imu_pos_radians * 180/M_PI);
    printf("Current Speed: %.1f%%\n", currentSpeed);
}

void printPathInfo() {
    if (!DEBUG_MODE) return;
    printf("Path Info:\n");
    printf("Path Size: %d points\n", (int)path.size());
    printf("Current Path Select: %d\n", pathSelect);
}

void initPath() {
    path.clear();
    
    switch (pathSelect) {
        case 1:
            path = {{0, 0}, {500, 0}, {1000, 0}, {1500, 0}};
            break;
        case 2:
            path = {{0, 0}, {300, 300}, {600, 600}, {900, 300}, {1200, 0}};
            break;
        case 3:
            path = {{0, 0}, {0, 500}, {0, 1000}};
            break;
        default:
            printf("ERROR: Invalid path selection %d\n", pathSelect);
            path = {{0, 0}}; // Safe default
    }
    
    if (DEBUG_MODE) {
        printf("Initialized path with %d points\n", (int)path.size());
    }
}

Point findLookaheadPoint() {
    if (path.empty()) {
        printf("ERROR: Path is empty in findLookaheadPoint()\n");
        return Point(x_pos, y_pos);
    }

    Point lookahead = path.back();
    bool foundIntersection = false;

    for (size_t i = 0; i < path.size() - 1; i++) {
        Point p1 = path[i];
        Point p2 = path[i + 1];

        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;
        double fx = p1.x - x_pos;
        double fy = p1.y - y_pos;

        double a = dx*dx + dy*dy;
        double b = 2 * (fx*dx + fy*dy);
        double c = (fx*fx + fy*fy) - LOOKAHEAD_DIST*LOOKAHEAD_DIST;

        double discriminant = b*b - 4*a*c;
        
        if (discriminant >= 0) {
            discriminant = sqrt(discriminant);
            double t1 = (-b - discriminant) / (2*a);
            double t2 = (-b + discriminant) / (2*a);

            if (t1 >= 0 && t1 <= 1) {
                lookahead.x = p1.x + t1 * dx;
                lookahead.y = p1.y + t1 * dy;
                foundIntersection = true;
                break;
            }
            if (t2 >= 0 && t2 <= 1) {
                lookahead.x = p1.x + t2 * dx;
                lookahead.y = p1.y + t2 * dy;
                foundIntersection = true;
                break;
            }
        }
    }

    if (DEBUG_MODE) {
        printf("Lookahead Point: (%.1f, %.1f) %s\n", 
               lookahead.x, lookahead.y,
               foundIntersection ? "FOUND" : "USING ENDPOINT");
    }

    return lookahead;
}

bool isPathComplete() {
    Point finalPoint = path.back();
    double dx = finalPoint.x - x_pos;
    double dy = finalPoint.y - y_pos;
    double distance = sqrt(dx*dx + dy*dy);
    
    if (DEBUG_MODE) {
        printf("Distance to target: %.1f mm\n", distance);
    }
    
    return distance < MIN_DISTANCE_TO_TARGET;
}

void purePursuitStep() {
    if (DEBUG_MODE) {
        printRobotState();
    }

    // Update odometry
    Odometry2();

    // Check completion
    if (isPathComplete()) {
        if (DEBUG_MODE) printf("Path complete!\n");
        stopper = true;
        return;
    }
    
    // Find lookahead
    Point lookahead = findLookaheadPoint();

    // Transform to robot frame
    double dx = lookahead.x - x_pos;
    double dy = lookahead.y - y_pos;
    double localX = dx * cos(-imu_pos_radians) - dy * sin(-imu_pos_radians);
    double localY = dx * sin(-imu_pos_radians) + dy * cos(-imu_pos_radians);

    // Curvature control
    double curvature = (2 * localY) / (LOOKAHEAD_DIST*LOOKAHEAD_DIST);

    // Speed ramping
    if (currentSpeed < MAX_SPEED) {
        currentSpeed += ACCELERATION_RATE;
    }
    currentSpeed = std::min(currentSpeed, MAX_SPEED);

    // Speed + steering
    double leftSpeed = currentSpeed * (1 - TURN_GAIN * curvature);
    double rightSpeed = currentSpeed * (1 + TURN_GAIN * curvature);

    // Normalize speeds
    double maxVal = fmax(fabs(leftSpeed), fabs(rightSpeed));
    if (maxVal > MAX_SPEED) {
        leftSpeed = (leftSpeed / maxVal) * MAX_SPEED;
        rightSpeed = (rightSpeed / maxVal) * MAX_SPEED;
    }

    // Convert to voltage
    int leftVoltage = (int)((leftSpeed / 100.0) * 12700);
    int rightVoltage = (int)((rightSpeed / 100.0) * 12700);

    if (DEBUG_MODE) {
        printf("Speeds (L,R): %.1f%%, %.1f%%\n", leftSpeed, rightSpeed);
        printf("Voltages (L,R): %d, %d\n", leftVoltage, rightVoltage);
    }

    // Apply to chassis
    chasMove(leftVoltage, leftVoltage, leftVoltage, 
             rightVoltage, rightVoltage, rightVoltage);
}

double calculatePathError() {
    double total_error = 0;
    for (const auto& point : path) {
        double dx = point.x - x_pos;
        double dy = point.y - y_pos;
        total_error += sqrt(dx*dx + dy*dy);
    }
    return total_error;
}

void runPurePursuit() {
    if (AUTO_TUNE) {
        printf("Starting auto-tune process...\n");
        
        double test_gain = TURN_GAIN;
        for (int i = 0; i < TUNE_ITERATIONS; i++) {
            TURN_GAIN = test_gain;
            
            // Run path
            currentSpeed = 0.0;
            stopper = false;
            path_error = 0;
            
            while (!stopper) {
                purePursuitStep();
                path_error += calculatePathError();
                pros::delay(20);
            }
            
            // Check if this gain was better
            if (path_error < best_error) {
                best_error = path_error;
                best_turn_gain = test_gain;
                printf("New best gain: %.2f (error: %.2f)\n", 
                       best_turn_gain, best_error);
            }
            
            test_gain += TUNE_INCREMENT;
        }
        
        TURN_GAIN = best_turn_gain;
        printf("Auto-tune complete. Best gain: %.2f\n", best_turn_gain);
    }
    
    // Regular path following
    currentSpeed = 0.0;
    stopper = false;
    
    if (DEBUG_MODE) {
        printf("Starting Pure Pursuit with gain: %.2f\n", TURN_GAIN);
        printPathInfo();
    }

    while (!stopper) {
        purePursuitStep();

        if (stopper) {
            if (DEBUG_MODE) printf("Stopping Pure Pursuit\n");
            chasMove(0,0,0,0,0,0);
            break;
        }

        pros::delay(20);
    }
}