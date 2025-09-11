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

// ========================
// Pure Pursuit Controller
// ========================


// --------------------------
// Robot Pose (global vars from odometry)
// --------------------------
// extern double x_pos;        // mm
// extern double y_pos;        // mm
// extern double imu_pos_radians; // radians

// --------------------------
// Tunable Constants
// --------------------------
constexpr double LOOKAHEAD_DIST = 200.0;  // mm
constexpr double MAX_SPEED      = 100.0;  // motor %
constexpr double TURN_GAIN      = 2.0;    // steering aggressiveness

// --------------------------
// Waypoint Struct
// --------------------------

struct Point {
    double x;
    double y;

    // Default constructor
    Point() : x(0), y(0) {}

    // Constructor for (x, y)
    Point(double _x, double _y) : x(_x), y(_y) {}
};

// Global path vector (extern if needed)
std::vector<Point> path;

int pathSelect = 1; // extern and set before auton

void initPath() {
    if (pathSelect == 1) {
        path = {
            {0, 0},
            {500, 0},
            {1000, 0},
            {1500, 0}
        };
    } 
    else if (pathSelect == 2) {
        path = {
            {0, 0},
            {300, 300},
            {600, 600},
            {900, 300},
            {1200, 0}
        };
    } 
    else if (pathSelect == 3) {
        path = {
            {0, 0},
            {0, 500},
            {0, 1000}
        };
    }
}



// --------------------------
// Find Lookahead Point
// --------------------------
Point findLookaheadPoint() {
    Point lookahead = path.back(); // default to last point

    for (size_t i = 0; i < path.size() - 1; i++) {
        Point p1 = path[i];
        Point p2 = path[i + 1];

        // Segment vector
        double dx = p2.x - p1.x;
        double dy = p2.y - p1.y;

        // Robot to p1
        double fx = p1.x - x_pos;
        double fy = p1.y - y_pos;

        double a = dx*dx + dy*dy;
        double b = 2 * (fx*dx + fy*dy);
        double c = (fx*fx + fy*fy) - LOOKAHEAD_DIST*LOOKAHEAD_DIST;

        double discriminant = b*b - 4*a*c;
        if (discriminant < 0) continue;

        discriminant = sqrt(discriminant);

        double t1 = (-b - discriminant) / (2*a);
        double t2 = (-b + discriminant) / (2*a);

        if (t1 >= 0 && t1 <= 1) {
            lookahead.x = p1.x + t1 * dx;
            lookahead.y = p1.y + t1 * dy;
            return lookahead;
        }
        if (t2 >= 0 && t2 <= 1) {
            lookahead.x = p1.x + t2 * dx;
            lookahead.y = p1.y + t2 * dy;
            return lookahead;
        }
    }

    return lookahead;
}

// --------------------------
// Pure Pursuit Drive Step
// --------------------------
void purePursuitStep() {
    // Update odometry
    Odometry2();

    // Find lookahead
    Point lookahead = findLookaheadPoint();

    // Transform to robot frame
    double dx = lookahead.x - x_pos;
    double dy = lookahead.y - y_pos;

    double localX = dx * cos(-imu_pos_radians) - dy * sin(-imu_pos_radians);
    double localY = dx * sin(-imu_pos_radians) + dy * cos(-imu_pos_radians);

    // Curvature control
    double curvature = (2 * localY) / (LOOKAHEAD_DIST*LOOKAHEAD_DIST);

    // Speed + steering
    double leftSpeed  = MAX_SPEED * (1 - TURN_GAIN * curvature);
    double rightSpeed = MAX_SPEED * (1 + TURN_GAIN * curvature);

    // Normalize if needed
    double maxVal = fmax(fabs(leftSpeed), fabs(rightSpeed));
    if (maxVal > MAX_SPEED) {
        leftSpeed  = (leftSpeed  / maxVal) * MAX_SPEED;
        rightSpeed = (rightSpeed / maxVal) * MAX_SPEED;
    }

    // Convert to voltage [-12700,12700]
    int leftVoltage  = (int)((leftSpeed / 100.0) * 12700);
    int rightVoltage = (int)((rightSpeed / 100.0) * 12700);

    // Apply to chassis
    chasMove(leftVoltage, leftVoltage, leftVoltage, rightVoltage, rightVoltage, rightVoltage);
}

// Global or passed in somewhere
bool stopper = false;  // set to true when you want to end PP


void runPurePursuit() {
    while (true) {
        purePursuitStep();

        // Stop if stopper flag is set
        if (stopper) {
            chasMove(0,0,0,0,0,0); // stop motors
            break; // exit loop, ready for PID or other control
        }

        pros::delay(20);
    }
}



// --------------------------
// Pure Pursuit Main Loop
// --------------------------





// // Define a struct for points
// struct Point {
//     double x;
//     double y;
// };

// // Define a struct for intersections
// struct Intersection {
//     Point point;
//     size_t segmentIndex;
//     double t;
// };

// // Global variables
// vector<Point> path;
// vector<double> pathDistances;
// double lookaheadDistance = 100.0; // Adjust as needed (in mm)
// double robotVelocity = 100.0;     // Desired robot velocity (in voltage)
// double wheelbase = 390.0;         // Distance between left and right wheels (in encoder_units)

// volatile uint32_t control_heartbeat = 0; // incremented by main control loop

// static void watchdog_task_fn(void*){
//     const uint32_t STALL_MS = 200; // if no heartbeat in this window -> stop motors
//     uint32_t last = control_heartbeat;
//     while (true) {
//         delay(50);
//         if (control_heartbeat == last) {
//             // heartbeat stalled -> cut motors
//             LF.move(0); LM.move(0); LB.move(0);
//             RF.move(0); RM.move(0); RB.move(0);
//             // optionally blink LCD / LED and block until human reset
//         } else {
//             last = control_heartbeat;
//         }
//     }
// }

// // call once at startup (e.g. in initialize())
// void start_watchdog() {
//     Task watchdog_task(watchdog_task_fn, (void*)nullptr, TASK_PRIORITY_DEFAULT);
// }

// // Function to initialize the path
// void initializePath() {
//     // Define your path here
//     path.clear();
//     path.push_back({0, 0});
//     path.push_back({500, 500});
//     path.push_back({1000, 1000});
//     path.push_back({0, 0});

//     // Recompute pathDistances any time the path changes
//     // initializePathDistances();
// }

// // Function to precompute distances along the path
// void initializePathDistances() {
//     pathDistances.clear();
//     pathDistances.push_back(0);
//     for (size_t i = 1; i < path.size(); i++) {
//         double dx = path[i].x - path[i - 1].x;
//         double dy = path[i].y - path[i - 1].y;
//         double dist = sqrt(dx * dx + dy * dy);
//         pathDistances.push_back(pathDistances.back() + dist);
//     }
// }

// // --- Tell robot to go to a single point (x, y) in mm ---
// void goTo(double targetX, double targetY) {
//     path.clear();
//     path.push_back({x_pos, y_pos});     // current position from odometry
//     path.push_back({targetX, targetY}); // desired point
//     initializePathDistances();
// }


// // Function to compute circle-line intersections
// vector<Intersection> getCircleLineIntersections(double r, Point center, Point p1, Point p2, size_t segmentIndex) {
//     vector<Intersection> result;

//     double dx = p2.x - p1.x;
//     double dy = p2.y - p1.y;

//     double fx = p1.x - center.x;
//     double fy = p1.y - center.y;

//     double a = dx * dx + dy * dy;
//     double b = 2 * (fx * dx + fy * dy);
//     double c = fx * fx + fy * fy - r * r;

//     double discriminant = b * b - 4 * a * c;

//     if (discriminant < 0) {
//         return result; // No intersection
//     }

//     discriminant = sqrt(discriminant);

//     double t1 = (-b - discriminant) / (2 * a);
//     double t2 = (-b + discriminant) / (2 * a);

//     if (t1 >= 0 && t1 <= 1) {
//         Point intersection;
//         intersection.x = p1.x + t1 * dx;
//         intersection.y = p1.y + t1 * dy;
//         result.push_back({intersection, segmentIndex, t1});
//     }

//     if (t2 >= 0 && t2 <= 1) {
//         Point intersection;
//         intersection.x = p1.x + t2 * dx;
//         intersection.y = p1.y + t2 * dy;
//         result.push_back({intersection, segmentIndex, t2});
//     }

//     return result;
// }

// // Function to find the goal point
// Point findGoalPoint(Point robotPosition, double lookaheadDistance) {
//     // safety guards
//     if (path.size() < 2) {
//         // nothing to follow, return robot position or last point
//         if (!path.empty()) return path.back();
//         return robotPosition;
//     }
//     if (pathDistances.size() != path.size()) {
//         initializePathDistances();
//     }

//     vector<Intersection> intersections;
//     // Find all intersections between the lookahead circle and path segments
//     for (size_t i = 0; i < path.size() - 1; i++) {
//         Point start = path[i];
//         Point end = path[i + 1];

//         vector<Intersection> points = getCircleLineIntersections(
//             lookaheadDistance, robotPosition, start, end, i);

//         intersections.insert(intersections.end(), points.begin(), points.end());
//     }

//     // if no intersections, return last path point (safe)
//     if (intersections.empty()) return path.back();

//     // Select the intersection point that is the furthest along the path
//     double maxProgress = -1;
//     Intersection bestIntersection;

//     for (const auto& inter : intersections) {
//         double segmentStartDistance = pathDistances[inter.segmentIndex];
//         double segmentLength = pathDistances[inter.segmentIndex + 1] - segmentStartDistance;
//         double progress = segmentStartDistance + inter.t * segmentLength;

//         if (progress > maxProgress) {
//             maxProgress = progress;
//             bestIntersection = inter;
//         }
//     }

//     if (maxProgress >= 0) {
//         return bestIntersection.point;
//     } else {
//         // No valid intersection found; return the last point in the path
//         return path.back();
//     }
// }

// // Function to compute the curvature
// double computeCurvature(Point robotPosition, double robotHeading, Point goalPoint, double lookaheadDistance) {
//     if (lookaheadDistance <= 0) return 0.0;

//     double dx = goalPoint.x - robotPosition.x;
//     double dy = goalPoint.y - robotPosition.y;

//     // Transform the goal point to the robot's coordinate frame
//     double x = cos(robotHeading) * dx + sin(robotHeading) * dy;   // forward
//     double y = -sin(robotHeading) * dx + cos(robotHeading) * dy;  // lateral (to robot's left)

//     // Use lateral offset (y) for pure-pursuit curvature: k = 2*y / L^2
//     if (fabs(y) < 1e-6) {
//         return 0.0;
//     }

//     double curvature = (2.0 * y) / (lookaheadDistance * lookaheadDistance);

//     return curvature;
// }

// // Function to compute wheel speeds based on curvature
// void computeWheelSpeeds(double curvature, double& leftSpeed, double& rightSpeed) {
//     leftSpeed = robotVelocity * (1.0 - (curvature * wheelbase / 2.0));
//     rightSpeed = robotVelocity * (1.0 + (curvature * wheelbase / 2.0));
// }

// // Function to set motor speeds
// void setMotorSpeeds(double leftSpeed, double rightSpeed) {
//     // safety: reject NaN/inf and extreme values
//     if (!std::isfinite(leftSpeed) || !std::isfinite(rightSpeed)) {
//         LF.move(0); LM.move(0); LB.move(0);
//         RF.move(0); RM.move(0); RB.move(0);
//         return;
//     }

//     // clamp linear speeds to some sane mm/s range before converting
//     const double MAX_WHEEL_MM_S = 3000.0; // tune for your robot
//     leftSpeed  = std::max(std::min(leftSpeed,  MAX_WHEEL_MM_S), -MAX_WHEEL_MM_S);
//     rightSpeed = std::max(std::min(rightSpeed, MAX_WHEEL_MM_S), -MAX_WHEEL_MM_S);

//     const double wheelCircumference = 3.25 * 25.4 * M_PI; // mm per wheel rev
//     const double gearMotorToWheel = 36.0/48.0;            // same convention as Odometry2 (motor->wheel)
//     const double maxMotorRPM = 600.0;

//     // wheel RPM from mm/s
//     double leftWheelRPM  = (leftSpeed  / wheelCircumference) * 60.0;
//     double rightWheelRPM = (rightSpeed / wheelCircumference) * 60.0;

//     // convert wheel RPM -> motor RPM (motorRPM = wheelRPM / gearMotorToWheel)
//     double leftMotorRPM  = leftWheelRPM  / gearMotorToWheel;
//     double rightMotorRPM = rightWheelRPM / gearMotorToWheel;

//     // clamp to motor capability
//     leftMotorRPM  = std::max(std::min(leftMotorRPM,  maxMotorRPM), -maxMotorRPM);
//     rightMotorRPM = std::max(std::min(rightMotorRPM, maxMotorRPM), -maxMotorRPM);

//     LF.move_velocity(leftMotorRPM);
//     LM.move_velocity(leftMotorRPM);
//     LB.move_velocity(leftMotorRPM);
//     RF.move_velocity(rightMotorRPM);
//     RM.move_velocity(rightMotorRPM);
//     RB.move_velocity(rightMotorRPM);

// }

// // The main Pure Pursuit controller loop
// void purePursuitController() {
//     while (true) {
//         control_heartbeat++; // keep watchdog happy
//         Odometry2();
//         // Get robot's current position and heading
//         double robotX = x_pos;
//         double robotY = y_pos;
//         double robotHeading = imu.get_heading() * M_PI / 180.0; /// In radians

//         Point robotPosition = {robotX, robotY};

//         // Find the goal point
//         Point goalPoint = findGoalPoint(robotPosition, lookaheadDistance);

//         // Compute the curvature
//         double curvature = computeCurvature(robotPosition, robotHeading, goalPoint, lookaheadDistance);

//         // Compute wheel speeds
//         double leftSpeed, rightSpeed;
//         computeWheelSpeeds(curvature, leftSpeed, rightSpeed);

    
//         setMotorSpeeds(leftSpeed, rightSpeed);

//         delay(10);
//     }
// }


// //add motion tracking