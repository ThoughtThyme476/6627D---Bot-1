#include "pure_pursuit.h"
#include "main.h"
#include "api.h"
#include "auton.h"
#include "pid.h"
#include "pros/misc.h"
#include "pros/motors.h"
#include "robot.h"
#include "odom.h"
#include "main.h"
#include <vector>
#include <cmath>

using namespace std;
using namespace pros;

// Define a struct for points
struct Point {
    double x;
    double y;
};

// Define a struct for intersections
struct Intersection {
    Point point;
    size_t segmentIndex;
    double t;
};

// Global variables
vector<Point> path;
vector<double> pathDistances;
double lookaheadDistance = 220.0; // mm
double robotVelocity = 100.0;     // desired speed (arbitrary units)
double wheelbase = 390.0;         // mm between wheels
double speedLimit = 1.0;          // fraction [0..1] global speed limiter (set to 0.75 for 75%)
double goalTolerance = 10.0;      // mm, stop distance to target
double waypointTolerance = 10.0;  // mm, waypoint acceptance

// Function to initialize the path (for testing/demo)
// void initializePath() {
//     path.clear();
//     path.push_back({0, 0});
//     path.push_back({500, 500});
//     path.push_back({1000, 1000});
//     path.push_back({0, 1000});
//     path.push_back({0, 0});
// }

// Function to precompute distances along the path
void initializePathDistances() {
    pathDistances.clear();
    pathDistances.push_back(0);
    for (size_t i = 1; i < path.size(); i++) {
        double dx = path[i].x - path[i - 1].x;
        double dy = path[i].y - path[i - 1].y;
        double dist = sqrt(dx * dx + dy * dy);
        pathDistances.push_back(pathDistances.back() + dist);
    }
}

// Function to compute circle-line intersections
vector<Intersection> getCircleLineIntersections(double r, Point center, Point p1, Point p2, size_t segmentIndex) {
    vector<Intersection> result;

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;

    double fx = p1.x - center.x;
    double fy = p1.y - center.y;

    double a = dx * dx + dy * dy;
    double b = 2 * (fx * dx + fy * dy);
    double c = fx * fx + fy * fy - r * r;

    double discriminant = b * b - 4 * a * c;
    if (discriminant < 0) {
        return result; // no intersection
    }

    discriminant = sqrt(discriminant);
    double t1 = (-b - discriminant) / (2 * a);
    double t2 = (-b + discriminant) / (2 * a);

    if (t1 >= 0 && t1 <= 1) {
        Point intersection{p1.x + t1 * dx, p1.y + t1 * dy};
        result.push_back({intersection, segmentIndex, t1});
    }
    if (t2 >= 0 && t2 <= 1) {
        Point intersection{p1.x + t2 * dx, p1.y + t2 * dy};
        result.push_back({intersection, segmentIndex, t2});
    }
    return result;
}

// Function to find the goal point
Point findGoalPoint(Point robotPosition, double lookaheadDistance) {
    vector<Intersection> intersections;

    // find all intersections between lookahead circle and path
    for (size_t i = 0; i < path.size() - 1; i++) {
        Point start = path[i];
        Point end = path[i + 1];
        vector<Intersection> points = getCircleLineIntersections(lookaheadDistance, robotPosition, start, end, i);
        intersections.insert(intersections.end(), points.begin(), points.end());
    }

    // choose the one furthest along the path
    double maxProgress = -1;
    Intersection bestIntersection;
    for (const auto &inter : intersections) {
        double segmentStartDistance = pathDistances[inter.segmentIndex];
        double segmentLength = pathDistances[inter.segmentIndex + 1] - segmentStartDistance;
        double progress = segmentStartDistance + inter.t * segmentLength;
        if (progress > maxProgress) {
            maxProgress = progress;
            bestIntersection = inter;
        }
    }

    if (maxProgress >= 0) {
        return bestIntersection.point;
    } else {
        return path.back();
    }
}

// Function to compute curvature
double computeCurvature(Point robotPosition, double robotHeading, Point goalPoint, double lookaheadDistance) {
    double dx = goalPoint.x - robotPosition.x;
    double dy = goalPoint.y - robotPosition.y;

    // Transform to robot frame
    double x = cos(robotHeading) * dx + sin(robotHeading) * dy;
    double y = -sin(robotHeading) * dx + cos(robotHeading) * dy;

    if (fabs(lookaheadDistance) < 1e-6) return 0.0;

    // Correct pure pursuit curvature formula
    double curvature = (2.0 * y) / (lookaheadDistance * lookaheadDistance);
    return curvature;
}

// Function to compute wheel speeds
void computeWheelSpeeds(double curvature, double &leftSpeed, double &rightSpeed) {
    leftSpeed = robotVelocity * (1.0 - (curvature * wheelbase / 2.0));
    rightSpeed = robotVelocity * (1.0 + (curvature * wheelbase / 2.0));
}

// Function to set motor speeds (scaled to PROS move range)
void setMotorSpeeds(double leftSpeed, double rightSpeed) {
    const double MAX_CMD = 127.0;

    // apply user speed limiter first (preserves left/right ratio)
    leftSpeed  *= speedLimit;
    rightSpeed *= speedLimit;

    // find the largest magnitude among desired speeds (after limiter)
    double maxAbs = std::max(fabs(leftSpeed), fabs(rightSpeed));
    double scale = 1.0;
    if (maxAbs > MAX_CMD) {
        scale = MAX_CMD / maxAbs; // scale down only if needed to fit motor command range
    }

    double leftCmd  = leftSpeed * scale;
    double rightCmd = rightSpeed * scale;

    // safety clamp to [-127,127]
    leftCmd  = std::max(std::min(leftCmd,  MAX_CMD), -MAX_CMD);
    rightCmd = std::max(std::min(rightCmd, MAX_CMD), -MAX_CMD);

    LF.move(leftCmd);
    LM.move(leftCmd);
    LB.move(leftCmd);
    RF.move(rightCmd);
    RM.move(rightCmd);
    RB.move(rightCmd);
}

// Pure Pursuit loop (used internally by goTo)
void purePursuitController(double targetX, double targetY) {
    while (true) {
        Odometry2();

        double robotX = x_pos;
        double robotY = y_pos;
        double robotHeading = imu.get_rotation(); // degrees, same as Odometry2 uses
        if (robotHeading > 180) robotHeading -= 360;
        if (robotHeading < -180) robotHeading += 360;
        robotHeading = robotHeading * M_PI / 180.0; // convert to radians
        Point robotPosition = {robotX, robotY};

        // stop when close to target
        if (hypot(robotX - targetX, robotY - targetY) < goalTolerance) {
            setMotorSpeeds(0, 0);
            break;
        }

        Point goalPoint = findGoalPoint(robotPosition, lookaheadDistance);
        double curvature = computeCurvature(robotPosition, robotHeading, goalPoint, lookaheadDistance);

        // small heading correction toward the line-to-goal direction
        double targetHeading = atan2(targetY - robotY, targetX - robotX); // radians
        double headingError = targetHeading - robotHeading;
        while (headingError > M_PI)  headingError -= 2*M_PI;
        while (headingError < -M_PI) headingError += 2*M_PI;
        curvature += headingError * 0.01; // reduce gain if overcorrection

        double leftSpeed, rightSpeed;
        computeWheelSpeeds(curvature, leftSpeed, rightSpeed);
        setMotorSpeeds(leftSpeed, rightSpeed);

        delay(10);
    }
}

// Rotate in place to target heading
void rotateToHeading(double targetHeadingDeg) {
    double kP = 2.0; // tuning gain
    while (true) {
        double error = targetHeadingDeg - imu.get_heading();
        // normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        if (fabs(error) < 5.0) break; // looser tolerance (was 2.0)

        double turn = kP * error;
        turn = std::max(std::min(turn, 127.0), -127.0);

        LF.move(-turn); LM.move(-turn); LB.move(-turn);
        RF.move(turn);  RM.move(turn);  RB.move(turn);

        delay(20);
    }
    setMotorSpeeds(0, 0);
}

// Build path to target point
void setPathToTarget(double targetX, double targetY) {
    path.clear();
    path.push_back({x_pos, y_pos}); // start at current odometry
    path.push_back({targetX, targetY});
    initializePathDistances();
}

// Public command: drive to (x,y) then rotate to heading
void goTo(double targetX, double targetY, double targetHeadingDeg) {
    setPathToTarget(targetX, targetY);
    purePursuitController(targetX, targetY);
    rotateToHeading(targetHeadingDeg);
}

struct Waypoint {
    double x;
    double y;
    double headingDeg; // Desired heading at this point
};

bool stopPP = false;
pros::Task* ppTask = nullptr; // handle to the Pure Pursuit task

// Pure pursuit task function
void purePursuitTask(void* param) {
    auto* waypoints = static_cast<std::vector<Waypoint>*>(param);
    int currentIndex = 0;

    while (!stopPP && currentIndex < waypoints->size()) {
        Odometry2();

        double robotX = x_pos;
        double robotY = y_pos;
        double robotHeading = imu.get_rotation() * M_PI / 180.0;

        Point robotPosition = {robotX, robotY};
        Point goalPoint = findGoalPoint(robotPosition, lookaheadDistance);

        // Heading correction
        double targetHeading = (*waypoints)[currentIndex].headingDeg * M_PI / 180.0;
        double headingError = targetHeading - robotHeading;

        while (headingError > M_PI)  headingError -= 2*M_PI;
        while (headingError < -M_PI) headingError += 2*M_PI;

        double curvature = computeCurvature(robotPosition, robotHeading, goalPoint, lookaheadDistance);
        curvature += headingError * 0.02; // small heading correction gain

        double leftSpeed, rightSpeed;
        computeWheelSpeeds(curvature, leftSpeed, rightSpeed);
        setMotorSpeeds(leftSpeed, rightSpeed);

        // Check proximity to current waypoint
        double dx = (*waypoints)[currentIndex].x - x_pos;
        double dy = (*waypoints)[currentIndex].y - y_pos;
        double dist = sqrt(dx*dx + dy*dy);

        if (dist < waypointTolerance) {
            currentIndex++;
        }

        pros::delay(10);
    }

    // Stop motors at the end
    LF.move(0); LM.move(0); LB.move(0);
    RF.move(0); RM.move(0); RB.move(0);

    stopPP = true;
    delete (std::vector<Waypoint>*)param; // free allocated memory
}

// Start a Pure Pursuit run
void followPath(const std::vector<Waypoint>& waypoints) {
    if (waypoints.empty()) return;

    // Reset path
    path.clear();
    path.push_back({x_pos, y_pos});
    for (const auto& wp : waypoints) {
        path.push_back({wp.x, wp.y});
    }
    initializePathDistances();

    stopPP = false;

    // Copy waypoints to heap so task can access them
    auto* waypointsCopy = new std::vector<Waypoint>(waypoints);

    // Kill old task if still running
   if (ppTask != nullptr) {
    stopPP = true;
    pros::delay(20);
    ppTask->remove();
    delete ppTask;
    ppTask = nullptr;
}

    // Start a new PROS task
    ppTask = new pros::Task(purePursuitTask, waypointsCopy, "PurePursuitTask");
}

