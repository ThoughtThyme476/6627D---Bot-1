#include "main.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include "odom.h"
#include "robot.h"
#include "pid.h"
#include "pros/distance.hpp"

namespace SensorTracker {
    struct Config {
        //uses mm for all distances
        double fieldHalfX = 3650.0 / 2.0;
        double fieldHalfY = 3650.0 / 2.0;
        
        double offsetFront = 0.0;
        double offsetBack = 0.0;
        double offsetLeft = 0.0;
        double offsetRight = 0.0;
        
        double minValid = 20.0;
        double maxValid = 2000.0;
        double smoothAlpha = 0.3;
        bool requirePair = false;
    };

    static Config g_config;
    static double g_xPos = 0.0;
    static double g_yPos = 0.0;
    static bool g_validX = false;
    static bool g_validY = false;

    static bool isValidReading(int32_t mm) {
        return mm >= g_config.minValid && mm <= g_config.maxValid;
    }

    void init(const Config& cfg = Config()) {
        g_config = cfg;
        g_xPos = 0.0;
        g_yPos = 0.0;
        g_validX = false;
        g_validY = false;
    }

    bool update(int32_t frontDist, int32_t backDist, int32_t leftDist, int32_t rightDist) {
        // X position from left/right sensors
        double newX = NAN;
        bool haveX = false;
        
        if (isValidReading(leftDist) && isValidReading(rightDist)) {
            double xFromLeft = -g_config.fieldHalfX + leftDist + g_config.offsetLeft;
            double xFromRight = g_config.fieldHalfX - rightDist - g_config.offsetRight;
            newX = (xFromLeft + xFromRight) * 0.5;
            haveX = true;
        }
        
        // Y position from front/back sensors
        double newY = NAN;
        bool haveY = false;
        
        if (isValidReading(frontDist) && isValidReading(backDist)) {
            double yFromFront = g_config.fieldHalfY - frontDist - g_config.offsetFront;
            double yFromBack = -g_config.fieldHalfY + backDist + g_config.offsetBack;
            newY = (yFromFront + yFromBack) * 0.5;
            haveY = true;
        }

        // Apply smoothing
        if (haveX) {
            g_xPos = g_config.smoothAlpha * newX + (1.0 - g_config.smoothAlpha) * g_xPos;
            g_validX = true;
        } else {
            g_validX = false;
        }
        
        if (haveY) {
            g_yPos = g_config.smoothAlpha * newY + (1.0 - g_config.smoothAlpha) * g_yPos;
            g_validY = true;
        } else {
            g_validY = false;
        }

        return (haveX || haveY);
    }

    double getX() { return g_xPos; }
    double getY() { return g_yPos; }
    bool isValidX() { return g_validX; }
    bool isValidY() { return g_validY; }
}

// Declare sensors with proper port numbers
static pros::Distance distFront(1);  // Change these port numbers
static pros::Distance distBack(2);   // to match your actual
static pros::Distance distLeft(3);   // sensor port connections
static pros::Distance distRight(4);  // on the brain

static uint32_t lastPrintTime = 0;

void setupSensors() {
    // Configure tracker
    SensorTracker::Config cfg;
    cfg.offsetFront = 100.0;    // 100mm from robot center to front sensor
    cfg.offsetBack = 100.0;     // Distance from robot center to back sensor (mm)
    cfg.offsetLeft = 100.0;     // Distance from robot center to left sensor (mm)
    cfg.offsetRight = 100.0;    // Distance from robot center to right sensor (mm)
    cfg.minValid = 20.0;        // 20mm minimum valid reading
    cfg.maxValid = 2000.0;      // 2000mm maximum valid reading
    cfg.smoothAlpha = 0.3;      // Smoothing factor (0.1 = more smooth, 0.9 = more responsive)
    
    SensorTracker::init(cfg);
}

void updateSensors() {
    // Get current sensor readings
    int32_t front = distFront.get();
    int32_t back = distBack.get();
    int32_t left = distLeft.get();
    int32_t right = distRight.get();

    // Update position estimate
    SensorTracker::update(front, back, left, right);
    
    // Print debug info every 250ms
    uint32_t now = pros::millis();
    if (now - lastPrintTime >= 250) {
        lastPrintTime = now;
        
        pros::Controller master(pros::E_CONTROLLER_MASTER);
        
        // Print position if valid
        if (SensorTracker::isValidX() && SensorTracker::isValidY()) {
            master.print(0, 0, "X:%.0f Y:%.0f", 
                SensorTracker::getX(),
                SensorTracker::getY());
        } else {
            master.print(0, 0, "Pos Invalid   ");
        }
        
        // Always print raw readings for debugging
        master.print(1, 0, "F:%d B:%d L:%d R:%d",
            front, back, left, right);
    }

    // Add delay to prevent CPU hogging
    pros::delay(10);  // 10ms delay = ~100Hz update rate
}

namespace Navigation {
    // Tuning constants
    constexpr double DRIVE_KP = 0.1;      // Speed proportional gain (tune this)
    constexpr double Heading_KP = 2.0;     // Turning proportional gain (tune this)
    constexpr double MIN_DRIVE_POWER = 20; // Minimum power to move
    constexpr double MAX_DRIVE_POWER = 90; // Maximum power (0-127)
    constexpr double POSITION_TOLERANCE = 50.0; // 50mm tolerance for arrival
    constexpr double SLOWDOWN_DISTANCE = 500.0; // Start slowing 500mm from target
    constexpr double HEADING_TOLERANCE = 3.0;   // degrees
    constexpr double TURN_MIN_POWER = 15.0;     // minimum power for turning
    constexpr double TURN_MAX_POWER = 70.0;     // maximum power for turning

    // Drive to specific (x,y) coordinates
    bool driveToPoint(double targetX, double targetY) {
        if (!SensorTracker::isValidX() || !SensorTracker::isValidY()) {
            // Stop if position invalid
            chasMove(0,0,0,0,0,0);
            return false;
        }

        // Get current position
        double currentX = SensorTracker::getX();
        double currentY = SensorTracker::getY();

        // Calculate error
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;
        double distance = std::hypot(deltaX, deltaY);
        
        // Check if we've arrived
        if (distance < POSITION_TOLERANCE) {
            chasMove(0,0,0,0,0,0);
            return true;
        }

        // Calculate desired heading (in radians)
        double targetHeading = std::atan2(deltaY, deltaX);
        double currentHeading = imu.get_rotation() * M_PI / 180.0; // Convert IMU degrees to radians
        
        // Calculate heading error (-π to π)
        double headingError = targetHeading - currentHeading;
        while (headingError > M_PI) headingError -= 2*M_PI;
        while (headingError < -M_PI) headingError += 2*M_PI;

        // Calculate drive power with slowdown
        double drivePower = DRIVE_KP * distance;
        if (distance < SLOWDOWN_DISTANCE) {
            drivePower *= (distance / SLOWDOWN_DISTANCE); // Gradual slowdown
        }
        drivePower = std::clamp(drivePower, MIN_DRIVE_POWER, MAX_DRIVE_POWER);

        // Calculate turn power
        double turnPower = HEADING_KP * headingError;
        
        // Apply powers to drive
        double leftPower = drivePower - turnPower;
        double rightPower = drivePower + turnPower;
        
        // Normalize powers to max range
        double maxPower = std::max(std::abs(leftPower), std::abs(rightPower));
        if (maxPower > MAX_DRIVE_POWER) {
            leftPower *= MAX_DRIVE_POWER / maxPower;
            rightPower *= MAX_DRIVE_POWER / maxPower;
        }

        // Apply to motors
        chasMove(leftPower, leftPower, leftPower, 
                 rightPower, rightPower, rightPower);

        return false; // Not there yet
    }

    // Helper function to drive to point with timeout
    bool driveToPointWithTimeout(double targetX, double targetY, uint32_t timeoutMs) {
        uint32_t startTime = pros::millis();
        while (pros::millis() - startTime < timeoutMs) {
            if (driveToPoint(targetX, targetY)) {
                return true;
            }
            pros::delay(10);
        }
        chasMove(0,0,0,0,0,0);
        return false;
    }

    // New function to turn to absolute heading
    bool turnToHeading(double targetDegrees) {
        double currentDeg = imu.get_rotation();
        double error = targetDegrees - currentDeg;
        
        // Normalize error to [-180, 180]
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        // Check if we're at target
        if (std::abs(error) < HEADING_TOLERANCE) {
            chasMove(0,0,0,0,0,0);
            return true;
        }
        
        // Calculate turn power
        double turnPower = Heading_KP * error;
        turnPower = std::clamp(turnPower, -TURN_MAX_POWER, TURN_MAX_POWER);
        
        // Apply minimum power to overcome static friction
        if (turnPower > 0) {
            turnPower = std::max(turnPower, TURN_MIN_POWER);
        } else {
            turnPower = std::min(turnPower, -TURN_MIN_POWER);
        }
        
        // Apply to motors
        chasMove(-turnPower, -turnPower, -turnPower,
                 turnPower, turnPower, turnPower);
        
        return false;
    }

    // Modified drive function with final heading
    bool driveToPointAndTurn(double targetX, double targetY, double finalHeadingDeg) {
        static enum class State { DRIVING, TURNING } state = State::DRIVING;
        
        if (!SensorTracker::isValidX() || !SensorTracker::isValidY()) {
            chasMove(0,0,0,0,0,0);
            return false;
        }

        switch (state) {
            case State::DRIVING: {
                if (driveToPoint(targetX, targetY)) {
                    state = State::TURNING;  // Reached position, start turning
                }
                break;
            }
            
            case State::TURNING: {
                if (turnToHeading(finalHeadingDeg)) {
                    state = State::DRIVING;  // Reset for next use
                    return true;  // Done!
                }
                break;
            }
        }
        
        return false;
    }
}

// Example usage function
void goToPosition(double x, double y) {
    // Ensure sensors are initialized
    setupSensors();
    
    while (true) {
        // Update sensor position
        updateSensors();
        
        // Try to drive to point
        if (Navigation::driveToPoint(x, y)) {
            break;
        }
        
        pros::delay(10);
    }
}

/*
--------------------------------------EXAMPLES----------------------------
// Drive to (500,500) then turn to face 90 degrees
goToPositionAndTurn(500, 500, 90.0);

// Or with timeout
Navigation::driveToPointWithTimeout(500, 500, 90.0, 5000);
--------------------------------------------------------------------------

TUNING NOTES:
DRIVE:
Start with low DRIVE_KP (0.1) and HEADING_KP (2.0)
Increase HEADING_KP until turning is responsive but not oscillating
Increase DRIVE_KP until forward motion is good but not overshooting
Adjust SLOWDOWN_DISTANCE for smooth deceleration
Adjust POSITION_TOLERANCE based on accuracy needs

TURN:
Adjust HEADING_TOLERANCE for desired accuracy
Adjust TURN_MIN_POWER to overcome static friction
Adjust TURN_MAX_POWER for smooth but quick turning
Tune Heading_KP for turning performance
*/