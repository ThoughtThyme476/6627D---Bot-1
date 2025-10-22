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

        // Reference point configurations
        struct WallReference {
            double distance;     // Expected distance in mm
            double tolerance;    // Allowable deviation in mm
            bool isValid;       // Whether this reference is currently valid
        };

        WallReference backWall;
        WallReference leftWall;
        WallReference rightWall;
        double interferenceThreshold = 200.0;

        // Turning compensation
        double turnSpeedThreshold = 20.0;  // deg/sec
        double turnCompensation = 0.8;     // Reduce sensitivity during turns
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

    // Track multiple reference points
    static struct ReferenceState {
        double lastBackDist = 0.0;
        double lastLeftDist = 0.0;
        double lastRightDist = 0.0;
        bool inInterference = false;
        int interferenceCount = 0;
    } refState;

    void updateWallReference(double currentDist, Config::WallReference& wall) {
        if (isValidReading(currentDist)) {
            // Update reference if it's the first reading or within tolerance
            if (!wall.isValid || std::abs(currentDist - wall.distance) < wall.tolerance) {
                wall.distance = currentDist;
                wall.isValid = true;
            }
        }
    }

    bool detectInterference(int32_t leftDist, int32_t rightDist, int32_t backDist) {
        // Get turning speed from IMU
        double turnSpeed = std::abs(imu.get_gyro_rate().z);
        bool isTurning = turnSpeed > g_config.turnSpeedThreshold;
        
        bool suspiciousReading = false;

        // Modify thresholds during turning
        double currentThreshold = g_config.interferenceThreshold;
        if (isTurning) {
            currentThreshold *= (1.0 / g_config.turnCompensation);  // More forgiving during turns
        }

        // Check for sudden changes in width
        if (isValidReading(leftDist) && isValidReading(rightDist)) {
            double totalWidth = leftDist + rightDist;
            double expectedWidth = g_config.backWall.isValid ? 
                (g_config.fieldHalfX * 2.0) : (refState.lastLeftDist + refState.lastRightDist);
            
            if (!isTurning && std::abs(totalWidth - expectedWidth) > currentThreshold) {
                suspiciousReading = true;
            }
        }

        // Only use back sensor during turns (more reliable)
        if (isTurning) {
            if (isValidReading(backDist) && refState.lastBackDist > 0) {
                double backDelta = std::abs(backDist - refState.lastBackDist);
                if (backDelta > currentThreshold) {
                    suspiciousReading = true;
                }
            }
        }

        // Update interference state
        if (suspiciousReading) {
            refState.interferenceCount++;
        } else {
            refState.interferenceCount = std::max(0, refState.interferenceCount - 1);
        }

        // Update last valid readings
        if (isValidReading(backDist)) refState.lastBackDist = backDist;
        if (isValidReading(leftDist)) refState.lastLeftDist = leftDist;
        if (isValidReading(rightDist)) refState.lastRightDist = rightDist;

        refState.inInterference = (refState.interferenceCount >= 3);
        return refState.inInterference;
    }

    static uint32_t lastValidUpdate = 0;
    static const uint32_t SENSOR_TIMEOUT = 500; // 500ms

    bool update(int32_t frontDist, int32_t backDist, int32_t leftDist, int32_t rightDist) {
        bool validUpdate = false;
        
        if (isValidReading(frontDist) || isValidReading(backDist) || 
            isValidReading(leftDist) || isValidReading(rightDist)) {
            lastValidUpdate = pros::millis();
            validUpdate = true;
        }
        
        // Check for sensor timeout
        if (pros::millis() - lastValidUpdate > SENSOR_TIMEOUT) {
            g_validX = false;
            g_validY = false;
            return false;
        }

        // Get turning status
        double turnSpeed = std::abs(imu.get_gyro_rate().z);
        bool isTurning = turnSpeed > g_config.turnSpeedThreshold;

        // Update wall references
        updateWallReference(backDist, g_config.backWall);
        if (!isTurning) {  // Only update side references when not turning
            updateWallReference(leftDist, g_config.leftWall);
            updateWallReference(rightDist, g_config.rightWall);
        }

        bool interference = detectInterference(leftDist, rightDist, backDist);

        if (interference || isTurning) {
            // During turns or interference, prefer back sensor for Y position
            if (isValidReading(backDist) && g_config.backWall.isValid) {
                g_yPos = -g_config.fieldHalfY + backDist + g_config.offsetBack;
                g_validY = true;
            } else {
                g_validY = false;
            }
            
            if (isTurning) {
                g_validX = false;  // Don't trust X position during turns
            }
            return g_validY;
        }

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

    // Add this new function
    void setInitialBackWall(double distance) {
        g_config.backWall.distance = distance;
        g_config.backWall.isValid = true;
        
        // Initialize Y position based on back wall
        if (isValidReading(refState.lastBackDist)) {
            g_yPos = -g_config.fieldHalfY + refState.lastBackDist + g_config.offsetBack;
            g_validY = true;
        }
    }
}

// Declare sensors with proper port numbers
// static pros::Distance distFront(1);  // Change these port numbers
// static pros::Distance distBack(2);   // to match your actual
// static pros::Distance distLeft(3);   // sensor port connections
// static pros::Distance distRight(4);  // on the brain

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
    
    // Initialize wall references
    cfg.backWall.distance = 0.0;    // Will be set on first valid reading
    cfg.backWall.tolerance = 100.0;  // 100mm tolerance
    cfg.backWall.isValid = false;
    
    cfg.leftWall.distance = 0.0;
    cfg.leftWall.tolerance = 100.0;
    cfg.leftWall.isValid = false;
    
    cfg.rightWall.distance = 0.0;
    cfg.rightWall.tolerance = 100.0;
    cfg.rightWall.isValid = false;
    
    cfg.interferenceThreshold = 200.0;
    cfg.turnSpeedThreshold = 20.0;  // Adjust based on your turn speeds
    cfg.turnCompensation = 0.8;     // Adjust based on testing
    
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
    bool isImuCalibrated() {
        return !imu.is_calibrating();
    }

    // Move StallDetector definition to the top of Navigation namespace
    struct StallDetector {
        static constexpr double MIN_VELOCITY = 5.0;  // mm/sec
        static constexpr uint32_t STALL_TIME = 500;  // ms
        
        double lastX = 0, lastY = 0;
        uint32_t lastTime = 0;
        uint32_t stallStart = 0;
        bool isStalled = false;

        bool detectStall(double currentX, double currentY) {
            uint32_t now = pros::millis();
            if (lastTime == 0) {
                lastTime = now;
                lastX = currentX;
                lastY = currentY;
                return false;
            }

            double dt = (now - lastTime) / 1000.0;  // seconds
            double dx = currentX - lastX;
            double dy = currentY - lastY;
            double velocity = std::hypot(dx, dy) / dt;

            if (velocity < MIN_VELOCITY) {
                if (!isStalled) {
                    stallStart = now;
                    isStalled = true;
                }
                return (now - stallStart) >= STALL_TIME;
            }

            isStalled = false;
            lastX = currentX;
            lastY = currentY;
            lastTime = now;
            return false;
        }
    };
    
    static StallDetector stallDetector;

    // Add handlePositionLoss declaration before it's used
    bool handlePositionLoss();  // Declaration

    // Tuning constants - adjusted for -127 to 127 range
    constexpr double DRIVE_KP = 0.1;       // Keep this as is
    constexpr double Heading_KP = 1.1;     // Keep this as is
    constexpr double MIN_DRIVE_POWER = 20; // 20/127 ~15% power
    constexpr double MAX_DRIVE_POWER = 90; // 90/127 ~70% power - prevent tipping
    constexpr double POSITION_TOLERANCE = 50.0; // 50mm tolerance for arrival
    constexpr double SLOWDOWN_DISTANCE = 500.0; // Start slowing 500mm from target
    constexpr double HEADING_TOLERANCE = 3.0;   // degrees
    constexpr double TURN_MIN_POWER = 15;      // 15/127 ~12% power
    constexpr double TURN_MAX_POWER = 70;      // 70/127 ~55% power

    // Min effective power threshold also needs to be in voltage range
    const double MIN_EFFECTIVE_POWER = 10.0;  // 10/127 ~8% power

    // Add acceleration limiting
    static double lastLeftPower = 0;
    static double lastRightPower = 0;
    static constexpr double MAX_POWER_CHANGE = 8.0; // Maximum power change per 10ms
    
    double limitAcceleration(double requestedPower, double& lastPower) {
        double powerChange = requestedPower - lastPower;
        powerChange = std::clamp(powerChange, -MAX_POWER_CHANGE, MAX_POWER_CHANGE);
        lastPower += powerChange;
        return lastPower;
    }
    
    struct MovementMonitor {
        double lastX = 0;
        double lastY = 0;
        uint32_t stuckTime = 0;
        static constexpr uint32_t STUCK_TIMEOUT = 2000; // 2 seconds
        
        bool isStuck(double currentX, double currentY) {
            double movement = std::hypot(currentX - lastX, currentY - lastY); //added as a fix for for deadlock detection 
            if (movement < 5.0) { // Less than 5mm movement
                stuckTime += 10;  // Assuming 10ms loop time
            } else {
                stuckTime = 0;
            }
            
            lastX = currentX;
            lastY = currentY;
            return stuckTime >= STUCK_TIMEOUT;
        }
    };
    
    static MovementMonitor moveMonitor;
    
    // Add reset function
    void resetAccelerationLimits() {
        lastLeftPower = 0;
        lastRightPower = 0;
    }

    // Drive to specific (x,y) coordinates
    bool driveToPoint(double targetX, double targetY) {
        // Add IMU check at start
        if (!isImuCalibrated()) {
            chasMove(0,0,0,0,0,0);
            return false;
        }
        
        static double lastTargetX = 0, lastTargetY = 0;
        
        // Reset acceleration limits on new target
        if (lastTargetX != targetX || lastTargetY != targetY) {
            resetAccelerationLimits();
            lastTargetX = targetX;
            lastTargetY = targetY;
        }

        // Handle position loss
        if (handlePositionLoss()) {
            return false;
        }

        // Check for stall condition
        if (stallDetector.detectStall(SensorTracker::getX(), SensorTracker::getY())) {
            chasMove(0,0,0,0,0,0);
            return false;
        }

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
        
        // Should be modified to handle IMU wrap-around:
        double currentHeadingRad = fmod(imu.get_rotation(), 360.0) * M_PI / 180.0;
        if (currentHeadingRad < 0) currentHeadingRad += 2 * M_PI;
        
        // Calculate heading error (-π to π)
        double headingError = targetHeading - currentHeadingRad;
        while (headingError > M_PI) headingError -= 2*M_PI;
        while (headingError < -M_PI) headingError += 2*M_PI;

        // Calculate drive power with slowdown
        double drivePower = DRIVE_KP * distance;
        if (distance < SLOWDOWN_DISTANCE && SLOWDOWN_DISTANCE > 0) {
            drivePower *= (distance / SLOWDOWN_DISTANCE); // Gradual slowdown
        }
        drivePower = std::clamp(drivePower, MIN_DRIVE_POWER, MAX_DRIVE_POWER);

        // Calculate turn power
        double turnPower = HEADING_KP * headingError;
        
        // Apply powers to drive
        double leftPower = drivePower - turnPower;
        double rightPower = drivePower + turnPower;
        
        // Should add minimum power to overcome static friction:
        const double MIN_EFFECTIVE_POWER = 5.0; //added as a fix for overcoming friction
        if (std::abs(leftPower) < MIN_EFFECTIVE_POWER && std::abs(rightPower) < MIN_EFFECTIVE_POWER) {
            if (std::abs(turnPower) > MIN_EFFECTIVE_POWER) {
                // If only turning, maintain minimum turn power
                leftPower = turnPower > 0 ? -MIN_EFFECTIVE_POWER : MIN_EFFECTIVE_POWER;
                rightPower = -leftPower;
            } else {
                // If power too low, stop
                chasMove(0,0,0,0,0,0);
                return false;
            }
        }

        // Normalize powers to max range
        double maxPower = std::max(std::abs(leftPower), std::abs(rightPower));
        if (maxPower > MAX_DRIVE_POWER && maxPower != 0) {
            leftPower *= MAX_DRIVE_POWER / maxPower;
            rightPower *= MAX_DRIVE_POWER / maxPower;
        }

        // Apply acceleration limiting
        leftPower = limitAcceleration(leftPower, lastLeftPower);
        rightPower = limitAcceleration(rightPower, lastRightPower);
        
        // Apply to motors
        chasMove(leftPower, leftPower, leftPower,
                 rightPower, rightPower, rightPower);

        if (moveMonitor.isStuck(currentX, currentY)) {
            chasMove(0,0,0,0,0,0);
            return false;
        }
        
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
    bool turnToHeading2(double targetDegrees) {
        double currentDeg = std::fmod(imu.get_rotation(), 360.0);
        if (currentDeg < 0) currentDeg += 360.0;
        
        double targetNorm = std::fmod(targetDegrees, 360.0);
        if (targetNorm < 0) targetNorm += 360.0;
        
        double error = targetNorm - currentDeg;
        if (error > 180) error -= 360;
        if (error < -180) error += 360;
        
        // Check if we're at target
        if (std::abs(error) < HEADING_TOLERANCE) {
            chasMove(0,0,0,0,0,0);
            // Add small delay to let sensors stabilize
            pros::delay(100);
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
        static enum class State { DRIVING, TURNING, STABILIZING } state = State::DRIVING;
        static uint32_t stabilizeStart = 0;
        static double lastTargetX = 0, lastTargetY = 0;
        static double lastHeading = 0;  // Add this
        
        // Reset state if any target changes
        if (lastTargetX != targetX || lastTargetY != targetY || lastHeading != finalHeadingDeg) {
            state = State::DRIVING;
            lastTargetX = targetX;
            lastTargetY = targetY;
            lastHeading = finalHeadingDeg;
        }
        
        switch (state) {
            case State::DRIVING: {
                if (driveToPoint(targetX, targetY)) {
                    state = State::TURNING;
                }
                break;
            }
            
            case State::TURNING: {
                if (turnToHeading2(finalHeadingDeg)) {  // Changed to turnToHeading2
                    state = State::STABILIZING;
                    stabilizeStart = pros::millis();
                }
                break;
            }

            case State::STABILIZING: {
                // Wait for sensors to stabilize after turn
                if (pros::millis() - stabilizeStart > 250) {  // 250ms stabilization time
                    state = State::DRIVING;  // Reset for next use
                    return true;
                }
                chasMove(0,0,0,0,0,0);  // Stay still while stabilizing
                break;
            }
        }
        
        return false;
    }

    // Add position recovery strategy
    enum class RecoveryState { NORMAL, LOST_POSITION, RECOVERING };
    static RecoveryState recoveryState = RecoveryState::NORMAL;
    static uint32_t recoveryStartTime = 0;

    bool handlePositionLoss() {
        static uint32_t recoveryTimeout = 0;
        
        switch (recoveryState) {
            case RecoveryState::NORMAL:
                if (!SensorTracker::isValidX() || !SensorTracker::isValidY()) {
                    recoveryState = RecoveryState::LOST_POSITION;
                    recoveryStartTime = pros::millis();
                }
                return false;

            case RecoveryState::LOST_POSITION:
                chasMove(0,0,0,0,0,0);
                if (pros::millis() - recoveryStartTime > 250) {
                    recoveryState = RecoveryState::RECOVERING;
                }
                return false;

            case RecoveryState::RECOVERING:
                if (pros::millis() - recoveryStartTime > 2000) { // 2 second timeout
                    // Give up recovery after timeout
                    recoveryState = RecoveryState::NORMAL;
                    return false;
                }
                if (SensorTracker::isValidX() && SensorTracker::isValidY()) {
                    recoveryState = RecoveryState::NORMAL;
                    return false;
                }
                chasMove(-15, -15, -15, 15, 15, 15);
                return false;
        }
        return false;
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

void testNavigation() {
    setupSensors();
    SensorTracker::setInitialBackWall(500.0);  // Set initial position
    
    // Try a simple square pattern
    while (!Navigation::driveToPointAndTurn(0, 1000, 0)) {
        updateSensors();
        pros::delay(10);
    }
    
    while (!Navigation::driveToPointAndTurn(1000, 1000, 90)) {
        updateSensors();
        pros::delay(10);
    }
    
    while (!Navigation::driveToPointAndTurn(1000, 0, 180)) {
        updateSensors();
        pros::delay(10);
    }
    
    while (!Navigation::driveToPointAndTurn(0, 0, 0)) {
        updateSensors();
        pros::delay(10);
        }
    }

}
/*
--------------------------------------EXAMPLES----------------------------
// Drive to (500,500) then turn to face 90 degrees
goToPositionAndTurn(500, 500, 90.0);

// Or with timeout
Navigation::driveToPointWithTimeout(500, 500, 90.0, 5000);

//use in autonomous
void autonomous() {
    setupSensors();
    SensorTracker::setInitialBackWall(500.0);  // Set starting position
    
    // Basic movement sequence
    Navigation::driveToPointWithTimeout(0, 1000, 3000);  // Drive forward 1m
    Navigation::turnToHeading(90);                       // Turn 90 degrees
    Navigation::driveToPointWithTimeout(0, 0, 3000);     // Return to start
}

// Good example - proper control loop
void autonomous() {
    setupSensors();
    SensorTracker::setInitialBackWall(500.0);
    
    while (!Navigation::driveToPointWithTimeout(0, 1000, 3000)) {
        updateSensors();  // Regular position updates
        pros::delay(10);  // Give time for system to respond
    }
    
    while (!Navigation::turnToHeading(90)) {
        updateSensors();
        pros::delay(10);
    }
}
--------------------------------------------------------------------------

TUNING NOTES:
DRIVE:
Start with low DRIVE_KP (0.1) and HEADING_KP (2.0)
Increase HEADING_KP until turning is responsive but not oscillating
Increase DRIVE_KP until forward motion is good but not overshooting
Adjust SLOWDOWN_DISTANCE for smooth deceleration
Adjust POSITION_TOLERANCE based on accuracy needs

TURN:
Remember to tune turnSpeedThreshold and turnCompensation based on your robot's actual turning behavior!
Adjust HEADING_TOLERANCE for desired accuracy
Adjust TURN_MIN_POWER to overcome static friction
Adjust TURN_MAX_POWER for smooth but quick turning
Tune Heading_KP for turning performance

Remember to:

Test and adjust timeout values based on your robot's speed
Tune PID values in the Navigation namespace
Verify your sensor offsets are correct in setupSensors()
Make sure your IMU is calibrated properly

TUNABLE VARIABLES:

SENSOR CONFIGURATION:
1. cfg.offsetFront     - Distance from robot center to front sensor (mm)
2. cfg.offsetBack      - Distance from robot center to back sensor (mm)
3. cfg.offsetLeft      - Distance from robot center to left sensor (mm)
4. cfg.offsetRight     - Distance from robot center to right sensor (mm)
5. cfg.smoothAlpha     - Position smoothing factor (0.1-0.9)
6. cfg.minValid        - Minimum valid sensor reading (mm)
7. cfg.maxValid        - Maximum valid sensor reading (mm)

INTERFERENCE DETECTION:
8. cfg.interferenceThreshold  - Threshold for detecting sudden changes (mm)
9. cfg.turnSpeedThreshold    - Speed above which robot is considered turning (deg/sec)
10. cfg.turnCompensation     - Factor to adjust sensitivity during turns (0.0-1.0)
11. cfg.backWall.tolerance   - Allowable deviation in wall readings (mm)

NAVIGATION CONSTANTS:
12. DRIVE_KP           - Forward movement proportional gain
13. HEADING_KP         - Turning movement proportional gain
14. MIN_DRIVE_POWER    - Minimum power to overcome friction
15. MAX_DRIVE_POWER    - Maximum power (0-127)
16. POSITION_TOLERANCE - How close robot needs to get to target (mm)
17. SLOWDOWN_DISTANCE  - Distance to start slowing down (mm)
18. HEADING_TOLERANCE  - Acceptable heading error (degrees)
19. TURN_MIN_POWER     - Minimum power for turning
20. TURN_MAX_POWER     - Maximum power for turning

TUNING INSTRUCTIONS:

1. SENSOR OFFSETS:
   - Measure physical distances from robot center to each sensor
   - Set offsetFront/Back/Left/Right to these measurements in millimeters
   - Verify readings match actual distances from walls

2. SMOOTHING AND VALIDITY:
   - Start with smoothAlpha = 0.3, increase for more responsiveness
   - Set minValid just above minimum reliable sensor reading
   - Set maxValid just below maximum reliable sensor reading

3. INTERFERENCE DETECTION:
   - Set interferenceThreshold to ~200mm initially
   - Increase if false positives occur, decrease if missing interference
   - Set turnSpeedThreshold to speed where sensor readings become unreliable
   - Adjust turnCompensation (0.8 typical) - lower = more forgiving during turns

4. DRIVE TUNING:
   - Start with DRIVE_KP = 0.1
   - Gradually increase until robot moves quickly but doesn't overshoot
   - Set MIN_DRIVE_POWER just high enough to move robot
   - Set MAX_DRIVE_POWER to prevent tipping/wheel slip
   - Set POSITION_TOLERANCE based on required accuracy
   - Set SLOWDOWN_DISTANCE to allow smooth deceleration

5. TURNING TUNING:
   - Start with HEADING_KP = 2.0
   - Increase until turns are quick but don't oscillate
   - Set HEADING_TOLERANCE based on required turning accuracy
   - Set TURN_MIN_POWER just high enough to start turning
   - Set TURN_MAX_POWER to prevent skidding during turns

6. WALL REFERENCE:
   - Set backWall.tolerance based on field variation (~100mm typical)
   - Adjust if position updates are too strict/lenient




Remember to test each parameter in isolation and verify changes improve overall performance.
*/