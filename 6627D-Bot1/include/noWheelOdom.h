#pragma once
#include "api.h"

// Sensor declarations
extern pros::Distance distFront;
extern pros::Distance distBack;
extern pros::Distance distLeft;
extern pros::Distance distRight;

// Core sensor functions
void setupSensors();
void updateSensors();

// SensorTracker namespace
namespace SensorTracker {
    struct Config {
        double fieldHalfX;
        double fieldHalfY;
        double offsetFront;
        double offsetBack;
        double offsetLeft;
        double offsetRight;
        double minValid;
        double maxValid;
        double smoothAlpha;
        bool requirePair;
        
        struct WallReference {
            double distance;
            double tolerance;
            bool isValid;
        };
        
        WallReference backWall;
        WallReference leftWall;
        WallReference rightWall;
        double interferenceThreshold;
        double turnSpeedThreshold;
        double turnCompensation;
    };

    void init(const Config& cfg);
    bool update(int32_t frontDist, int32_t backDist, int32_t leftDist, int32_t rightDist);
    double getX();
    double getY();
    bool isValidX();
    bool isValidY();
    void setInitialBackWall(double distance);
}

// Navigation namespace
namespace Navigation {
    // Core movement functions
    bool driveToPoint(double targetX, double targetY);
    bool driveToPointWithTimeout(double targetX, double targetY, uint32_t timeoutMs);
    bool turnToHeading(double targetDegrees);
    bool driveToPointAndTurn(double targetX, double targetY, double finalHeadingDeg);

    // Constants that might need tuning
    extern const double DRIVE_KP;
    extern const double HEADING_KP;
    extern const double MIN_DRIVE_POWER;
    extern const double MAX_DRIVE_POWER;
    extern const double POSITION_TOLERANCE;
    extern const double SLOWDOWN_DISTANCE;
    extern const double HEADING_TOLERANCE;
    extern const double TURN_MIN_POWER;
    extern const double TURN_MAX_POWER;
}

// Helper functions
void goToPosition(double x, double y);
void testNavigation();