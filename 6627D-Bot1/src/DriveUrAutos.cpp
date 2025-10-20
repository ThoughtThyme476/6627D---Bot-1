#include "main.h"
#include <vector>
#include <cmath>
#include <algorithm>
#include <random>
#include "odom.h"
#include "robot.h"
#include "pid.h"
//#include "adi.h"
#include "okapi/impl/device/button/adiButton.hpp"

// pros::ADIDigitalOut TongueMech ('A',false);
// pros::ADIDigitalOut MiddleHood ('D',false);
// pros::ADIDigitalOut Descore('C',false);
// pros::ADIDigitalOut Park('B', false);

struct Movement {
    enum Type { STRAIGHT, TURN, MAININTAKE, INTAKE1, INTAKE2, PARK, TONGUE, DESCORE, MIDDLE_HOOD };
    Type type;
    double value;      // Distance(mm) or angle(deg)
    double speed;      // Motor speed (-127 to 127)
    bool state;        // For pneumatics (true/false)
    int duration_ms;   // For mechanism actions
    
    Movement(Type t, double v, double s, bool st = false, int d = 0) 
        : type(t), value(v), speed(s), state(st), duration_ms(d) {}
};

std::vector<Movement> recordedPath;
bool isRecording = false;
int errorCount = 0;

// Add these global variables at the top with your other globals
bool currentParkState = false;
bool currentTongueState = false;
bool currentDescoreState = false;
bool currentMiddleHoodState = false;

// Error checking function
bool validateMovement(const Movement& move) {
    switch(move.type) {
        case Movement::STRAIGHT:
            return (abs(move.value) < 2000 && abs(move.speed) <= 100);
        case Movement::TURN:
            return (abs(move.value) <= 360 && abs(move.speed) <= 100);
        case Movement::MAININTAKE:
        case Movement::INTAKE1:
        case Movement::INTAKE2:
            return (abs(move.speed) <= 127 && move.duration_ms <= 2000);
        case Movement::PARK:
        case Movement::TONGUE:
        case Movement::DESCORE:
        case Movement::MIDDLE_HOOD:
            return true; // Pneumatics are always valid
        default:
            return false;
    }
}

// Path optimization
void optimizePath() {
    std::vector<Movement> optimized;
    
    for(size_t i = 0; i < recordedPath.size(); i++) {
        // Combine similar consecutive movements
        if(i > 0 && recordedPath[i].type == recordedPath[i-1].type) {
            if(recordedPath[i].type == Movement::STRAIGHT) {
                optimized.back().value += recordedPath[i].value;
                optimized.back().speed = (optimized.back().speed + recordedPath[i].speed) / 2;
            }
            else if(recordedPath[i].type == Movement::TURN) {
                optimized.back().value += recordedPath[i].value;
                optimized.back().speed = (optimized.back().speed + recordedPath[i].speed) / 2;
            }
            else {
                optimized.push_back(recordedPath[i]);
            }
        }
        else {
            optimized.push_back(recordedPath[i]);
        }
    }
    
    recordedPath = optimized;
}

void recordMovement(int duration_ms) {
    // Reset sensors
    imu.reset();
    LF.tare_position();
    RF.tare_position();
    
    double lastHeading = 0;
    double lastPosition = 0;
    bool lastIntakeState = false;
    bool lastShooterState = false;
    int startTime = pros::millis();
    
    printf("Recording started - drive for %d ms\n", duration_ms);
    isRecording = true;
    errorCount = 0;

    bool lastMainIntakeState = false;
    bool lastIntake1State = false;
    bool lastIntake2State = false;
    bool lastParkState = false;
    bool lastTongueState = false;
    bool lastDescoreState = false;
    bool lastMiddleHoodState = false;

    while (pros::millis() - startTime < duration_ms && errorCount < 5) {
        // Get current readings
        double currentHeading = imu.get_heading();
        double currentPosition = (LF.get_position() + RF.get_position()) / 2;
        double currentSpeed = (LF.get_actual_velocity() + RF.get_actual_velocity()) / 2;
        
        // Check mechanism states
        bool mainIntakeActive = (abs(MainIntake.get_actual_velocity()) > 10);
        bool intake1Active = (abs(Intake1.get_actual_velocity()) > 10);
        bool intake2Active = (abs(Intake2.get_actual_velocity()) > 10);
        
        // Use the tracked states instead of trying to read them
        bool parkActive = currentParkState;
        bool tongueActive = currentTongueState;
        bool descoreActive = currentDescoreState;
        bool middleHoodActive = currentMiddleHoodState;

        // Record mechanism actions
        if(mainIntakeActive != lastMainIntakeState) {
            Movement move(Movement::MAININTAKE, 0, MainIntake.get_voltage(), false, 0);
            if(validateMovement(move)) recordedPath.push_back(move);
            else errorCount++;
        }

        if(intake1Active != lastIntake1State) {
            Movement move(Movement::INTAKE1, 0, Intake1.get_voltage(), false, 0);
            if(validateMovement(move)) recordedPath.push_back(move);
            else errorCount++;
        }

        if(intake2Active != lastIntake2State) {
            Movement move(Movement::INTAKE2, 0, Intake2.get_voltage(), false, 0);
            if(validateMovement(move)) recordedPath.push_back(move);
            else errorCount++;
        }

        // Record pneumatic changes
        if(parkActive != lastParkState) {
            currentParkState = parkActive;
            Park.set_value(parkActive);
            recordedPath.push_back(Movement(Movement::PARK, 0, 0, parkActive));
        }

        if(tongueActive != lastTongueState) {
            currentTongueState = tongueActive;
            TongueMech.set_value(tongueActive);
            recordedPath.push_back(Movement(Movement::TONGUE, 0, 0, tongueActive));
        }

        if(descoreActive != lastDescoreState) {
            currentDescoreState = descoreActive;
            Descore.set_value(descoreActive);
            recordedPath.push_back(Movement(Movement::DESCORE, 0, 0, descoreActive));
        }

        if(middleHoodActive != lastMiddleHoodState) {
            currentMiddleHoodState = middleHoodActive;
            //MiddleHood.set_value(middleHoodActive);
            recordedPath.push_back(Movement(Movement::MIDDLE_HOOD, 0, 0, middleHoodActive));
        }

        // Record movements
        double headingDiff = currentHeading - lastHeading;
        double positionDiff = currentPosition - lastPosition;

        // Adjust heading for wraparound
        if(headingDiff > 180) headingDiff -= 360;
        else if(headingDiff < -180) headingDiff += 360;

        if(abs(headingDiff) > 5) {
            Movement move(Movement::TURN, headingDiff, currentSpeed);
            if(validateMovement(move)) {
                recordedPath.push_back(move);
                lastHeading = currentHeading;
            } else {
                errorCount++;
            }
        }
        else if(abs(positionDiff) > 10) {
            Movement move(Movement::STRAIGHT, positionDiff, currentSpeed);
            if(validateMovement(move)) {
                recordedPath.push_back(move);
                lastPosition = currentPosition;
            } else {
                errorCount++;
            }
        }

        // Update previous states
        lastMainIntakeState = mainIntakeActive;
        lastIntake1State = intake1Active;
        lastIntake2State = intake2Active;
        lastParkState = parkActive;
        lastTongueState = tongueActive;
        lastDescoreState = descoreActive;
        //lastMiddleHoodState = middleHoodActive;

        pros::delay(20);
    }

    isRecording = false;
    optimizePath();

    // Generate autonomous code
    printf("\nGenerated Autonomous Path:\n");
    printf("void recordedAuto() {\n");
    
    for(const auto& move : recordedPath) {
        switch(move.type) {
            case Movement::STRAIGHT:
                printf("    driveStraightSlow(%.0f, %.0f);  // Drive %.0f mm at %.0f%%\n", 
                       move.value, move.speed, move.value, move.speed);
                break;
            case Movement::TURN:
                printf("    driveTurn2(%.0f);      // Turn %.0f degrees\n", 
                       move.value, move.value);
                break;
            case Movement::MAININTAKE:
                printf("    MainIntake.move(%.0f);  // Main intake at %.0f%%\n", 
                       move.speed, move.speed);
                break;
            case Movement::INTAKE1:
                printf("    Intake1.move(%.0f);     // Intake 1 at %.0f%%\n", 
                       move.speed, move.speed);
                break;
            case Movement::INTAKE2:
                printf("    Intake2.move(%.0f);     // Intake 2 at %.0f%%\n", 
                       move.speed, move.speed);
                break;
            case Movement::PARK:
                printf("    Park.set_value(%s);     // Park %s\n", 
                       move.state ? "true" : "false", move.state ? "enabled" : "disabled");
                break;
            case Movement::TONGUE:
                printf("    TongueMech.set_value(%s);  // Tongue %s\n", 
                       move.state ? "true" : "false", move.state ? "extended" : "retracted");
                break;
            case Movement::DESCORE:
                printf("    Descore.set_value(%s);     // Descore %s\n", 
                       move.state ? "true" : "false", move.state ? "enabled" : "disabled");
                break;
            case Movement::MIDDLE_HOOD:
                printf("    MiddleHood.set_value(%s);  // Middle hood %s\n", 
                       move.state ? "true" : "false", move.state ? "extended" : "retracted");
                break;
        }
        printf("    pros::delay(20);\n");
    }
    
    printf("}\n");
    
    if(errorCount > 0) {
        printf("\nWarning: %d invalid movements were filtered out\n", errorCount);
    }
}

// Call this function to start recording
void startRecording() {
    recordedPath.clear();
    recordMovement(15000);  // Record for 15 seconds
}

