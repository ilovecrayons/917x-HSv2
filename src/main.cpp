#include "main.h"
#include "config.hpp"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include <fstream>
#include <iostream>

// PID Tuner
std::ofstream myfile;

// runtime variables
bool sort = false;
double fwd;
double turning;
float up;
float down;
bool clamped = false;
int autoSelector = 5;
int secondCounter = 0;

void incrementAutonSelector() {
    autoSelector++;
    if (autoSelector > 5) { autoSelector = 0; }
}

void callSelectedAuton() {
    switch (autoSelector) {
        case 0: break;
        case 1: break;
        default: break;
    }
}

void displayCurrentAuton() {
    while (true) {
        switch (autoSelector) {
            case 0: pros::lcd::print(5, "Close Safe"); break;
            case 1: pros::lcd::print(5, "Close Disrupt"); break;
            default: break;
        }
        pros::delay(100);
    }
}

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    pros::lcd::register_btn1_cb(incrementAutonSelector);

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });

    // TESTING
    chassis.lateralPID.setGains(10, 0, 3);
    pros::lcd::set_text(6, std::to_string(chassis.lateralPID.getGains()[0]));
}


void disabled() {
    displayCurrentAuton();
}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void scoreWallStake(){
    //ADD CODE
}

void autonomous() {
    // controller.clear();
    // pros::Task pidTunerTask {[=] { chassis.lateralPID.constantChanger(controller); }};
    // while (true) {
    //     while (controller.get_digital(DIGITAL_R1)) pros::delay(10);

    //     chassis.moveFor(12, 1000, {.maxSpeed = 127, .earlyExitRange = 5}, false);
    //     pros::delay(10);
    // }
    
    // callSelectedAuton();
    chassis.setPose(63,0,-90);
    scoreWallStake();
    chassis.turnToHeading(0,1000);
    chassis.moveToPose(-46,-23,-90,1000,{.forwards = false},false);
    clamp.set_value(true);



}

//     // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
//     chassis.moveToPose(20, 15, 90, 4000);
//     // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
//     chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
//     // cancel the movement after it has traveled 10 inches
//     chassis.waitUntil(10);
//     chassis.cancelMotion();
//     // Turn to face the point x:45, y:-45. Timeout set to 1000
//     // dont turn faster than 60 (out of a maximum of 127)
//     chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
//     // Turn to face a direction of 90º. Timeout set to 1000
//     // will always be faster than 100 (out of a maximum of 127)
//     // also force it to turn clockwise, the long way around
//     chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
//     // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
//     // following the path with the back of the robot (forwards = false)
//     // see line 116 to see how to define a path
//     chassis.follow(example_txt, 15, 4000, false);
//     // wait until the chassis has traveled 10 inches. Otherwise the code directly after
//     // the movement will run immediately
//     // Unless its another movement, in which case it will wait
//     chassis.waitUntil(10);
//     pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
//     // wait until the movement is done
//     chassis.waitUntilDone();
//     pros::lcd::print(4, "pure pursuit finished!");

void arcadeCurve(pros::controller_analog_e_t power, pros::controller_analog_e_t turn, pros::Controller mast, float f) {
    up = mast.get_analog(power);
    down = mast.get_analog(turn);
    fwd = (exp(-f / 10) + exp((fabs(up) - 127) / 10) * (1 - exp(-f / 10))) * up;
    turning = -1 * down;
    leftMotors.move(fwd * 0.9 - turning);
    rightMotors.move(fwd * 0.9 + turning);
}

void opAsyncButtons() {
    while (true) {
        // toggle clamp
        if (controller.get_digital(DIGITAL_R1)) {
            clamped = !clamped;
            clamp.set_value(clamped);
            pros::delay(500);
        }
        pros::delay(10);
    }
}

void opcontrol() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::Task asyncButtons(opAsyncButtons);
    while (true) {
        pros::lcd::print(5, "L: %d", leftMotors.get_temperature_all()[0], leftMotors.get_temperature_all()[1], leftMotors.get_temperature_all()[2]);
        pros::lcd::print(6, "R: %d", rightMotors.get_temperature_all()[0], rightMotors.get_temperature_all()[1], rightMotors.get_temperature_all()[2]);
        arcadeCurve(pros::E_CONTROLLER_ANALOG_LEFT_Y, pros::E_CONTROLLER_ANALOG_RIGHT_X, controller, 9.6);

        //  red segregator
        if (topSort.get_hue() == 0) {
            sort = true;
            intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        }

        if (sort) {
            if (secondCounter < 50) {
                secondCounter++;
                intake.move(0);
            } else {
                sort = false;
                intake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            }
        }
        if (!sort) {
            if (controller.get_digital(DIGITAL_L2)) // outtake
            {
                intake.move(-0);
            }
            if (controller.get_digital(DIGITAL_L1)) // outtake
            {
                intake.move(-120);
            }
            if (controller.get_digital(DIGITAL_L1) == false &&
                controller.get_digital(DIGITAL_L2) == false) // stop intake
            {
                intake.move(127);
            }
        }

        pros::delay(10);
    }
}
