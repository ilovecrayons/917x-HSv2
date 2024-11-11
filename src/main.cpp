#include "main.h"
#include "config.hpp"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include <fstream>
#include <iostream>
#include "pros/screen.hpp"

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

void printOdomValues() {
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::screen::print(TEXT_MEDIUM, 1, "x: %f", pose.x); // prints the x position
        pros::screen::print(TEXT_MEDIUM, 2, "y: %f", pose.y); // prints the y position
        pros::screen::print(TEXT_MEDIUM, 3, "theta: %f", pose.theta); // prints the heading
        pros::delay(20);
    }
}

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
    chassis.calibrate(); // calibrate the chassis
    pros::Task printOdomTask(printOdomValues); // create a task to print the odometry values
    arm.reset();
}

void disabled() { displayCurrentAuton(); }

void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void resetArm(){
    // ADD CODE
}

void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    //chassis.angularPID.setGains(2.8, 0.55, 21);
    chassis.setPose(0,0,0);
    chassis.turnToHeading(90, 3000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 3000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 10000);
    while(true){
        pros::delay(10);
    }


    
    chassis.setPose(-58,0,-90); // set the starting position of the robot
    chassis.moveToPoint(-52,0,10000,{.forwards = false},false);
    pros::delay(500);
    // arm.scoreWallstake();                                 Initialize once arm is tuned
    chassis.moveToPoint(-58,0,10000,{},false);
    //resetArm();                                           Initialize once arm is tuned
    chassis.moveToPose(-52,-27,-60,10000,{.forwards = false,.maxSpeed = 70},false); 
    clamp.set_value(true); // clamp the stake
    pros::delay(750); // wait for the stake to be clamped
//first stake
    intake.move(127); // start the intake
    chassis.moveToPoint(-24,-24,3000,{.maxSpeed = 70},false);
    chassis.turnToPoint(-24, -48, 10000, {}, false);
    chassis.moveToPoint(-24,-48,10000,{.maxSpeed = 70},false);

    chassis.turnToPoint(0,-55,10000,{},false);
    chassis.moveToPoint(0,-55,10000,{.maxSpeed = 90},false);
    chassis.turnToPoint(23,-43,10000,{},false);
    chassis.moveToPoint(23,-43,10000,{.maxSpeed = 70},false);
    pros::delay(500);
    // arm.loadWallstake();                                  Initialize once arm is tuned
    chassis.turnToHeading(-90,10000,{},false);
    chassis.moveToPoint(-5,-53,10000,{.maxSpeed = 70},false);
    chassis.turnToHeading(180,10000,{},false);
    // arm.scoreWallstake();                                 Initialize once arm is tuned
    chassis.moveToPoint(-5,-40,10000,{.forwards = false},false);        
    chassis.turnToHeading(-90,10000,{.minSpeed = 90},false);
    chassis.moveToPoint(-47,-46,10000,{.maxSpeed =  50},false);
    chassis.moveToPoint(-58,-46,10000,{},false);
    pros::delay(500);
    chassis.moveToPoint(-38,-48,10000,{.forwards = false,.maxSpeed = 70},false);
    chassis.turnToPoint(-48,-59,10000,{},false);
    chassis.moveToPoint(-53,-59,10000,{.maxSpeed = 70},false);
    chassis.turnToHeading(70,10000,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE},false);

    

    // chassis.moveToPoint(-59,-7,1500,{.maxSpeed = 70},false);
    // chassis.turnToHeading(180,10000,{.minSpeed = 90},false);
    // chassis.moveToPoint(-52,19,2000,{.forwards = false,.maxSpeed = 80},false);

}

void arcadeCurve(pros::controller_analog_e_t power, pros::controller_analog_e_t turn, pros::Controller mast, float f) {
    up = mast.get_analog(power);
    down = mast.get_analog(turn);
    fwd = (exp(-f / 10) + exp((fabs(up) - 127) / 10) * (1 - exp(-f / 10))) * up;
    turning = -1 * down;
    leftMotors.move(fwd - turning);
    rightMotors.move(fwd + turning);
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
    // arm.scoreWallstake();
    // arm.loadWallstake();
    // arm.scoreWallstake();
    // arm.loadWallstake();
    
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::Task asyncButtons(opAsyncButtons);
    while (true) {
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
