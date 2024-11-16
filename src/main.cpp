#include "main.h"
#include "config.hpp"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include <fstream>
#include <iostream>
#include "pros/misc.h"
#include "pros/screen.hpp"
#include "subsystem/intake.hpp"

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
        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "position: %3d", (arm.rotation->get_position()/100) );
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
    
    pros::Task task{[=] {
        intake.intakeControl();
    }};
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
    // TODO: remove this
    chassis.setPose(0,0,0);
    chassis.moveToPoint(0,24,1000,{},false);
    while(true){
        pros::delay(20);
    }
    chassis.setPose(-58,0,90); // set the starting position of the robot
    chassis.moveToPoint(-64,0,10000,{.forwards = false},false);
    intake.set(Intake::IntakeState::INTAKING); // start the intake
    pros::delay(1000);
    chassis.moveToPoint(-58,0,10000,{},false);
    chassis.turnToHeading(0,1000,{},false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPose(-50,-27,-60,10000,{.forwards = false,.maxSpeed = 70},false); 
    clamp.set_value(true); // clamp the stake
    pros::delay(750); // wait for the stake to be clamped
    intake.set(Intake::IntakeState::INTAKING);
//first stake
    chassis.moveToPoint(-24,-24,3000,{.maxSpeed = 70},false);
    chassis.turnToPoint(-24, -48, 10000, {}, false);
    chassis.moveToPoint(-24,-48,10000,{.maxSpeed = 70},false);

    chassis.turnToPoint(0,-53,10000,{},false);
    chassis.moveToPoint(0,-53,10000,{.maxSpeed = 90},false);
    chassis.turnToPoint(23,-43,10000,{},false);
    chassis.moveToPoint(23,-43,10000,{.maxSpeed = 70},false);
    pros::delay(500);
    // arm.loadWallstake();                                  Initialize once arm is tuned
    chassis.turnToHeading(-90,5000,{},false);
    chassis.moveToPoint(-5,-53,10000,{.maxSpeed = 70},false);
    chassis.turnToHeading(180,10000,{},false);
    // arm.scoreWallstake();                                 Initialize once arm is tuned
    chassis.moveToPoint(-5,-40,10000,{.forwards = false},false);        
    chassis.turnToHeading(-90,10000,{.minSpeed = 90},false);
    chassis.moveToPoint(-47,-43,10000,{.maxSpeed =  70},false);
    chassis.moveToPoint(-58,-43,10000,{},false);
    pros::delay(500);
    chassis.moveToPoint(-38,-43,10000,{.forwards = false,.maxSpeed = 70},false);
    chassis.turnToPoint(-48,-53,10000,{},false);
    chassis.moveToPoint(-53,-53,10000,{.maxSpeed = 70},false);
    pros::delay(500);
    chassis.turnToHeading(70,5000,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE},false);
    clamp.set_value(false); //release stake

    
//grabbin second stake
    chassis.moveToPoint(-42,-7,1500,{.maxSpeed = 70},false);
    chassis.turnToHeading(180,10000,{.minSpeed = 90},false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(-44,32,2000,{.forwards = false,.maxSpeed = 100},false);
    clamp.set_value(true); 
    pros::delay(1000); //grab stake
//begin scoring stake
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(-21,34,10000,{},false);
    chassis.moveToPoint(-21,34,10000,{.maxSpeed = 70},false);
    chassis.turnToHeading(135,10000,{},false);
    chassis.setPose(-24,24,135);
    pros::delay(500);
    chassis.moveToPoint(0,0,10000,{.maxSpeed = 70},false);
    pros::delay(500);
    chassis.turnToHeading(-50,10000,{},false);
    chassis.moveToPose(-23,42,-10,10000,{.maxSpeed = 70},false);
    chassis.turnToHeading(-90,10000,{},false);
    chassis.moveToPoint(-60,42,1000,{.maxSpeed = 70},false);
    pros::delay(500);
    chassis.moveToPoint(-40,42,10000,{.forwards = false},false);
    chassis.turnToPoint(-48,50,10000,{},false);
    chassis.moveToPoint(-48,50,10000,{.maxSpeed = 70},false);
    chassis.turnToHeading(110,10000,{.direction = AngularDirection::CW_CLOCKWISE},false);
    clamp.set_value(false); //release stake
    chassis.turnToPoint(0,60,10000,{},false);
    chassis.moveToPoint(0,60,10000,{.maxSpeed = 70},false);
    chassis.turnToHeading(0,10000,{},false);
    arm.scoreWallstake();

}

void arcadeCurve(pros::controller_analog_e_t power, pros::controller_analog_e_t turn, pros::Controller mast, float f) {
    up = mast.get_analog(power);
    down = mast.get_analog(turn);
    fwd = (exp(-f / 10) + exp((fabs(up) - 127) / 10) * (1 - exp(-f / 10))) * up;
    turning = -1 * down;
    leftMotors.move(fwd*0.9 - turning);
    rightMotors.move(fwd*0.9 + turning);
}

void opAsyncButtons() {
    while (true) {
        // toggle clamp
        if (controller.get_digital(DIGITAL_R1)) {
            clamped = !clamped;
            clamp.set_value(clamped);
            pros::delay(500);
        }
        if(controller.get_digital(DIGITAL_UP)){
            intake.set(Intake::IntakeState::OUTTAKE, 50);
            
            arm.scoreWallstake(165, false);
            intake.set(Intake::IntakeState::STOPPED);
        }

        if(controller.get_digital(DIGITAL_DOWN)){
            arm.retract(10, false);
        }

        if(controller.get_digital(DIGITAL_RIGHT)){
            arm.loadWallstake(64, false);
        }
        pros::delay(10);
    }
}

void opcontrol() {
    //arm.retract(10, true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::Task asyncButtons(opAsyncButtons);
    while (true) {
        arcadeCurve(pros::E_CONTROLLER_ANALOG_LEFT_Y, pros::E_CONTROLLER_ANALOG_RIGHT_X, controller, 9.6);

        if (!sort) {
            if (controller.get_digital(DIGITAL_L2)) // intake
            {
                intake.set(Intake::IntakeState::INTAKING);
            }
            if (controller.get_digital(DIGITAL_L1)) // outtake
            {
                intake.set(Intake::IntakeState::OUTTAKE);
            }
            if (controller.get_digital(DIGITAL_L1) == false &&
                controller.get_digital(DIGITAL_L2) == false && controller.get_digital(DIGITAL_UP) == false ) // stop intake
            {
                intake.set(Intake::IntakeState::STOPPED);
            }
        }
        
        pros::delay(10);
    }
}
