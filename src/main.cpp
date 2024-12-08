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

// runtime variables
bool sort = false;
double fwd;
double turning;
float up;
float down;
bool clamped = false;
int armState = 0;

bool blueAlliance = true;

void printTelemetry() {
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::screen::print(TEXT_MEDIUM, 1, "x: %f", pose.x); // prints the x position
        pros::screen::print(TEXT_MEDIUM, 2, "y: %f", pose.y); // prints the y position
        pros::screen::print(TEXT_MEDIUM, 3, "theta: %f", pose.theta); // prints the heading
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Arm position: %d", arm.getPosition()); // prints the arm position
        pros::delay(20);
    }
}

void initialize() {
    chassis.calibrate(); // calibrate the chassis
    pros::Task printOdomTask(printTelemetry); // create a task to print the odometry values
    pros::Task task {[=] { intake.intakeControl(); }};
    intakeMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
}

void disabled() {}

void competition_initialize() {}

void progSkills(){
    wallstake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-60, 0, 90); // set the starting position of the robot
    intake.set(Intake::IntakeState::INTAKING); // start the intake
    pros::delay(1000);
    chassis.moveToPoint(-48, 0, 500, {.minSpeed = 90,}, false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.turnToHeading(0, 750, {}, false);
    chassis.moveToPoint(-48,-32,1000,{.forwards = false, .maxSpeed = 70},true);
    chassis.waitUntil(24.5);
    clamp.set_value(false); // clamp the stake
    chassis.waitUntilDone();
    pros::delay(750);
    chassis.moveToPose(-23,-24,90,1000,{.minSpeed = 90},true);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(-23,-50,1000,{},false);
    chassis.moveToPoint(-23,-50,1000,{.minSpeed = 90},false);
    chassis.turnToPoint(28,-48,500,{},false);
    chassis.moveToPoint(28,-48,2000,{.maxSpeed = 100},false);
    pros::delay(750);
    chassis.moveToPoint(46,-48,1000,{ .maxSpeed = 90},false);

//code if we get our wedge
    // chassis.moveToPoint(64,-48,1000,{},false);
    // chassis.turnToHeading(90+30,1000,{.minSpeed = 120});
    // chassis.turnToHeading(90-30,1000,{.minSpeed = 120});
    // pros::delay(1000);


    // arm.loadWallstake(); 
    pros::delay(1000);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.turnToPoint(2,-45,1000,{.forwards = false},false);
    chassis.moveToPoint(2,-45,1000,{.forwards = false, .minSpeed = 90},false);
    chassis.turnToHeading(180,1000,{},false);
    chassis.moveToPoint(2,-59,1000,{},false);
    // arm.scoreWallstake();
    // arm.loadWallstake();
    intake.set(Intake::IntakeState::INTAKING);
    pros::delay(500);
    intake.set(Intake::IntakeState::STOPPED);
    // arm.scoreWallstake();
    // arm.retract();

    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(-24,-46,1000,{},false);
    chassis.moveToPose(-59,-52,-90,2000,{},false);
    pros::delay(500);
    chassis.moveToPoint(-40,-46,1000,{.forwards = false, .minSpeed = 90},false);
    chassis.turnToPoint(-48,-63,1000,{},false);
    chassis.moveToPoint(-50,-59,1000,{},false);
    chassis.turnToHeading(80,1000,{},false);
    chassis.moveToPoint(-60,-68,1000,{.forwards = false},false);
    clamp.set_value(true);
    intake.set(Intake::IntakeState::OUTTAKE, 50);
    pros::delay(500);

    chassis.moveToPoint(-51,0,1000,{},false);
    chassis.turnToHeading(180,1000,{},false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(-51,20,2000,{.forwards = false,.maxSpeed = 70},false);
    chassis.waitUntil(18);
    clamp.set_value(false);
    chassis.waitUntilDone();
    pros::delay(500);
    intake.set(Intake::IntakeState::INTAKING);
    pros::delay(500);
    chassis.turnToPoint(0,0,1000,{},false);
    chassis.moveToPose(-3,-13,90+45,3000,{},false);
    chassis.setPose(-4,-4,90+45);
    pros::delay(750);
    chassis.turnToHeading(-45,1000,{},false);
    chassis.moveToPose(-24,50,0,2000,{},false);
    chassis.turnToPoint(-60,48,1000,{},false);
    chassis.moveToPoint(-60,48,1000,{},false);
    chassis.moveToPoint(-46,48,1000,{.forwards = false},false);
    chassis.moveToPoint(-46,60,1000,{},false);
    pros::delay(500);
    chassis.turnToHeading(80,1000,{},false);
    chassis.moveToPoint(-60,63,1000,{.forwards = false},false);
    
}

void oldprogSkills() {
    chassis.lateralPID.setGains(10, 0, 15);
    chassis.angularPID.setGains(2.8, 0.55, 21);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-58, 0, 90); // set the starting position of the robot
    chassis.moveToPoint(-64, 0, 10000, {.forwards = false}, false);
    intake.set(Intake::IntakeState::INTAKING); // start the intake
    pros::delay(1000);
    chassis.moveToPoint(-58, 0, 10000, {}, false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(-50, 0, 10000, {.forwards = false}, false);
    chassis.turnToHeading(0, 10000, {}, false);
    chassis.moveToPoint(-51, -28, 10000, {.forwards = false, .maxSpeed = 70}, true); // previously -50,-27,-60
    chassis.waitUntil(26.5);
    clamp.set_value(true); // clamp the stake
    pros::delay(750); // wait for the stake to be clamped
    intake.set(Intake::IntakeState::INTAKING);
    // first stake
    chassis.moveToPoint(-24, -24, 3000, {.maxSpeed = 70}, false);
    chassis.turnToPoint(-24, -48, 10000, {}, false);
    chassis.moveToPoint(-24, -48, 10000, {.maxSpeed = 70}, false);

    chassis.turnToPoint(0, -53, 10000, {}, false);
    chassis.moveToPoint(0, -53, 10000, {.maxSpeed = 90}, false);
    pros::delay(500);
    chassis.turnToPoint(-47, -43, 10000, {});
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-47, -43, 10000, {.maxSpeed = 70}, false);
    chassis.moveToPoint(-58, -43, 10000, {}, false);
    pros::delay(500);
    chassis.moveToPoint(-38, -43, 10000, {.forwards = false, .maxSpeed = 70}, false);
    chassis.turnToPoint(-48, -53, 10000, {}, false);
    chassis.moveToPoint(-53, -53, 10000, {.maxSpeed = 70}, false);
    pros::delay(500);
    chassis.turnToHeading(70, 5000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    chassis.moveToPoint(-55, -55, 10000, {.forwards = false, .maxSpeed = 70}, false);
    clamp.set_value(false); // release stake

    // grabbin second stake
    chassis.moveToPoint(-42, -7, 1500, {.maxSpeed = 70}, false);
    chassis.turnToHeading(180, 10000, {.maxSpeed = 70}, false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(-44, 32, 2000, {.forwards = false, .maxSpeed = 50}, false);
    clamp.set_value(true);
    pros::delay(1000); // grab stake
    // begin scoring stake
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(-21, 34, 10000, {}, false);
    chassis.moveToPoint(-21, 34, 10000, {.maxSpeed = 70}, false);
    chassis.turnToHeading(135, 10000, {}, false);
    chassis.setPose(-24, 24, 135);
    pros::delay(500);
    chassis.moveToPoint(0, 0, 10000, {.maxSpeed = 70}, false);
    pros::delay(500);
    chassis.turnToHeading(-50, 10000, {}, false);
    chassis.moveToPose(-23, 42, -10, 10000, {.maxSpeed = 70}, false);
    chassis.turnToHeading(-90, 10000, {}, false);
    chassis.moveToPoint(-60, 42, 1000, {.maxSpeed = 70}, false);
    pros::delay(500);
    chassis.moveToPoint(-40, 42, 10000, {.forwards = false}, false);
    chassis.turnToPoint(-48, 50, 10000, {}, false);
    chassis.moveToPoint(-48, 50, 10000, {.maxSpeed = 70}, false);
    chassis.turnToHeading(110, 10000, {.direction = AngularDirection::CW_CLOCKWISE}, false);
    chassis.moveToPoint(-54, 53, 10000, {.forwards = false, .maxSpeed = 70}, false);
    clamp.set_value(false); // release stake
    chassis.turnToPoint(0, 60, 10000, {}, false);
    chassis.moveToPoint(0, 63, 10000, {.maxSpeed = 70});
    arm.loadWallstake();
    chassis.turnToHeading(0, 10000, {}, false);
    intake.set(Intake::IntakeState::OUTTAKE, 30);
    arm.scoreWallstake();
    intake.set(Intake::IntakeState::INTAKING, 110);
}

void awpRed() {
    chassis.setPose(-56, -16, 0);
    chassis.moveToPoint(-55, -1, 1000);
    chassis.waitUntilDone();
    chassis.turnToPoint(-63, 3, 1000, {.forwards = false});
    chassis.moveToPoint(-63, 3, 1000, {.forwards = false});
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::INTAKING);
    pros::delay(800);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(-55, 3, 1000);
    chassis.waitUntilDone();
    // first stake
    chassis.turnToPoint(-50, 14, 1000);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.waitUntilDone();
    chassis.moveToPoint(-50, 14, 1000);
    chassis.waitUntilDone();
    // go in
    chassis.turnToPoint(-23, 27, 1000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(-23, 27, 2000, {.forwards = false, .maxSpeed = 50});
    chassis.waitUntil(18);
    clamp.set_value(true);
    chassis.waitUntilDone();
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPose(-30, 47, 0, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 1000);
    chassis.waitUntilDone();
    chassis.moveFor(12, 1000);
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.moveToPoint(-43, 40, 1000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(135, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-30, 30, 2000, {.maxSpeed = 90});
    chassis.waitUntilDone();
    arm.setPower(50);
}

void awpBlue() {
    chassis.setPose(56, -16, 0);
    chassis.moveToPoint(55, -2, 1000);
    chassis.waitUntilDone();
    chassis.turnToPoint(63, -2, 1000, {.forwards = false});
    chassis.moveToPoint(63, -2, 1000, {.forwards = false});
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::INTAKING);
    pros::delay(800);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(57, -2, 1000);
    chassis.waitUntilDone();

    // first stake
    chassis.turnToPoint(22, 26, 1000);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.waitUntilDone();
    chassis.moveToPoint(42, 6, 1000);
    chassis.waitUntilDone();
    // go in
    chassis.turnToPoint(22, 26, 1000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.moveToPoint(22, 26, 2000, {.forwards = false, .maxSpeed = 50});
    intake.set(Intake::IntakeState::STOPPED);
    chassis.waitUntil(15);
    clamp.set_value(true);
    chassis.waitUntilDone();
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPose(30, 47, 0, 1000);
    chassis.waitUntilDone();
    chassis.turnToHeading(-90, 1000);
    chassis.waitUntilDone();
    chassis.moveFor(12, 1000);
    chassis.waitUntilDone();
    pros::delay(250);
    chassis.moveToPoint(43, 40, 1000, {.forwards = false});
    chassis.waitUntilDone();
    chassis.turnToHeading(-135, 1000);
    chassis.waitUntilDone();
    chassis.moveToPoint(25, 20, 2000, {.minSpeed = 0});
    chassis.waitUntilDone();
    arm.setPower(100);
    pros::delay(500);
    arm.setPower(30);
}

void elimsRed() {}

void elimsBlue() {}

void autonomous() {
    // chassis.setPose(0,0,0);
    // chassis.turnToHeading(90, 10000);
    // chassis.waitUntilDone();
    
    // chassis.turnToHeading(0,10000);
    // chassis.waitUntilDone();
    // //11 0 50
    // //chassis.lateralPID.setGains(16.5, 0, 100);
    // chassis.moveToPoint(48,48, 10000, {.maxSpeed = 70});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0,0, 10000, {.forwards=false});
    // chassis.waitUntilDone();

    progSkills();



}

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

        if (controller.get_digital(DIGITAL_R2)) {
            armState++;
            if (armState > 2) { armState = 0; }
            switch (armState) {
                case 0: arm.retract(20, false); break;
                case 1: arm.loadWallstake(60, false); break;
                case 2:
                    intake.set(Intake::IntakeState::OUTTAKE, 50);
                    arm.scoreWallstake(165, false);
                    intake.set(Intake::IntakeState::STOPPED);
                    break;
                default: break;
            }
        }
        pros::delay(10);
    }
}


int SEPARATION_WAIT = 0;    
int timeToCompleteSeparation = 50;         //milliseconds divided by 10
double INITIAL_POSITION = 0;
double SEPARATION_MOVEMENT = 45;  //Degrees
void opcontrol() {
    intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    //arm.retract(20, true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::Task asyncButtons(opAsyncButtons);
    while (true) {
        if (blueAlliance){
            if (topSort.get_hue()<30 && topSort.get_hue()>=0) {
                sort = true;
                INITIAL_POSITION = intakeMotor.get_position();
            }
        }
        else {
            if (topSort.get_hue()>100 && topSort.get_hue()<=270) {
                sort = true;
                INITIAL_POSITION = intakeMotor.get_position();
            }
        }
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
            if (controller.get_digital(DIGITAL_L1) == false && controller.get_digital(DIGITAL_L2) == false &&
                controller.get_digital(DIGITAL_R2) == false) // stop intake
            {
                intake.set(Intake::IntakeState::STOPPED);
            }
        }
        else if (sort && (INITIAL_POSITION+SEPARATION_MOVEMENT) > intakeMotor.get_position()) {
            intake.set(Intake::IntakeState::INTAKING);
        }
        else if (sort && (INITIAL_POSITION+SEPARATION_MOVEMENT) <= intakeMotor.get_position() && SEPARATION_WAIT < timeToCompleteSeparation) {
            intake.set(Intake::IntakeState::STOPPED);
            SEPARATION_WAIT++;
        }
        else {
            intake.set(Intake::IntakeState::STOPPED);
            sort = !sort;
            SEPARATION_WAIT = 0;
        }
        pros::delay(10);
    }
}
