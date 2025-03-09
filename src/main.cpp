// files
#include "main.h"
#include "config.hpp"
#include "lemlib/api.hpp"
#include <iostream>
#include "pros/misc.h"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "autons/contest.hpp"
#include "autons/quals.hpp"
#include "autons/tune.hpp"
#include "autons/prog.hpp"
#include "autons/ringSide.hpp"

// runtime variables
double fwd;
double turning;
float up;
float down;
bool clamped = false;
bool lifted = false;
int autoSelector = 0;

void printTelemetry() {
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::screen::print(TEXT_MEDIUM, 1, "x: %f", pose.x); // prints the x position
        pros::screen::print(TEXT_MEDIUM, 2, "y: %f", pose.y); // prints the y position
        pros::screen::print(TEXT_MEDIUM, 3, "theta: %f", pose.theta); // prints the heading
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Cata position: %d", cata.getPosition()); // prints the arm position

        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "left temp: %.1f %.1f %.1f", leftMotors.get_temperature(0),
                            leftMotors.get_temperature(1), leftMotors.get_temperature(2));

        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "right temp: %.1f %.1f %.1f", rightMotors.get_temperature(0),
                            rightMotors.get_temperature(1), rightMotors.get_temperature(2));

        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "intake temp: %.1f", intake.motor.get_temperature());

        switch (autoSelector) {
            case 0: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Prog"); break;
            case 1: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: AWP Red"); break;
            case 2: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: AWP Blue"); break;
            case 3: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Half AWP Red"); break;
            case 4: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Half AWP Blue"); break;
            case 5: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Mogo Rush Red"); break;
            case 6: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Mogo Rush Blue"); break;
            case 7: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Elim Red Top Side"); break;
            case 8: pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Auto: Elim Blue Top Side"); break;
            default: break;
        }

        std::cout << pose.x << " " << pose.y << " " << imu.get_rotation() << pose.theta << std::endl;

        pros::delay(200);
    }
}

void initialize() {
    pros::delay(1000);
    chassis.calibrate(); // calibrate the chassis
    pros::Task printOdomTask(printTelemetry); // create a task to print the odometry values
    pros::Task task {[=] { intake.intakeControl(); }}; // create a task to control the intake
    cata.initialize(); // initialize the cata object
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    // halfAwpRed();
    // halfAwpBlue();
    // mogoRushRed();
    // mogoRushBlue();
    // awpRed();
    // awpBlue();
    // elimRedTopSide();
    // elimBlueTopSide();
    // hook.set_value(true); 
    
    prog();
    // chassis.moveFor(12,2000,{},false);
    // tune();
    
    // chassis.moveToPoint(0,30, 10000);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0,0, 10000, {.forwards = false});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(90, 10000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(0,10000);
    // chassis.waitUntilDone();
    //prog();

    /** 
    switch (autoSelector) {
        case 0: prog(); break;
        case 1: awpRed(); break;
        case 2: awpBlue(); break;
        case 3: halfAwpRed(); break;
        case 4: halfAwpBlue(); break;
        case 5: mogoRushRed(); break;
        case 6: mogoRushBlue(); break;
        case 7: elimRedTopSide(); break;
        case 8: elimBlueTopSide(); break;
        default: break;
    }
    */
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

        if (controller.get_digital(DIGITAL_RIGHT)) {
            lifted = !lifted;
            if(lifted){
                raiseLift();
            } else {
                lowerLift();
            }
            pros::delay(500);
        }

        if (controller.get_digital(DIGITAL_R2)) { cata.toggle(); }

        pros::delay(20);
    }
}

void opcontrol() {
    //perform reset
    std::pair<float, float> reset = distReset.getDistance(DistanceReset::Wall::BOTTOM);
    chassis.setPose(reset.first, reset.second, 0);
    
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::Task asyncButtons(opAsyncButtons);

    while (true) {
        arcadeCurve(pros::E_CONTROLLER_ANALOG_LEFT_Y, pros::E_CONTROLLER_ANALOG_RIGHT_X, controller, 9.6);

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

        if (controller.get_digital(DIGITAL_X)) {
            autoSelector++;
            if (autoSelector > 8) { autoSelector = 0; }
        }
        
        pros::delay(10);
    }
}