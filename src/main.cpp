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
bool hooked = false;

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

        std::cout << pose.x << " " << pose.y << " " << imu.get_rotation() << pose.theta << std::endl;

        pros::delay(200);
    }
}

void initialize() {
    pros::delay(500);
    chassis.calibrate(); // calibrate the chassis
    pros::Task printOdomTask(printTelemetry); // create a task to print the odometry values
    pros::Task task {[=] { intake.intakeControl(); }};
    cata.initialize();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // halfAwpRed();
    // mogoRushRed();
    //mogoRushBlue();
    // awpRed();
    // awpBlue();
    // elimRedTopSide();
    // elimBlueTopSide();
    prog();
    // chassis.moveFor(12,2000,{},false);
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
            hooked = !hooked;
            hook.set_value(hooked);
            pros::delay(500);
        }

        if (controller.get_digital(DIGITAL_R2)) { cata.toggle(); }

        pros::delay(20);
    }
}

void opcontrol() {
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

        pros::delay(10);
    }
}