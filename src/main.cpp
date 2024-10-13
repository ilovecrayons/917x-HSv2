#include "main.h"
#include "config.hpp"
#include "lemlib/api.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include <fstream>
#include <iostream>

//PID Tuner
std::ofstream myfile;



// runtime variables
bool sort = false;



/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

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

    //TESTING
    chassis.lateralPID.setGains(10, 0, 3);
    pros::lcd::set_text(6, std::to_string(chassis.lateralPID.getGains()[0]));
}
/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
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

    
}

/**
 * Runs in driver control
 */
void tankCurve(pros::controller_analog_e_t leftPower, pros::controller_analog_e_t rightPower, pros::Controller mast,
               float t) {
    // Get the joystick values for the left and right sides
    int leftInput = mast.get_analog(leftPower);
    int rightInput = mast.get_analog(rightPower);

    // Apply the exponential curve to smooth the power inputs
    float leftOutput = (exp(-t / 10) + exp((fabs(leftInput) - 127) / 10) * (1 - exp(-t / 10))) * leftInput;
    float rightOutput = (exp(-t / 10) + exp((fabs(rightInput) - 127) / 10) * (1 - exp(-t / 10))) * rightInput;

    // Move the motors accordingly
    leftMotors.move(leftOutput);
    rightMotors.move(rightOutput);
}

void opcontrol() {
    

    while (true) {
        tankCurve(pros::E_CONTROLLER_ANALOG_LEFT_Y, pros::E_CONTROLLER_ANALOG_RIGHT_Y, controller, 10);
        if (controller.get_digital(DIGITAL_R2)) // intake
        {
            intake.move(127);
        }
        if (controller.get_digital(DIGITAL_R1)) // outtake
        {
            intake.move(-127);
        }
        if (controller.get_digital(DIGITAL_R1) == false && controller.get_digital(DIGITAL_R2) == false) // stop intake
        {
            intake.move(0);
        }
    
    if (controller.get_digital(DIGITAL_L1)) { clamp.set_value(true); }
    if (controller.get_digital(DIGITAL_L2)) { clamp.set_value(false); }

    // red segregator
    // if (topSort.get_hue() == 0 && bottomSort.get_hue() == 0) { sort = true; }

    // if (sort == true && topSort.get_hue() == 0) {
    //     dGate.set_value(true);
    // } else {
    //     dGate.set_value(false);
    // }
    if (controller.get_digital(DIGITAL_DOWN)) { dGate.set_value(false); };
    if (controller.get_digital(DIGITAL_UP)) { dGate.set_value(true); };

    pros::delay(10);
    }
}
