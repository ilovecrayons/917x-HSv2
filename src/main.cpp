// files
#include "main.h"
#include "config.hpp"
#include "lemlib/api.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include <iostream>
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/screen.h"
#include "pros/screen.hpp"
#include "subsystem/intake.hpp"

// runtime variables
bool sort = false;
double fwd;
double turning;
float up;
float down;
bool clamped = false;
bool hooked = false;
int armState = 0;
int separationState = 0;

void printTelemetry() {
    while (true) {
        lemlib::Pose pose = chassis.getPose(); // get the current position of the robot
        pros::screen::print(TEXT_MEDIUM, 1, "x: %f", pose.x); // prints the x position
        pros::screen::print(TEXT_MEDIUM, 2, "y: %f", pose.y); // prints the y position
        pros::screen::print(TEXT_MEDIUM, 3, "theta: %f", pose.theta); // prints the heading
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "Arm position: %d", arm.getPosition()); // prints the arm position

        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "left temp: %.1f %.1f %.1f", leftMotors.get_temperature(0),
                            leftMotors.get_temperature(1), leftMotors.get_temperature(2));

        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "right temp: %.1f %.1f %.1f", rightMotors.get_temperature(0),
                            rightMotors.get_temperature(1), rightMotors.get_temperature(2));

        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "intake temp: %.1f", intake.motor.get_temperature());

        std::cout << pose.x << " " << pose.y << " " << imu.get_rotation() << pose.theta << std::endl;
        switch (intake.ring) {
            case Intake::Ring::BLUE: controller.print(1, 1, "%s", "SEPARATING BLUE"); break;
            case Intake::Ring::RED: controller.print(1, 1, "%s", "SEPARATING RED"); break;
            case Intake::Ring::NONE: controller.print(1, 1, "%s", "SEPARATING NONE"); break;
            default: break;
        }

        pros::delay(200);
    }
}

void init_separation(Intake::Ring ring) {
    intakeMotor.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    topSort.set_led_pwm(100);
    intake.ring = ring;
    if (intake.ring == Intake::Ring::BLUE) {
        separationState = 1;
    } else if (intake.ring == Intake::Ring::RED) {
        separationState = 0;
    } else if (intake.ring == Intake::Ring::NONE) {
        separationState = 2;
    }
}

void initialize() {
    init_separation(Intake::Ring::NONE);
    pros::delay(500);
    chassis.calibrate(); // calibrate the chassis
    pros::Task printOdomTask(printTelemetry); // create a task to print the odometry values
    pros::Task task {[=] { intake.intakeControl(); }};
    arm.initialize();
}

void disabled() {}

void competition_initialize() {}

void elimBlueTopSide(){
    chassis.setPose(56, 11.5, 120);
    arm.scoreWallstake(125, true);
    pros::delay(500);
    chassis.moveFor(13, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(21, 28.5, 2000, {.forwards = false, .maxSpeed = 60});
    //chassis.moveToPoint(-50, 12, 2000, {.forwards = false, .maxSpeed = 80});
    pros::delay(500);
    arm.retract();
    //chassis.waitUntilDone();
    chassis.waitUntil(43);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(26, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(26, 44, 2000, {}, true);
    pros::delay(1000);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveFor(15, 1000,{},false);
    chassis.turnToPoint(9, 50, 2000);
    intake.set(Intake::IntakeState::INTAKING, 127);
    pros::delay(1000);

    // second ring
    chassis.moveToPoint(10, 52, 2000, {.maxSpeed = 90}, false);
    pros::delay(1000);
    chassis.moveFor(15,2000,{.forwards = false},false); 
    chassis.turnToPoint(9, 44, 2000);
    chassis.moveToPoint(9, 44, 2000,{},false);
    pros::delay(750);

chassis.moveFor(20, 1000,{.forwards = false});
    // releasing stake near POSITIVE CORNER
    // chassis.turnToPoint(40, 0, 2000, {}, false);
    // chassis.moveToPoint(40, 0, 3000,
    //                     {
    //                         .maxSpeed = 90,
    //                         .minSpeed = 50,
    //                         .earlyExitRange = 5,
    //                     },
    //                     false);
    // intake.set(Intake::IntakeState::STOPPED);

    // chassis.moveToPoint(44, -45, 3000, {.maxSpeed = 70}, true);
    // chassis.waitUntil(15);
    // clamp.set_value(false);

    // // clamp next goal
    // chassis.turnToPoint(21, -26, 2000, {.forwards = false}, false);
    // chassis.moveToPoint(21, -26, 2000, {.forwards = false, .maxSpeed = 70}, true);
    // chassis.waitUntil(25);
    // clamp.set_value(true);

    // // scoring next ring
    // chassis.moveToPoint(21, -50, 2000, {}, true);
    // pros::delay(500);
    // intake.set(Intake::IntakeState::INTAKING, 127);

    // // touching bar
    // chassis.turnToPoint(13, -24, 2000, {.minSpeed = 50}, false);
    // chassis.moveToPoint(13, -24, 2000, {.minSpeed = 60}, false);
}


void elimRedTopSide(){
    chassis.setPose(-56, 11.5, -120);
    arm.scoreWallstake(125, true);
    pros::delay(500);
    chassis.moveFor(13, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-21, 28.5, 2000, {.forwards = false, .maxSpeed = 60});
    //chassis.moveToPoint(-50, 12, 2000, {.forwards = false, .maxSpeed = 80});
    pros::delay(500);
    arm.retract();
    //chassis.waitUntilDone();
    chassis.waitUntil(43);
    clamp.set_value(true);
    

    chassis.turnToPoint(-7, 40, 2000,{},false);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-7, 40, 2000,{},false);
    pros::delay(750);
    chassis.moveFor(3,2000,{.forwards = false},false); 
    // second ring
    chassis.moveToPoint(-10, 50, 2000, {.maxSpeed = 90}, false);
    pros::delay(750);

    // first ring
    chassis.turnToPoint(-30, 48, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    arm.loadWallstake();
    chassis.moveToPoint(-26, 48, 2000, {}, false);
    chassis.turnToPoint(-8,62,2000,{},false);
    chassis.moveToPose(-8,62,60,2000,{},false);
    intake.set(Intake::IntakeState::STOPPED);
    arm.scoreWallstake();
//     pros::delay(3000);
//     intake.set(Intake::IntakeState::OUTTAKE,90);
//     chassis.moveFor(15, 1000,{},false);
//     chassis.turnToPoint(-9, 50, 2000);
//     intake.set(Intake::IntakeState::INTAKING, 127);
//     pros::delay(1000);


// chassis.moveFor(20, 1000,{.forwards = false});


    // releasing stake near POSITIVE CORNER
    // chassis.turnToPoint(40, 0, 2000, {}, false);
    // chassis.moveToPoint(40, 0, 3000,
    //                     {
    //                         .maxSpeed = 90,
    //                         .minSpeed = 50,
    //                         .earlyExitRange = 5,
    //                     },
    //                     false);
    // intake.set(Intake::IntakeState::STOPPED);

    // chassis.moveToPoint(44, -45, 3000, {.maxSpeed = 70}, true);
    // chassis.waitUntil(15);
    // clamp.set_value(false);

    // // clamp next goal
    // chassis.turnToPoint(21, -26, 2000, {.forwards = false}, false);
    // chassis.moveToPoint(21, -26, 2000, {.forwards = false, .maxSpeed = 70}, true);
    // chassis.waitUntil(25);
    // clamp.set_value(true);

    // // scoring next ring
    // chassis.moveToPoint(21, -50, 2000, {}, true);
    // pros::delay(500);
    // intake.set(Intake::IntakeState::INTAKING, 127);

    // // touching bar
    // chassis.turnToPoint(13, -24, 2000, {.minSpeed = 50}, false);
    // chassis.moveToPoint(13, -24, 2000, {.minSpeed = 60}, false);
}

void elimRedTopSide_Slow(){
    chassis.setPose(-56, 11.5, -120);
    arm.scoreWallstake(125, true);
    pros::delay(500);
    chassis.moveFor(13, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-21, 28.5, 2000, {.forwards = false, .maxSpeed = 60});
    //chassis.moveToPoint(-50, 12, 2000, {.forwards = false, .maxSpeed = 80});
    pros::delay(500);
    arm.retract();
    //chassis.waitUntilDone();
    chassis.waitUntil(43);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(-26, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(-26, 43, 2000, {}, true);
    pros::delay(3000);
    intake.set(Intake::IntakeState::OUTTAKE,90);
    chassis.moveFor(15, 1000,{},false);
    chassis.turnToPoint(-9, 50, 2000);
    intake.set(Intake::IntakeState::INTAKING, 127);
    pros::delay(1000);

    // second ring
    chassis.moveToPoint(-10, 54, 2000, {.maxSpeed = 90}, false);
    pros::delay(1000);
    chassis.moveFor(11,2000,{.forwards = false},false); 
    chassis.turnToPoint(-9, 44, 2000);
    chassis.moveToPoint(-9, 44, 2000,{},false);
    pros::delay(750);

chassis.moveFor(20, 1000,{.forwards = false});
    // releasing stake near POSITIVE CORNER
    // chassis.turnToPoint(40, 0, 2000, {}, false);
    // chassis.moveToPoint(40, 0, 3000,
    //                     {
    //                         .maxSpeed = 90,
    //                         .minSpeed = 50,
    //                         .earlyExitRange = 5,
    //                     },
    //                     false);
    // intake.set(Intake::IntakeState::STOPPED);

    // chassis.moveToPoint(44, -45, 3000, {.maxSpeed = 70}, true);
    // chassis.waitUntil(15);
    // clamp.set_value(false);

    // // clamp next goal
    // chassis.turnToPoint(21, -26, 2000, {.forwards = false}, false);
    // chassis.moveToPoint(21, -26, 2000, {.forwards = false, .maxSpeed = 70}, true);
    // chassis.waitUntil(25);
    // clamp.set_value(true);

    // // scoring next ring
    // chassis.moveToPoint(21, -50, 2000, {}, true);
    // pros::delay(500);
    // intake.set(Intake::IntakeState::INTAKING, 127);

    // // touching bar
    // chassis.turnToPoint(13, -24, 2000, {.minSpeed = 50}, false);
    // chassis.moveToPoint(13, -24, 2000, {.minSpeed = 60}, false);
}
void rightRed() {
    chassis.setPose(-52, -54, 90);
    arm.loadWallstake(33, true);
    chassis.moveToPoint(-24, -56, 3000, {.minSpeed = 50, .earlyExitRange = 10});

    chassis.moveToPoint(-7, -52, 2000, {.maxSpeed = 70});
    intake.set(Intake::IntakeState::INTAKING);
    chassis.waitUntil(15);
    hook.set_value(true);
    chassis.waitUntilDone();
    chassis.moveToPoint(-20, -54, 2000, {.forwards = false});
    chassis.waitUntilDone();
    hook.set_value(false);
    chassis.moveToPoint(-12.5, -53, 2000, {.forwards = true});
    chassis.waitUntilDone();

    chassis.swingToHeading(125, lemlib::DriveSide::RIGHT, 2000);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.waitUntilDone();
    arm.scoreWallstake();
    chassis.moveToPoint(-34, -52, 2000, {.forwards = false});
    arm.retract();
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-20, -43, 2000);
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.turnToHeading(180, 2000);
    intake.set(Intake::IntakeState::INTAKING, 20);
    chassis.moveFor(26, 2000, {.forwards = false, .maxSpeed = 50});
    chassis.waitUntil(23);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::INTAKING, 127);
    pros::delay(500);
    chassis.moveToPoint(-50, -35, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::OUTTAKE);
    clamp.set_value(false);
    pros::delay(200);
    chassis.moveToPoint(-12, -11, 3000, {.maxSpeed = 70});
}

void rightBlue() {
    chassis.setPose(52, -54, -90);
    // arm.loadWallstake(33, true);
    // chassis.moveToPoint(24, -56, 3000, {.minSpeed = 50, .earlyExitRange = 10});

    // chassis.moveToPoint(7, -52, 2000, {.maxSpeed = 70});
    // intake.set(Intake::IntakeState::INTAKING);
    // chassis.waitUntil(15);
    // hook.set_value(true);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(20, -54, 2000, {.forwards = false});
    // chassis.waitUntilDone();
    // hook.set_value(false);
    // chassis.moveToPoint(12.5, -53, 2000, {.forwards = true});
    // chassis.waitUntilDone();

    // chassis.swingToHeading(-125, lemlib::DriveSide::RIGHT, 2000);
    // intake.set(Intake::IntakeState::STOPPED);
    // chassis.waitUntilDone();
    // arm.scoreWallstake();
    // chassis.moveToPoint(34, -52, 2000, {.forwards = false});
    // arm.retract();
    // chassis.waitUntilDone();
    // intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(19, -43, 2000, {.maxSpeed = 70});
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.turnToHeading(180, 2000);
    // intake.set(Intake::IntakeState::INTAKING, 30);
    chassis.moveFor(26, 2000, {.forwards = false, .maxSpeed = 45});
    chassis.waitUntil(23);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::INTAKING, 127);
    pros::delay(500);
    chassis.moveToPoint(50, -35, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::OUTTAKE);
    clamp.set_value(false);
    pros::delay(200);
    // chassis.moveToPoint(12, -11, 3000, {.maxSpeed = 70});
}

void autonomous() {
    intake.setSeparation(Intake::Ring::NONE);
    // chassis.setPose(0,0,0);
    // chassis.moveFor(24, 10000, {.maxSpeed = 70});
    // chassis.waitUntilDone();
    // chassis.moveFor(24, 10000, {.forwards=false, .maxSpeed = 70});
    // chassis.waitUntilDone();
    // chassis.moveFor(24, 10000, {.maxSpeed = 70});
    // chassis.waitUntilDone();
    // chassis.moveFor(24, 10000, {.forwards=false, .maxSpeed = 70});
    // chassis.waitUntilDone();
    //chassis.angularPID.setGains(5.2, 0, 45);
    // chassis.turnToHeading(90, 10000);
    // chassis.waitUntilDone();

    // chassis.turnToHeading(0,10000);
    // chassis.waitUntilDone();
    // // //11 0 50
    //chassis.lateralPID.setGains(26,0,150);
    
    // chassis.moveToPoint(0,-30, 10000, {.forwards=false, .maxSpeed = 80});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0,0, 10000, {.maxSpeed = 80});
    // chassis.waitUntilDone();
    // rightRed();
    //progSkillsWithOneWallstake();
    // awpRed();
    //awpBlue();
    elimRedTopSide();
    //rightBlue();
    //topElim_Red();
    // progSkillsWithOneWallstake();
}
// remember to attend the 1v1 meowing

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

        if (controller.get_digital(DIGITAL_R2)) {
            armState++;
            if (armState > 2) { armState = 0; }
            switch (armState) {
                case 0: arm.retract(); break;
                case 1: arm.loadWallstake(); break;
                case 2:
                    intake.set(Intake::IntakeState::OUTTAKE, 50);
                    arm.scoreWallstake();
                    intake.set(Intake::IntakeState::STOPPED);
                    break;
                default: break;
            }
        }

        if (controller.get_digital(DIGITAL_DOWN)) {
            arm.scoreWallstake(125);
            armState = 1;
        }

        pros::delay(10);
    }
}

void switchSeparation() {
    if (controller.get_digital(DIGITAL_UP) && controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
        separationState++;
        if (separationState > 2) { separationState = 0; }
        switch (separationState) {
            case 0: intake.setSeparation(Intake::Ring::RED); break;
            case 1: intake.setSeparation(Intake::Ring::BLUE); break;
            case 2: intake.setSeparation(Intake::Ring::NONE); break;
        }
        pros::delay(500);
    }
}

void opcontrol() {
    intakeMotor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // arm.retract(20, true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::Task asyncButtons(opAsyncButtons);
    while (true) {
        switchSeparation();
        arcadeCurve(pros::E_CONTROLLER_ANALOG_LEFT_Y, pros::E_CONTROLLER_ANALOG_RIGHT_X, controller, 9.6);

        if (!intake.sort) {
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

            if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
                arm.retract();
                armState = 0;
            }
        }
        pros::delay(10);
    }
}