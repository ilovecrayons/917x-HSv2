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

void progSkillsWithOneWallstake() {
    wallstake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-60, 0, 90); // set the starting position of the robot
    intake.set(Intake::IntakeState::INTAKING); // start the intake
    pros::delay(600);
    chassis.moveToPoint(-46, 0, 2000,
                        {
                            .maxSpeed = 70,
                        },
                        false);
    intake.set(Intake::IntakeState::STOPPED);

    chassis.turnToHeading(0, 2000, {}, false);
    chassis.moveToPoint(-45, -32, 2000, {.forwards = false, .maxSpeed = 60}, true);
    chassis.waitUntil(23.5);
    clamp.set_value(true); // clamp the stake
    chassis.waitUntilDone();
    chassis.moveToPose(-23, -24, 90, 2000, {.minSpeed = 90}, true);
    chassis.waitUntil(5);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(-20, -50, 2000, {}, false);
    chassis.moveToPoint(-20, -50, 2000, {.maxSpeed = 60}, false);
    chassis.turnToPoint(28, -52, 2000, {}, false);
    chassis.moveToPoint(28, -52, 2000, {.maxSpeed = 75}, false);
    pros::delay(250);

    chassis.moveToPoint(51, -52, 2000, {.maxSpeed = 75}, true);
    chassis.waitUntil(15);
    arm.loadWallstake();
    chassis.waitUntilDone();
    pros::delay(800);
//
    // autistic ass mf of a wallstake
    chassis.moveToPoint(5.5, -45.5, 3000, {.forwards = false, .maxSpeed = 70}, false); // was 0.7
    chassis.turnToHeading(180, 3000, {.maxSpeed = 100}, false);
    intake.set(Intake::IntakeState::OUTTAKE,30);
    arm.loadWallstake(70);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveFor(20, 3000, {.maxSpeed = 60});
    // chassis.moveToPoint(4,-54.75,1000,{},false);
    chassis.turnToHeading(180, 3000, {.maxSpeed = 100}, false);
    arm.scoreWallstake();
    arm.retract();
    intake.set(Intake::IntakeState::INTAKING);
    pros::delay(500);
    chassis.moveFor(5, 1000, {.forwards = false}, false);

    chassis.turnToPoint(-24, -48, 2000, {}, false);
    chassis.moveToPose(-54, -55, -90, 3000, {.maxSpeed = 75}, false);
    pros::delay(100);

    chassis.moveFor(10,1000,{.forwards = false},false);
    chassis.turnToHeading(80, 2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    intake.set(Intake::IntakeState::OUTTAKE, 50);
    chassis.moveToPoint(-61, -64, 2000, {.forwards = false}, false);
    clamp.set_value(false);
    pros::delay(200);

    // get next mobile goal
    chassis.moveToPoint(-45, 0, 4000, {.maxSpeed = 65}, false);
    chassis.turnToHeading(180, 2000, {}, false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(-48, 22, 2000, {.forwards = false, .maxSpeed = 60}, false);
    chassis.waitUntil(21);
    clamp.set_value(true);
    chassis.waitUntilDone();
    pros::delay(500);

    chassis.turnToPoint(-23, 22, 2000, {}, true);
    pros::delay(300);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-23, 22, 2000, {}, false);
    pros::delay(1000);
    chassis.moveToPose(7, -13, 90 + 45, 4000, {}, false);
    chassis.turnToHeading(-30, 1000, {}, false);
    chassis.moveToPose(-22, 43, 0, 2000, {}, false);
    chassis.turnToPoint(-62, 48, 1000, {.maxSpeed = 90}, false);
    pros::delay(250);
    chassis.moveToPoint(-62, 48, 1000, {.maxSpeed = 55}, false);
    pros::delay(500);
    chassis.moveToPoint(-30, 45, 1000, {.forwards = false}, false);
    chassis.moveToPoint(-52, 60, 1000, {}, false);
    pros::delay(250);

    // releasing stake
    chassis.turnToHeading(80, 1000, {}, false);
    chassis.moveToPoint(-63, 60, 1000, {.forwards = false}, false);
    clamp.set_value(false);
    intake.set(Intake::IntakeState::OUTTAKE, 30);

//score second wallstake
    chassis.moveToPoint(0, 58,3000,{.maxSpeed = 100},true);
    chassis.waitUntil(8);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToHeading(0, 1000);
    chassis.moveFor(5,1000,{},false);
    // intake.set(Intake::INTAKING, 127);
    // arm.loadWallstake();
    // chassis.moveFor(6,2000,{},false);
    // pros::delay(300);
    // arm.scoreWallstake();
    // arm.retract();

    // // holding one rings in the intake
    // intake.set(Intake::IntakeState::INTAKING, 60);
    // chassis.moveToPoint(21, 20, 2000, {.maxSpeed = 70}, false);
    // pros::delay(250);
    // chassis.turnToPoint(45, -3, 2000,
    //                     {
    //                         .forwards = false,
    //                     },
    //                     true);
    // pros::delay(200);
    // intake.set(Intake::IntakeState::STOPPED);

    // // clamping new mobile goal
    // chassis.moveToPoint(48, -6, 2000, {.forwards = false, .maxSpeed = 60}, false);
    // clamp.set_value(true);
    // pros::delay(200);

    // // beginning scoring on new mobile goal
    // chassis.turnToPoint(20, 47, 2000, {}, false);

    // chassis.moveToPoint(21, 48, 2000, {.maxSpeed = 60}, true);
    // intake.set(Intake::IntakeState::INTAKING, 127);
    // chassis.turnToHeading(90, 2000, {}, false);
    // chassis.moveToPoint(48, 47, 2000, {.maxSpeed = 70}, false);

    // chassis.moveToPoint(27, 47, 2000, {.forwards = false, .maxSpeed = 100}, false);
    // chassis.turnToPoint(47, 58, 2000, {}, false);
    // chassis.moveToPoint(47, 58, 2000, {}, false);
    // chassis.turnToHeading(180 + 45, 2000, {}, false);
    // intake.set(Intake::IntakeState::OUTTAKE, 30);
    // clamp.set_value(false);
    // chassis.moveToPoint(58, 63, 1000, {.forwards = false}, false);
    // chassis.moveFor(5, 1000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(172, 2000, {}, false);
    // chassis.moveFor(150, 4000);
    
    // // not tested
    // //  chassis.turnToPoint(48,60,2000,{},false);
    // //  chassis.turnToHeading(180+45,2000,{.direction = AngularDirection::CCW_COUNTERCLOCKWISE},false);
    // //  chassis.moveToPoint(61,60,1000,{.forwards = false},true);
    // //  intake.set(Intake::IntakeState::OUTTAKE, 30);
    // //  clamp.set_value(false);

    // // chassis.moveToPoint(37, -10, 2000, {.forwards = false, .minSpeed = 80}, false);
    // // chassis.turnToPoint(58, -47, 1000, {}, false);
    // // chassis.moveToPoint(61, -47, 1000, {}, false);
    // // chassis.turnToHeading(-45, 1000, {}, false);
    // // chassis.moveToPoint(-61, -58, 1000, {.forwards = false}, false);
    // // clamp.set_value(true);
    // // intake.set(Intake::IntakeState::OUTTAKE, 30);
    // // pros::delay(500);

    // // final rings
}

void topElim_Red() {
    chassis.setPose(-54.7, 45, 90);

    // curve to hold first ring
    intake.set(Intake::INTAKING, 80);
    chassis.moveToPoint(-10, 47, 2000, {.maxSpeed = 70}, false);
    pros::delay(500);
    intake.set(Intake::STOPPED);

    // aligning to stake

    chassis.turnToPoint(-33, 18, 2000, {.forwards = false}, false);
    chassis.moveToPoint(-33, 18, 4000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.waitUntil(14);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.set(Intake::INTAKING, 127);

    // second ring
    chassis.turnToPoint(-25, 44, 2000, {}, false);
    chassis.moveFor(35, 2000, {.maxSpeed = 70});

    // third ring
    chassis.turnToPoint(-6, 60, 2000, {}, false);

    chassis.moveToPoint(-6, 60, 2000, {}, false);

    // score allianceStake
    chassis.turnToPoint(-58, 15, 2000, {}, false);
    chassis.moveToPoint(-58, 15, 4000, {.maxSpeed = 90}, true);
    chassis.waitUntil(14);
    arm.scoreWallstake(123, false);
    chassis.waitUntilDone();
    arm.scoreWallstake(130);
}

void topElim_Blue() {
    chassis.setPose(54.7, 45, -90);

    // curve to hold first ring
    intake.set(Intake::INTAKING, 80);
    chassis.moveToPoint(12, 47, 2000, {.maxSpeed = 70}, false);
    pros::delay(500);
    intake.set(Intake::STOPPED);
    chassis.moveFor(3, 2000, {.forwards = false});

    // aligning to stake

    chassis.turnToPoint(33, 18, 2000, {.forwards = false}, false);
    chassis.moveToPoint(33, 18, 4000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.waitUntil(14);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.set(Intake::INTAKING, 127);

    // second ring
    chassis.turnToPoint(25, 44, 2000, {}, false);
    chassis.moveFor(35, 2000, {.maxSpeed = 70});

    // third ring
    chassis.turnToPoint(6, 60, 2000, {}, false);

    chassis.moveToPoint(6, 60, 2000, {}, false);

    // score allianceStake
    chassis.turnToPoint(61, 14, 2000, {}, false);
    chassis.moveToPoint(61, 14, 4000, {.maxSpeed = 90}, true);
    chassis.waitUntil(20);
    arm.scoreWallstake(123, false);
    chassis.waitUntilDone();
    arm.scoreWallstake(130);
}

void mogoRush_Red() {
    // add fking code
    chassis.setPose(-58, -23, 90);

    chassis.turnToPoint(-3, -43, 1000, {}, false);
    chassis.moveToPoint(-3, -43, 4000, {}, false);
    hook.set_value(true);

    chassis.moveToPoint(-63, -19, 3000, {.forwards = false, .minSpeed = 30}, false);
    hook.set_value(false);

    chassis.turnToHeading(180, 1000, {}, false);
    hook.set_value(false);

    // clamp stake
    chassis.moveToPoint(-22, -23, 2000, {}, true);
    chassis.waitUntil(14);
    clamp.set_value(true);

    chassis.turnToPoint(-23, -46, 2000, {}, false);
    chassis.moveToPoint(-23, -46, 2000, {}, false);

    chassis.turnToPoint(-56, -11, 2000, {}, false);
    chassis.moveToPoint(-65, -5, 2000, {}, true);
    chassis.waitUntil(20);
    arm.scoreWallstake(123);
    chassis.waitUntilDone();
    arm.scoreWallstake(130);
}

void progSkills() {
    wallstake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    clamp.set_value(true);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setPose(-60, 0, 90); // set the starting position of the robot
    intake.set(Intake::IntakeState::INTAKING); // start the intake
    pros::delay(750);
    chassis.moveToPoint(-48, 0, 500,
                        {
                            .maxSpeed = 70,
                        },
                        false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.turnToHeading(0, 750, {}, false);
    chassis.moveToPoint(-48, -32, 1000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.waitUntil(24.5);
    clamp.set_value(false); // clamp the stake
    chassis.waitUntilDone();
    pros::delay(500);
    chassis.moveToPose(-23, -24, 90, 1000, {.minSpeed = 90}, true);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(-23, -50, 1000, {}, false);
    chassis.moveToPose(-23, -50, 90, 1000, {.minSpeed = 100}, false);
    chassis.turnToPoint(28, -48, 500, {}, false);
    chassis.moveToPoint(28, -48, 2000, {.minSpeed = 70}, false);
    pros::delay(750);
    chassis.moveToPoint(48, -48, 1000, {.maxSpeed = 70}, false);

    // code if we get our wedge
    //  chassis.moveToPoint(64,-48,1000,{},false);
    //  chassis.turnToHeading(90+30,1000,{.minSpeed = 120});
    //  chassis.turnToHeading(90-30,1000,{.minSpeed = 120});
    //  pros::delay(1000);

    arm.loadWallstake();
    pros::delay(500);
    chassis.turnToPoint(2,-45,100,{.forwards = false},false);
    chassis.moveToPoint(2,-45,3000,{.forwards = false,.maxSpeed = 60},false);
    chassis.turnToHeading(180,2000,{},false);
    chassis.moveFor(10.5,1000);
    // // chassis.moveToPoint(4,-54.75,1000,{},false);
    // chassis.turnToHeading(180,2000,{},false);
    // intake.set(Intake::IntakeState::STOPPED);
    // arm.scoreWallstake();
    // arm.loadWallstake();
    // intake.set(Intake::IntakeState::INTAKING);
    // chassis.moveFor(5, 1000,{},false);
    // chassis.moveFor(5.9,1000,{.forwards = false},false);
    // // chassis.moveToPoint(4,-59,1000,{},false);
    // // chassis.moveToPoint(4,-54.75,2000,{.forwards = false},false);
    // pros::delay(1000);
    // intake.set(Intake::IntakeState::STOPPED);
    // arm.scoreWallstake();
    // arm.retract();

//     intake.set(Intake::IntakeState::INTAKING);
//     chassis.turnToPoint(-24, -46, 500, {.minSpeed = 80}, false);
//     chassis.moveToPose(-59, -52, -90, 2000, {}, false);
//     pros::delay(500);
//     chassis.moveToPoint(-36, -46, 1000, {.forwards = false, .minSpeed = 90}, false);
//     chassis.turnToPoint(-48, -63, 500, {.minSpeed = 100}, false);
//     chassis.moveToPoint(-50, -59, 1000, {}, false);
//     pros::delay(500);
//     chassis.turnToHeading(80, 500, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 100}, false);
//     chassis.moveToPoint(-60, -66, 1000, {.forwards = false}, false);
//     clamp.set_value(true);
//     intake.set(Intake::IntakeState::OUTTAKE, 50);
//     pros::delay(200);

//     chassis.moveToPoint(-49, 0, 1000, {.maxSpeed = 100}, false);
//     chassis.turnToHeading(180, 1000, {}, false);
//     intake.set(Intake::IntakeState::STOPPED);
//     chassis.moveToPoint(-49, 20, 2000, {.forwards = false, .maxSpeed = 70}, false);
//     chassis.waitUntil(19);
//     clamp.set_value(false);
//     chassis.waitUntilDone();
//     pros::delay(500);
//     intake.set(Intake::IntakeState::INTAKING);
//     chassis.turnToPoint(-23, 22, 500, {.minSpeed = 120}, false);
//     chassis.moveToPoint(-23, 22, 1000, {.minSpeed = 90, .earlyExitRange = 2}, true);
//     chassis.moveToPose(-3, -13, 90 + 45, 3000, {}, false);
//     chassis.waitUntil(-15);
//     chassis.setPose(-4, -4, 90 + 45);
//     pros::delay(750);
//     chassis.turnToHeading(-45, 1000, {}, false);
//     chassis.moveToPose(-24, 50, 0, 2000, {}, false);
//     chassis.turnToPoint(-59, 48, 1000, {}, false);
//     chassis.moveToPoint(-59, 48, 1000, {}, false);
//     pros::delay(500);
//     chassis.moveToPoint(-34, 48, 1000, {.forwards = false}, false);
//     chassis.moveToPoint(-50, 60, 1000, {}, false);
//     pros::delay(500);
//     chassis.turnToHeading(80, 1000, {}, false);
//     chassis.moveToPoint(-60, 63, 1000, {.forwards = false}, false);
//     clamp.set_value(true);
//     intake.set(Intake::IntakeState::OUTTAKE, 30);

//     chassis.moveToPoint(0, 40, 1000, {.maxSpeed = 70, .earlyExitRange = 2}, false);
//     // chassis.moveToPoint(-0.5,57.5,2000,{.maxSpeed= 60},true);
//     // intake.set(Intake::IntakeState::INTAKING);
//     // pros::delay(500);
//     // arm.loadWallstake();
//     // chassis.turnToHeading(0,1000,{},false);
//     // pros::delay(500);
//     // arm.scoreWallstake();
//     // arm.retract();

//     intake.set(Intake::IntakeState::INTAKING, 60);
//     chassis.turnToPoint(22, 51, 500, {.minSpeed = 50}, false);
//     chassis.moveToPoint(22, 51, 1000, {.minSpeed = 90}, false);
//     chassis.moveToPose(22, 26, 180, 1000, {}, false);
//     pros::delay(500);
//     intake.set(Intake::IntakeState::STOPPED);
//     chassis.turnToPoint(50, 0, 500, {.forwards = false}, false);
//     chassis.moveToPoint(50, 0, 2000, {.forwards = false, .maxSpeed = 70}, false);
//     chassis.waitUntil(20);
//     clamp.set_value(false);
//     pros::delay(500);
//     intake.set(Intake::IntakeState::INTAKING, 127);
//     pros::delay(500);
//     chassis.turnToHeading(0, 500, {.minSpeed = 100}, false);
//     chassis.moveToPoint(40, 23, 1000, {.minSpeed = 70, .earlyExitRange = 3}, false);
//     chassis.moveToPose(40, 70, 0, 2500, {.minSpeed = 90}, false);
//     pros::delay(1000);
//     chassis.turnToPoint(59, 60, 500, {.minSpeed = 120}, false);
//     // chassis.moveToPoint(36,36,1000,{.forwards = false,.minSpeed = 80},false);
//     chassis.turnToPoint(59, 60, 500, {.minSpeed = 120}, false);
//     chassis.moveToPoint(59, 60, 1000, {}, false);
//     chassis.moveToPoint(37, -10, 2000, {.forwards = false, .minSpeed = 80}, false);
//     chassis.turnToPoint(58, -47, 1000, {}, false);
//     chassis.moveToPoint(61, -47, 1000, {}, false);
//     chassis.turnToHeading(-45, 1000, {}, false);
//     chassis.moveToPoint(-61, -58, 1000, {.forwards = false}, false);
// }

// void oldprogSkills() {
//     chassis.lateralPID.setGains(10, 0, 15);
//     chassis.angularPID.setGains(2.8, 0.55, 21);
//     chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
//     chassis.setPose(-58, 0, 90); // set the starting position of the robot
//     chassis.moveToPoint(-64, 0, 10000, {.forwards = false}, false);
//     intake.set(Intake::IntakeState::INTAKING); // start the intake
//     pros::delay(1000);
//     chassis.moveToPoint(-58, 0, 10000, {}, false);
//     intake.set(Intake::IntakeState::STOPPED);
//     chassis.moveToPoint(-50, 0, 10000, {.forwards = false}, false);
//     chassis.turnToHeading(0, 10000, {}, false);
//     chassis.moveToPoint(-51, -28, 10000, {.forwards = false, .maxSpeed = 70}, true); // previously -50,-27,-60
//     chassis.waitUntil(26.5);
//     clamp.set_value(true); // clamp the stake
//     pros::delay(750); // wait for the stake to be clamped
//     intake.set(Intake::IntakeState::INTAKING);
//     // first stake
//     chassis.moveToPoint(-24, -24, 3000, {.maxSpeed = 70}, false);
//     chassis.turnToPoint(-24, -48, 10000, {}, false);
//     chassis.moveToPoint(-24, -48, 10000, {.maxSpeed = 70}, false);

//     chassis.turnToPoint(0, -53, 10000, {}, false);
//     chassis.moveToPoint(0, -53, 10000, {.maxSpeed = 90}, false);
//     pros::delay(500);
//     chassis.turnToPoint(-47, -43, 10000, {});
//     intake.set(Intake::IntakeState::INTAKING);
//     chassis.moveToPoint(-47, -43, 10000, {.maxSpeed = 70}, false);
//     chassis.moveToPoint(-58, -43, 10000, {}, false);
//     pros::delay(500);
//     chassis.moveToPoint(-38, -43, 10000, {.forwards = false, .maxSpeed = 70}, false);
//     chassis.turnToPoint(-48, -53, 10000, {}, false);
//     chassis.moveToPoint(-53, -53, 10000, {.maxSpeed = 70}, false);
//     pros::delay(500);
//     chassis.turnToHeading(70, 5000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
//     chassis.moveToPoint(-55, -55, 10000, {.forwards = false, .maxSpeed = 70}, false);
//     clamp.set_value(false); // release stake

//     // grabbin second stake
//     chassis.moveToPoint(-42, -7, 1500, {.maxSpeed = 70}, false);
//     chassis.turnToHeading(180, 10000, {.maxSpeed = 70}, false);
//     intake.set(Intake::IntakeState::STOPPED);
//     chassis.moveToPoint(-44, 32, 2000, {.forwards = false, .maxSpeed = 50}, false);
//     clamp.set_value(true);
//     pros::delay(1000); // grab stake
//     // begin scoring stake
//     intake.set(Intake::IntakeState::INTAKING);
//     chassis.turnToPoint(-21, 34, 10000, {}, false);
//     chassis.moveToPoint(-21, 34, 10000, {.maxSpeed = 70}, false);
//     chassis.turnToHeading(135, 10000, {}, false);
//     chassis.setPose(-24, 24, 135);
//     pros::delay(500);
//     chassis.moveToPoint(0, 0, 10000, {.maxSpeed = 70}, false);
//     pros::delay(500);
//     chassis.turnToHeading(-50, 10000, {}, false);
//     chassis.moveToPose(-23, 42, -10, 10000, {.maxSpeed = 70}, false);
//     chassis.turnToHeading(-90, 10000, {}, false);
//     chassis.moveToPoint(-60, 42, 1000, {.maxSpeed = 70}, false);
//     pros::delay(500);
//     chassis.moveToPoint(-40, 42, 10000, {.forwards = false}, false);
//     chassis.turnToPoint(-48, 50, 10000, {}, false);
//     chassis.moveToPoint(-48, 50, 10000, {.maxSpeed = 70}, false);
//     chassis.turnToHeading(110, 10000, {.direction = AngularDirection::CW_CLOCKWISE}, false);
//     chassis.moveToPoint(-54, 53, 10000, {.forwards = false, .maxSpeed = 70}, false);
//     clamp.set_value(false); // release stake
//     chassis.turnToPoint(0, 60, 10000, {}, false);
//     chassis.moveToPoint(0, 63, 10000, {.maxSpeed = 70});
//     arm.loadWallstake();
//     chassis.turnToHeading(0, 10000, {}, false);
//     intake.set(Intake::IntakeState::OUTTAKE, 30);
//     arm.scoreWallstake();
//     intake.set(Intake::IntakeState::INTAKING, 110);
}

void awpRed() {
    chassis.setPose(-56, 11, -120);
    arm.scoreWallstake(123, true);
    pros::delay(500);
    chassis.moveFor(11.8, 2000);
    chassis.waitUntilDone();
    // arm.scoreWallstake(130, true);
    // pros::delay(250);
    chassis.moveToPoint(-47, 12, 2000, {.forwards = false, .maxSpeed = 80});
    pros::delay(500);
    arm.retract();
    chassis.waitUntilDone();
    chassis.moveToPoint(-21, 25, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(25);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(-23, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(-23, 44, 2000, {}, true);
    pros::delay(200);

    // second ring
    chassis.moveToPoint(-10, 43, 2000, {}, false);
    pros::delay(30);

    // third

    // releasing stake near POSITIVE CORNER
    chassis.turnToPoint(-40, 0, 2000, {}, false);
    chassis.moveToPoint(-40, 0, 3000,
                        {
                            .maxSpeed = 90,
                            .minSpeed = 50,
                            .earlyExitRange = 5,
                        },
                        false);
    intake.set(Intake::IntakeState::STOPPED);

    chassis.moveToPoint(-44, -42, 3000, {.maxSpeed = 70}, true);
    chassis.waitUntil(15);
    clamp.set_value(false);

    // clamp next goal
    chassis.turnToPoint(-21, -23, 2000, {.forwards = false}, false);
    chassis.moveToPoint(-21, -23, 2000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.waitUntil(15);
    clamp.set_value(true);

    // scoring next ring
    chassis.moveToPoint(-28, -48, 2000, {}, true);
    pros::delay(500);
    intake.set(Intake::IntakeState::INTAKING, 127);

    // touching bar
    chassis.turnToPoint(-15, -28, 2000, {.minSpeed = 50}, false);
    chassis.moveToPoint(-15, -28, 2000, {.minSpeed = 60}, false);
}

void awpBlue() {
    chassis.setPose(56, 11, 120);
    arm.scoreWallstake(123, true);
    pros::delay(500);
    chassis.moveFor(11.8, 2000);
    chassis.waitUntilDone();
    // arm.scoreWallstake(130, true);
    // pros::delay(250);
    chassis.moveToPoint(47, 12, 2000, {.forwards = false, .maxSpeed = 80});
    pros::delay(500);
    arm.retract();
    chassis.waitUntilDone();
    chassis.moveToPoint(21, 25, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntil(25);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(23, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(23, 44, 2000, {}, true);
    pros::delay(200);

    // second ring
    chassis.moveToPoint(10, 43, 2000, {}, false);
    pros::delay(30);

    // third

    // releasing stake near POSITIVE CORNER
    chassis.turnToPoint(40, 0, 2000, {}, false);
    chassis.moveToPoint(40, 0, 3000,
                        {
                            .maxSpeed = 90,
                            .minSpeed = 50,
                            .earlyExitRange = 5,
                        },
                        false);
    intake.set(Intake::IntakeState::STOPPED);

    chassis.moveToPoint(44, -42, 3000, {.maxSpeed = 70}, true);
    chassis.waitUntil(15);
    clamp.set_value(false);

    // clamp next goal
    chassis.turnToPoint(21, -23, 2000, {.forwards = false}, false);
    chassis.moveToPoint(21, -23, 2000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.waitUntil(15);
    clamp.set_value(true);

    // scoring next ring
    chassis.moveToPoint(28, -48, 2000, {}, true);
    pros::delay(500);
    intake.set(Intake::IntakeState::INTAKING, 127);

    // touching bar
    chassis.turnToPoint(15, -28, 2000, {.minSpeed = 50}, false);
    chassis.moveToPoint(15, -28, 2000, {.minSpeed = 60}, false);
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
    chassis.angularPID.setGains(5.2, 0, 45);
    chassis.turnToHeading(90, 10000);
    chassis.waitUntilDone();

    chassis.turnToHeading(0,10000);
    chassis.waitUntilDone();
    // // //11 0 50
    // // //chassis.lateralPID.setGains(16.5, 0, 100);
    // // chassis.moveToPoint(48,48, 10000, {.maxSpeed = 70});
    // // chassis.waitUntilDone();
    // // chassis.moveToPoint(0,0, 10000, {.forwards=false});
    // // chassis.waitUntilDone();

    // rightRed();
    //progSkillsWithOneWallstake();
    // awpRed();
    //awpBlue();
    rightBlue();
    //topElim_Red();
    //progSkillsWithOneWallstake();
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