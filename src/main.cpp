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
        pros::screen::print(pros::E_TEXT_MEDIUM, 8, "top sort: %d", topSort.get_proximity());

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

void prog() {
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
    chassis.moveToPoint(-45, -32, 2000, {.forwards = false, .maxSpeed = 65}, true);
    chassis.waitUntil(23.5);
    clamp.set_value(true); // clamp the stake
    chassis.waitUntilDone();
    chassis.moveToPose(-23, -25, 90, 2000, {}, true); // used to be min speed 90
    chassis.waitUntil(5);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(-20, -50, 2000, {}, false);
    chassis.moveToPoint(-20, -50, 2000, {.maxSpeed = 70}, false);
    chassis.turnToPoint(8, -64, 2000, {}, false);
    chassis.moveToPoint(8, -64, 2000, {.maxSpeed = 65, .minSpeed = 30, .earlyExitRange = 6});
    chassis.moveToPoint(28, -54, 2000, {.maxSpeed = 65}, false);
    pros::delay(500);

    chassis.moveToPoint(51, -52, 2000, {.maxSpeed = 75}, true);
    chassis.waitUntil(15);
    arm.loadWallstake();
    chassis.waitUntilDone();
    pros::delay(200);

    // autistic ass mf of a wallstake
    chassis.moveToPoint(4.8, -45.5, 4000, {.forwards = false, .maxSpeed = 70}, false); // was 6.3 and 5.5 after // was 6.8 dropped to 4.8 went up to 5.8 dropped to 5.3
    chassis.turnToHeading(180, 3000, {.maxSpeed = 100});
    intake.set(Intake::IntakeState::STOPPED);
    pros::delay(200);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.waitUntilDone();
    chassis.moveFor(19.3, 1300, {.maxSpeed = 70}, false);
    // chassis.moveToPoint(4,-54.75,1000,{},false);

    intake.set(Intake::IntakeState::OUTTAKE, 30);
    arm.scoreWallstake();
    chassis.moveFor(7.5, 1000, {.forwards = false}, false);
    arm.retract();
    intake.set(Intake::IntakeState::INTAKING);

    chassis.turnToPoint(-40, -55.5, 2000, {}, false);
    chassis.moveToPose(-40, -54.5, -90, 3000, {.maxSpeed = 75}, false);

    chassis.moveFor(14, 3000, {.maxSpeed = 80});
    pros::delay(100);

    chassis.moveFor(13, 1000, {.forwards = false}, false);
    chassis.turnToPoint(-57, -69, 2000, {.forwards = false, .direction = AngularDirection::CCW_COUNTERCLOCKWISE},
                        false);
    intake.set(Intake::IntakeState::OUTTAKE, 50);
    chassis.moveToPoint(-56, -66, 2000, {.forwards = false}, false);
    clamp.set_value(false);
    pros::delay(200);

    // get next mobile goal
    chassis.moveToPoint(-43.8, 0, 4000, {.maxSpeed = 70}, false);
    chassis.turnToHeading(180, 2000, {}, false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(-43.8, 28, 2000, {.forwards = false, .maxSpeed = 60}, true);
    chassis.waitUntil(20);
    clamp.set_value(true);
    chassis.waitUntilDone();

    // chassis.turnToPoint(-20, 20, 2000, {}, true);
    pros::delay(300);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-16, 20, 2000, {.maxSpeed = 80}, false);
    pros::delay(750);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPose(8, -7, 90 + 45, 5000, {}, true);
    chassis.waitUntil(5);
    intake.set(Intake::IntakeState::INTAKING, 60);
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::STOPPED);
    chassis.turnToHeading(-30, 1000, {}, false);
    intake.set(Intake::IntakeState::INTAKING, 120);
    chassis.moveToPoint(-13, 28, 2000, {.minSpeed = 40, .earlyExitRange = 13}, true);
    chassis.moveToPoint(-13, 47, 3000, {.maxSpeed = 80}, false);
    pros::delay(500);
    // chassis.moveToPose(-22, 43, 0, 2000, {}, false);
    chassis.turnToPoint(-62, 48, 1000, {.maxSpeed = 90}, false);
    chassis.moveToPoint(-37.5, 49, 1000, {}, false);
    pros::delay(500);
    chassis.moveToPoint(-51, 49, 1000, {.maxSpeed = 65}, false);
    pros::delay(500);
    chassis.moveToPoint(-27, 45, 1000, {.forwards = false}, false);
    chassis.moveToPoint(-44, 62, 1000, {}, false);
    pros::delay(250);

    // releasing stake
    chassis.moveFor(8, 1000, {.forwards = false}, false);
    chassis.moveToPoint(-61, 65, 1000, {.forwards = false}, false);
    clamp.set_value(false);
    intake.set(Intake::IntakeState::OUTTAKE, 30);

    // score second wallstake
    chassis.moveToPoint(10.5, 46, 3000, {.maxSpeed = 60}, true); // was x = 10
    chassis.waitUntil(9);
    arm.loadWallstake();
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToHeading(0, 1000);
    chassis.moveFor(18.5, 2000, {.maxSpeed = 60}, true);
    chassis.waitUntilDone();
    pros::delay(750);
    intake.set(Intake::IntakeState::STOPPED);
    arm.scoreWallstake();
    arm.retract();

    // holding one rings in the intake
    intake.set(Intake::IntakeState::INTAKING, 70);
    chassis.moveToPoint(40, 18, 3000, {.maxSpeed = 70}, false);
    pros::delay(250);
    chassis.turnToPoint(68, -4, 300,
                        {
                            .forwards = false,
                        },
                        true);
    pros::delay(200);
    intake.set(Intake::IntakeState::STOPPED);

    // clamping new mobile goal
    chassis.moveToPoint(70, -3, 4000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.waitUntil(36);
    clamp.set_value(true);
    pros::delay(200);
    // chassis.turnToHeading(-5, 2000, {}, false);
    intake.set(Intake::INTAKING);
    chassis.moveToPoint(60, 57, 3000, {.maxSpeed = 90}, false);
    // chassis.moveFor(60,3000,{},true);
    chassis.turnToPoint(72, 65, 2000, {.forwards = false}, false);
    intake.set(Intake::IntakeState::OUTTAKE, 50);
    clamp.set_value(false);
    chassis.moveToPoint(72, 65, 1000, {.forwards = false}, false);

    chassis.moveToPoint(60, 55, 2000, {.minSpeed = 120, .earlyExitRange = 6});
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(95, -69, 5000, {.minSpeed = 120}, false);

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

void mogoRushRed() {
    chassis.setPose(-54, -36, 90);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPose(-10, -43, 90, 2000, {.maxSpeed = 90}, true);
    arm.loadWallstake();
    chassis.waitUntil(34);
    hook.set_value(true);
    // chassis.moveFor(30,2000,{.forwards = false},true);
    // chassis.waitUntil(25);
    // intake.set(Intake::IntakeState::STOPPED);
    // chassis.waitUntilDone();
    chassis.turnToHeading(-8, 2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    hook.set_value(false);
    // chassis.turnToPoint(24,-15,2000,{.forwards = false},false);
    chassis.moveToPose(-20, -17, 180, 2000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.waitUntil(33);
    clamp.set_value(true);
    chassis.moveToPoint(-10, -56, 2000, {.maxSpeed = 70}, false); // was 4,-61
    chassis.turnToHeading(+90 + 39, 2000, {}, false); // was -90-42
    chassis.moveFor(7, 2000, {}, false);
    chassis.turnToHeading(90 + 48, 1000, {}, false);
    intake.set(Intake::IntakeState::STOPPED);
    arm.scoreWallstake();
    intake.set(Intake::IntakeState::OUTTAKE, 90);
    pros::delay(200);
    chassis.moveFor(10, 2000, {.forwards = false});
    chassis.waitUntil(5);
    arm.retract();
    chassis.turnToPoint(-58, -26, 2000, {}, true);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPose(-59,-25,-90,2000,{},true);
    pros::delay(1000);
    clamp.set_value(true);
}

void mogoRushBlue() {
    chassis.setPose(54, -36, -90);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPose(12, -50, -105, 2000, {.maxSpeed = 100}, true);
    arm.loadWallstake();
    chassis.waitUntil(34);
    hook.set_value(true);
    // chassis.moveFor(30,2000,{.forwards = false},true);
    // chassis.waitUntil(25);
    // intake.set(Intake::IntakeState::STOPPED);
    // chassis.waitUntilDone();
    chassis.turnToHeading(90, 2000, {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 90}, false);
    hook.set_value(false);
    // chassis.turnToPoint(24,-15,2000,{.forwards = false},false);
    chassis.moveToPoint(12,-47,2000,{.forwards = false,.maxSpeed = 90,.minSpeed = 40,.earlyExitRange = 10},false);
    chassis.moveToPose(23, -19, -180, 2000, {.forwards = false, .maxSpeed = 70}, true); // was 23,-20 // was 21, -14.5
    chassis.waitUntil(32);
    clamp.set_value(true);
    chassis.moveToPoint(7, -61, 3000, {.maxSpeed = 80}, false); // was 4,-61
    chassis.turnToHeading(-90 - 50, 2000, {}, false); // was -90-48
    chassis.moveFor(3.5, 2000, {}, false);
    chassis.turnToHeading(-90 - 47, 500, {}, false);
    intake.set(Intake::IntakeState::STOPPED);
    arm.scoreWallstake();
    intake.set(Intake::IntakeState::OUTTAKE, 90);
    pros::delay(200);
    chassis.moveFor(10, 2000, {.forwards = false});
    chassis.waitUntil(5);
    arm.retract();
    chassis.turnToPoint(30, -35, 2000, {}, true);
    intake.set(Intake::IntakeState::INTAKING);
    //chassis.moveToPose(58,-28, 90,2000,{},false);
    chassis.moveToPoint(30, -35, 2000, {.minSpeed = 50, .earlyExitRange = 6}, false);
    chassis.moveToPoint(58, -28, 2000, {.maxSpeed = 80}, false);
    pros::delay(1000);
    clamp.set_value(false);
}

void awpRed() {
    chassis.setPose(-56, 11.5, -120);
    arm.scoreWallstake(125, true, 7);
    pros::delay(500);
    chassis.moveFor(12, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-47, 12, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 5});

    chassis.moveToPoint(-20, 28.5, 2000, {.forwards = false, .maxSpeed = 60});

    pros::delay(500);
    arm.retract();
    // chassis.waitUntilDone();
    chassis.waitUntil(27);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(-26, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(-26, 44, 2000, {}, true);
    pros::delay(200);

    // second ring
    chassis.moveToPoint(-10, 43, 2000, {}, false);
    pros::delay(30);

    // releasing stake near POSITIVE CORNER
    chassis.turnToPoint(-40, 0, 2000, {}, false);
    chassis.moveToPoint(-40, 0, 3000,
                        {
                            .maxSpeed = 90,
                            .minSpeed = 50,
                            .earlyExitRange = 5,
                        },
                        false);

    // chassis.cancelAllMotions();
    // while(true){
    //     pros::delay(200);
    // }

    intake.set(Intake::IntakeState::STOPPED);

    chassis.moveToPoint(-44, -46, 3000, {.maxSpeed = 70}, true);
    chassis.waitUntil(15);
    clamp.set_value(false);

    // clamp next goal
    chassis.turnToPoint(-21, -25, 1000, {.forwards = false}, false);
    chassis.moveToPoint(-21, -25, 2000, {.forwards = false, .maxSpeed = 60}, true);
    chassis.waitUntil(25);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::INTAKING, 127);

    // scoring next ring
    chassis.moveToPoint(-23, -50, 2000, {.maxSpeed = 65}, false);
    
    

    // touching bar
    chassis.turnToPoint(-13, -24, 2000, {.maxSpeed = 70}, false);
    chassis.moveToPoint(-13, -24, 2000, {.minSpeed = 60}, false);
}

void awpBlue() {
    chassis.setPose(56, 11.5, 120);
    arm.scoreWallstake(125, true, 7);
    pros::delay(500);
    chassis.moveFor(12, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(47, 12, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 5});

    chassis.moveToPoint(20, 28.5, 2000, {.forwards = false, .maxSpeed = 60});

    pros::delay(500);
    arm.retract();
    // chassis.waitUntilDone();
    chassis.waitUntil(27);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(26, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(26, 44, 2000, {}, true);
    pros::delay(200);

    // second ring
    chassis.moveToPoint(10, 43, 2000, {}, false);
    pros::delay(30);

    // releasing stake near POSITIVE CORNER
    chassis.turnToPoint(40, 0, 2000, {}, false);
    chassis.moveToPoint(40, 0, 3000,
                        {
                            .maxSpeed = 90,
                            .minSpeed = 50,
                            .earlyExitRange = 5,
                        },
                        false);

    // chassis.cancelAllMotions();
    // while(true){
    //     pros::delay(200);
    // }

    intake.set(Intake::IntakeState::STOPPED);

    chassis.moveToPoint(44, -46, 3000, {.maxSpeed = 70}, true);
    chassis.waitUntil(15);
    clamp.set_value(false);

    // clamp next goal
    chassis.turnToPoint(21, -25, 1000, {.forwards = false}, false);
    chassis.moveToPoint(21, -25, 2000, {.forwards = false, .maxSpeed = 60}, true);
    chassis.waitUntil(25);
    clamp.set_value(true);
    chassis.waitUntilDone();
    intake.set(Intake::IntakeState::INTAKING, 127);

    // scoring next ring
    chassis.moveToPoint(23, -50, 2000, {.maxSpeed = 65}, false);
    
    // touching bar
    chassis.turnToPoint(13, -24, 2000, {.maxSpeed = 70}, false);
    chassis.moveToPoint(13, -24, 2000, {.minSpeed = 60}, false);
}

void halfAwpRed() {
    chassis.setPose(-56, 11.5, -120);
    arm.scoreWallstake(125, true, 7);
    pros::delay(500);
    chassis.moveFor(12, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-47, 12, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 5});

    chassis.moveToPoint(-20, 28.5, 2000, {.forwards = false, .maxSpeed = 60});

    pros::delay(500);
    arm.retract();
    // chassis.waitUntilDone();
    chassis.waitUntil(27);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(-26, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(-26, 44, 2000, {}, true);
    pros::delay(200);

    // second ring
    chassis.moveToPoint(-10, 43, 2000, {}, false);
    pros::delay(30);

    // touch bar
    chassis.turnToPoint(-22, 26, 2000, {}, false);
    chassis.moveToPoint(-22, 26, 3000,
                        {
                            .maxSpeed = 80,
                        },
                        false);
    chassis.moveToPoint(-19, 10, 2000, {.maxSpeed = 70});
}

void halfAwpBlue() {
    chassis.setPose(56, 11.5, 120);
    arm.scoreWallstake(125, true, 7);
    pros::delay(500);
    chassis.moveFor(12, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(47, 12, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 5});

    chassis.moveToPoint(20, 28.5, 2000, {.forwards = false, .maxSpeed = 60});

    pros::delay(500);
    arm.retract();
    // chassis.waitUntilDone();
    chassis.waitUntil(27);
    clamp.set_value(true);

    // first ring
    chassis.turnToPoint(26, 40, 2000, {}, true);
    pros::delay(250);
    intake.set(Intake::IntakeState::INTAKING, 127);
    chassis.moveToPoint(26, 44, 2000, {}, true);
    pros::delay(200);

    // second ring
    chassis.moveToPoint(10, 43, 2000, {}, false);
    pros::delay(30);

    // touch bar
    chassis.turnToPoint(22, 26, 2000, {}, false);
    chassis.moveToPoint(22, 26, 3000,
                        {
                            .maxSpeed = 80,
                        },
                        false);
    chassis.moveToPoint(19, 10, 2000, {.maxSpeed = 70});
}

void elimBlueTopSide() {
    chassis.setPose(56, 11.5, 120);

    chassis.moveToPoint(21, 27, 2000, {.forwards = false, .maxSpeed = 50});
    // chassis.moveToPoint(-50, 12, 2000, {.forwards = false, .maxSpeed = 80});
    // chassis.waitUntilDone();
    chassis.waitUntil(30);
    clamp.set_value(true);

    intake.set(Intake::IntakeState::INTAKING, 127);

    chassis.turnToPoint(14, 40, 2000, {}, false);
    // intake.set(Intake::IntakeState::INTAKING);
    // chassis.moveToPose(-8.4, 50,0, 5000,{.minSpeed = 20},false);
    chassis.moveToPoint(14, 40, 2000, {.minSpeed = 50}, false);
    pros::delay(250);
    chassis.moveToPoint(8, 52, 2000, {}, false);
    // chassis.moveFor(7,2000,{.forwards = false},false);

    chassis.turnToPoint(26, 48, 2000, {}, true); // 30

    chassis.moveToPoint(26, 48, 2000, {.minSpeed = 80}, false);
    pros::delay(1000);
    intake.set(Intake::IntakeState::OUTTAKE, 127);

    chassis.moveToPose(60, 17, 180, 2000, {}, false);
    // chassis.waitUntil(5);
    // intake.set(Intake::IntakeState::OUTTAKE,127);
    // chassis.waitUntilDone();
    hook.set_value(true);
    pros::delay(250);
    chassis.moveFor(10, 2000, {.forwards = false}, false);
    intake.set(Intake::IntakeState::INTAKING);
    hook.set_value(false);
    chassis.moveToPoint(50, 14, 2000, {}, false); // -51,12

    chassis.turnToPoint(54, 54, 2000, {}, false); //-53
    chassis.moveToPose(54, 54, 24, 2000, {}, false);
    hook.set_value(true);
    pros::delay(500);
    chassis.turnToHeading(-90, 2000, {.minSpeed = 127}, false);

    // chassis.turnToPoint(13, -24, 2000, {.minSpeed = 50}, false);
    // chassis.moveToPoint(13, -24, 2000, {.minSpeed = 60}, false);
}

void elimRedTopSide() {
    chassis.setPose(-56, 11.5, -120);

    chassis.moveToPoint(-21, 27, 2000, {.forwards = false, .maxSpeed = 50});
    // chassis.moveToPoint(-50, 12, 2000, {.forwards = false, .maxSpeed = 80});
    // chassis.waitUntilDone();
    chassis.waitUntil(30);
    clamp.set_value(true);

    intake.set(Intake::IntakeState::INTAKING, 127);

    chassis.turnToPoint(-12, 40, 2000, {}, false);
    // intake.set(Intake::IntakeState::INTAKING);
    // chassis.moveToPose(-8.4, 50,0, 5000,{.minSpeed = 20},false);
    chassis.moveToPoint(-12, 40, 2000, {.minSpeed = 50}, false);
    pros::delay(250);
    chassis.moveToPoint(-10, 52, 2000, {}, false);
    chassis.moveFor(7, 2000, {.forwards = false}, false);

    chassis.turnToPoint(-28, 48, 2000, {}, true); // 30

    chassis.moveToPoint(-26, 48, 2000, {.minSpeed = 50}, false);
    chassis.moveToPose(-45, 17, 180, 2000, {}, false);

    hook.set_value(true);
    pros::delay(100);
    chassis.moveFor(10, 2000, {.forwards = false}, false);
    intake.set(Intake::IntakeState::INTAKING);
    hook.set_value(false);
    chassis.moveToPoint(-51, 14, 2000, {}, false); // -51,12

    chassis.turnToPoint(-54, 60, 2000, {}, false); //-53
    chassis.moveToPoint(-54, 61, 2000, {}, true); //-53
    chassis.waitUntil(10);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.turnToHeading(-75, 2000, {}, true);
    chassis.waitUntil(60);
    hook.set_value(true);
    pros::delay(500);
    chassis.swingToHeading(180, DriveSide::LEFT, 2000, {.minSpeed = 127}, false);

    // chassis.turnToPoint(-68,69,2000,{},false);
    // chassis.moveFor(60,2000,{.maxSpeed = 127},true);
    // chassis.waitUntil(10);
    // intake.set(Intake::IntakeState::OUTTAKE,127);
    // pros::delay(1000);
    // intake.set(Intake::IntakeState::INTAKING,127);
    // chassis.moveFor(10,2000,{.maxSpeed = 127},true);
    // chassis.moveFor(10,2000,{.forwards = false},true);
    // chassis.moveFor(5,2000,{},false);
    // chassis.moveFor(5,2000,{.forwards = false},false);

    // chassis.moveToPoint(-12,63,3000,{},false);
    // intake.set(Intake::IntakeState::STOPPED);
    // arm.scoreWallstake();
    // chassis.moveFor(10,2000,{.forwards = false},false);
    // arm.retract();
    // chassis.turnToPoint(13, -24, 2000, {.minSpeed = 50}, false);
    // chassis.moveToPoint(13, -24, 2000, {.minSpeed = 60}, false);
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
    // chassis.angularPID.setGains(5.2, 0, 45);
    // chassis.turnToHeading(90, 10000);
    // chassis.waitUntilDone();

    // chassis.turnToHeading(0,10000);
    // chassis.waitUntilDone();
    // chassis.lateralPID.setGains(26,0,150);

    // chassis.moveToPoint(0,-30, 10000, {.forwards=false, .maxSpeed = 80});
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0,0, 10000, {.maxSpeed = 80});
    // chassis.waitUntilDone();
    //halfAwpRed();
    //halfAwpBlue();
    // mogoRushRed();
    //mogoRushBlue();
    //awpRed();
    awpBlue();
    // elimRedTopSide();
    //elimBlueTopSide();

    //prog();
    //chassis.moveFor(12,2000,{},false);
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