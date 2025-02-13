#include "config.hpp"

inline void prog() {
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
    cata.load();
    chassis.waitUntilDone();
    pros::delay(200);

    // autistic ass mf of a wallstake
    chassis.moveToPoint(5.8, -45.5, 4000, {.forwards = false, .maxSpeed = 70}, false); // was 4.8
    chassis.turnToHeading(180, 3000, {.maxSpeed = 100});
    intake.set(Intake::IntakeState::STOPPED);
    pros::delay(200);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.waitUntilDone();
    chassis.moveFor(19.3, 1300, {.maxSpeed = 70}, false);
    // chassis.moveToPoint(4,-54.75,1000,{},false);

    intake.set(Intake::IntakeState::OUTTAKE, 30);
    // arm.scoreWallstake();
    chassis.moveFor(7.5, 1000, {.forwards = false}, false);
    // arm.retract();
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
    chassis.moveToPoint(-13, 28, 2000, {.minSpeed = 40, .earlyExitRange = 13}, true);
    chassis.waitUntil(5);
    intake.set(Intake::IntakeState::INTAKING, 120);
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
    chassis.moveToPoint(9, 46, 3000, {.maxSpeed = 60}, true); // was x = 10
    chassis.waitUntil(9);
    // arm.loadWallstake();
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToHeading(0, 1000);
    chassis.moveFor(18.5, 2000, {.maxSpeed = 60}, true);
    chassis.waitUntilDone();
    pros::delay(750);
    intake.set(Intake::IntakeState::STOPPED);
    // arm.scoreWallstake();
    // arm.retract();

    // holding one rings in the intake
    intake.set(Intake::IntakeState::INTAKING, 60);
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
    chassis.moveToPoint(59, 40, 2000, {.maxSpeed = 90, .minSpeed = 30, .earlyExitRange = 8}, false);
    chassis.moveToPoint(59, 57, 3000, {.maxSpeed = 90}, false);
    // chassis.moveFor(60,3000,{},true);
    chassis.turnToPoint(71, 65, 2000, {.forwards = false}, false);
    intake.set(Intake::IntakeState::OUTTAKE, 50);
    clamp.set_value(false);
    chassis.moveToPoint(72, 65, 1000, {.forwards = false}, false);

    chassis.moveToPoint(60, 55, 2000, {.minSpeed = 120, .earlyExitRange = 6});
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(95, -69, 5000, {.minSpeed = 100}, false);

}