#include "autons/skills.hpp"

void skills() {
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
    chassis.moveToPose(-23, -24, 90, 2000, {.minSpeed = 90}, true);
    chassis.waitUntil(5);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(-20, -50, 2000, {}, false);
    chassis.moveToPoint(-20, -50, 2000, {.maxSpeed = 65}, false);
    chassis.turnToPoint(8, -64, 2000, {}, false);
    chassis.moveToPoint(8, -64, 2000, {.maxSpeed = 65, .minSpeed = 30,.earlyExitRange = 6});
    chassis.moveToPoint(28, -54, 2000, {.maxSpeed = 65}, false);
    pros::delay(250);

    chassis.moveToPoint(51, -52, 2000, {.maxSpeed = 75}, true);
    chassis.waitUntil(15);
    arm.loadWallstake();
    chassis.waitUntilDone();
    pros::delay(200);
//
    // autistic ass mf of a wallstake
    chassis.moveToPoint(6.3, -45.5, 3000, {.forwards = false, .maxSpeed = 70}, false); // was 0.7
    chassis.turnToHeading(180, 3000, {.maxSpeed = 100});
    intake.set(Intake::IntakeState::STOPPED);
    pros::delay(200);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.waitUntilDone();
    chassis.moveFor(22, 1000, {.maxSpeed = 70},false);
    // chassis.moveToPoint(4,-54.75,1000,{},false);

    intake.set(Intake::IntakeState::OUTTAKE, 30);
    arm.scoreWallstake();
    arm.retract();
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveFor(5, 1000, {.forwards = false}, false);

    chassis.turnToPoint(-40, -55.5, 2000, {}, false);
    chassis.moveToPose(-40,-55.5,-90,3000,{.maxSpeed = 70},false);

    chassis.moveFor(14, 3000, {.maxSpeed = 80});
    pros::delay(100);

    chassis.moveFor(13,1000,{.forwards = false},false);
    chassis.turnToPoint(-57,-69, 2000, {.forwards = false, .direction = AngularDirection::CCW_COUNTERCLOCKWISE}, false);
    intake.set(Intake::IntakeState::OUTTAKE, 50);
    chassis.moveToPoint(-57, -69, 2000, {.forwards = false}, false);
    clamp.set_value(false);
    pros::delay(200);

    // get next mobile goal
    chassis.moveToPoint(-45, 0, 4000, {.maxSpeed = 70}, false);
    chassis.turnToHeading(180, 2000, {}, false);
    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveToPoint(-45.5, 24, 2000, {.forwards = false, .maxSpeed = 60}, false);
    chassis.waitUntil(21);
    clamp.set_value(true);
    chassis.waitUntilDone();
    pros::delay(200);

    chassis.turnToPoint(-20, 22, 2000, {}, true);
    pros::delay(300);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-20, 22, 2000, {}, false);
    pros::delay(300);
    chassis.moveToPose(9, -11, 90 + 45, 5000, {}, false);
    chassis.turnToHeading(-30, 1000, {}, false);

    chassis.moveToPoint(-17,28,2000,{.minSpeed = 40, .earlyExitRange = 13},true);
    chassis.moveToPoint(-17, 42, 3000, {.maxSpeed = 80}, false);
    //chassis.moveToPose(-22, 43, 0, 2000, {}, false);
    chassis.turnToPoint(-62, 48, 1000, {.maxSpeed = 90}, false);
    pros::delay(250);
    chassis.moveToPoint(-40,48,1000,{},false);
    chassis.moveToPoint(-51, 48, 1000, {.maxSpeed = 65}, false);
    pros::delay(500);
    chassis.moveToPoint(-27, 45, 1000, {.forwards = false}, false);
    chassis.moveToPoint(-47, 62, 1000, {}, false);
    pros::delay(250);

    // releasing stake
    chassis.moveFor(8, 1000, {.forwards = false}, false);
    chassis.turnToPoint(-61,65, 1000, {}, false);
    chassis.moveToPoint(-61, 65, 1000, {.forwards = false}, false);
    clamp.set_value(false);
    intake.set(Intake::IntakeState::OUTTAKE, 30);

//score second wallstake
    chassis.moveToPoint(11, 45,3000,{.maxSpeed = 70},true);
    chassis.waitUntil(8);
    arm.loadWallstake();
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToHeading(0, 1000);
    chassis.moveFor(20,2000,{.maxSpeed = 60},true);
    chassis.waitUntilDone();
    pros::delay(500);
    intake.set(Intake::IntakeState::OUTTAKE, 30);
    arm.scoreWallstake();
    arm.retract();
    
    // holding one rings in the intake
    intake.set(Intake::IntakeState::INTAKING, 80);
    chassis.moveToPoint(38, 20, 2000, {.maxSpeed = 70}, false);
    pros::delay(250);
    chassis.turnToPoint(65, -5,300,
                        {
                            .forwards = false,
                        },
                        true);
    pros::delay(200);
    intake.set(Intake::IntakeState::STOPPED);

    // clamping new mobile goal
    chassis.moveToPoint(67,4, 3000, {.forwards = false, .maxSpeed = 70}, false);
    clamp.set_value(true);
    pros::delay(200);
    chassis.turnToHeading(-5, 2000, {}, false);
    intake.set(Intake::INTAKING);
    chassis.moveToPoint(60,55, 3000, {}, false);
    // chassis.moveFor(60,3000,{},true);
    chassis.turnToPoint(69,65,2000,{.forwards = false},false);
    clamp.set_value(false);
    chassis.moveToPoint(69,65,2000,{.forwards = false},false);
    intake.set(Intake::IntakeState::OUTTAKE,50);
    chassis.moveToPoint(76,24,2000,{.minSpeed = 120,.earlyExitRange = 6});
    chassis.moveToPoint(78,-69,5000,{.minSpeed = 120},false);

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