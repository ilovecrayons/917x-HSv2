#include "autons/topSide.hpp"

void awpRed() {
    chassis.setPose(-56, 11.5, -120);
    arm.scoreWallstake(125, true, 10);
    pros::delay(500);
    chassis.moveFor(12.5, 2100);
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
    intake.set(Intake::IntakeState::STOPPED);

    chassis.moveToPoint(-44, -45, 3000, {.maxSpeed = 70}, true);
    chassis.waitUntil(15);
    clamp.set_value(false);

    // clamp next goal
    chassis.turnToPoint(-21, -26, 2000, {.forwards = false}, false);
    chassis.moveToPoint(-21, -26, 2000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.waitUntil(25);
    clamp.set_value(true);

    // scoring next ring
    chassis.moveToPoint(-21, -50, 2000, {}, true);
    pros::delay(500);
    intake.set(Intake::IntakeState::INTAKING, 127);

    // touching bar
    chassis.turnToPoint(-13, -24, 2000, {.minSpeed = 50}, false);
    chassis.moveToPoint(-13, -24, 2000, {.minSpeed = 60}, false);
}
void awpBlue() {
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
    intake.set(Intake::IntakeState::STOPPED);

    chassis.moveToPoint(44, -45, 3000, {.maxSpeed = 70}, true);
    chassis.waitUntil(15);
    clamp.set_value(false);

    // clamp next goal
    chassis.turnToPoint(21, -26, 2000, {.forwards = false}, false);
    chassis.moveToPoint(21, -26, 2000, {.forwards = false, .maxSpeed = 70}, true);
    chassis.waitUntil(25);
    clamp.set_value(true);

    // scoring next ring
    chassis.moveToPoint(21, -50, 2000, {}, true);
    pros::delay(500);
    intake.set(Intake::IntakeState::INTAKING, 127);

    // touching bar
    chassis.turnToPoint(13, -24, 2000, {.minSpeed = 50}, false);
    chassis.moveToPoint(13, -24, 2000, {.minSpeed = 60}, false);
}
void topRedWithbar() {}
void topBlueWithBar() {}

void topElimRed() {
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

void topElimBlue() {
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