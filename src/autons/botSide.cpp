#include "autons/botSide.hpp"

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
