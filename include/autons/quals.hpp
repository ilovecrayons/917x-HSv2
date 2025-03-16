#include "config.hpp"

inline void awpRed() {
    intake.set(Intake::IntakeState::INTAKING);
    chassis.setPose(-55,24,-90);
    chassis.moveToPoint(-16,22,2000,{.forwards = false, .maxSpeed = 70},true);
    chassis.waitUntil(26);
    clamp.set_value(true);
    cata.edge(true);
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);

    chassis.turnToPoint(-7,36,2000,{.minSpeed = 40},true);
    chassis.moveToPoint(-7,36,2000,{.minSpeed = 20},false);

    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 70});
    
    chassis.moveFor(5,1000,{.forwards = false},false);

    chassis.turnToPoint(-24,46,2000,{.minSpeed = 80, .earlyExitRange = 2},true);

    chassis.moveToPoint(-23,45,2000,{.maxSpeed = 80},false);
    chassis.turnToPoint(-58,2,2000,{.minSpeed = 80},false);
    pros::delay(3000);
    chassis.moveToPoint(-58,2,2000,{},true);
    chassis.waitUntil(20);
    clamp.set_value(false);
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    intake.set(Intake::IntakeState::OUTTAKE, 127);
    chassis.turnToHeading(90,2000,{.minSpeed = 80, .earlyExitRange = 2},false);
    chassis.moveFor(10,750,{.forwards = false, .minSpeed = 127},false);
    cata.edge();
    // pros::delay(200);

    // chassis.moveToPoint(-26,-48,2000,{.minSpeed = 80},true);
    // chassis.waitUntil(20);
    // intake.set(Intake::IntakeState::INTAKING);

    // chassis.turnToPoint(-24,-20,2000,{.forwards = false,.minSpeed = 80, .earlyExitRange = 2},false);
    // chassis.moveToPoint(-24,-20,2000,{.forwards = false,.maxSpeed = 70},true);
    // chassis.waitUntil(26);
    // clamp.set_value(true);
    // cata.edge();
    // raiseLift();
    // // chassis.moveToPoint(-18,-14,2000,{.forwards = false},false);
    // chassis.swingToHeading(-90,DriveSide::LEFT,2000,{},false);


}

inline void awpBlue() {
    intake.set(Intake::IntakeState::INTAKING);
    chassis.setPose(55,24,90);
    chassis.moveToPoint(16,22,2000,{.forwards = false, .maxSpeed = 70},true);
    chassis.waitUntil(26);
    clamp.set_value(true);
    cata.edge(true);
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);

    chassis.turnToPoint(7,36,2000,{.minSpeed = 40},true);
    chassis.moveToPoint(7,36,2000,{.minSpeed = 20},false);

    chassis.swingToHeading(90, lemlib::DriveSide::LEFT, 1000, {.minSpeed = 70});
    
    chassis.moveFor(5,1000,{.forwards = false},false);

    chassis.turnToPoint(24,46,2000,{.minSpeed = 80, .earlyExitRange = 2},true);

    chassis.moveToPoint(23,45,2000,{.maxSpeed = 80},false);

    chassis.turnToPoint(58,2,2000,{.minSpeed = 80},false);
    // pros::delay(3000);

    // chassis.moveToPoint(58,2,2000,{},true);
    // chassis.waitUntil(20);
    // clamp.set_value(false);
    // chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    // intake.set(Intake::IntakeState::OUTTAKE, 127);
    // chassis.turnToHeading(-90,2000,{.minSpeed = 80, .earlyExitRange = 2},false);
    // chassis.moveFor(10,750,{.forwards = false, .minSpeed = 127},false);
    cata.edge();
    
    // chassis.moveToPoint(26,-48,2000,{.minSpeed = 80},true);
    // chassis.waitUntil(20);
    // intake.set(Intake::IntakeState::INTAKING);

    // chassis.turnToPoint(24,-20,2000,{.forwards = false,.minSpeed = 80, .earlyExitRange = 2},false);
    // chassis.moveToPoint(24,-20,2000,{.forwards = false,.maxSpeed = 70},true);
    // chassis.waitUntil(24);
    // clamp.set_value(true);
    // cata.edge();
    // raiseLift();
    // // chassis.moveToPoint(-18,-14,2000,{.forwards = false},false);
    // chassis.swingToHeading(90,DriveSide::RIGHT,2000,{},false);
}

inline void halfAwpRed() {
    chassis.setPose(-56, 11.5, -120);
    // arm.scoreWallstake(125, true, 7);
    pros::delay(500);
    chassis.moveFor(12, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(-47, 12, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 5});

    chassis.moveToPoint(-20, 28.5, 2000, {.forwards = false, .maxSpeed = 60});

    pros::delay(500);
    // arm.retract();
    //  chassis.waitUntilDone();
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

inline void halfAwpBlue() {
    chassis.setPose(56, 11.5, 120);
    // arm.scoreWallstake(125, true, 7);
    pros::delay(500);
    chassis.moveFor(12, 2000);
    chassis.waitUntilDone();
    chassis.moveToPoint(47, 12, 2000, {.forwards = false, .maxSpeed = 80, .minSpeed = 40, .earlyExitRange = 5});

    chassis.moveToPoint(20, 28.5, 2000, {.forwards = false, .maxSpeed = 60});

    pros::delay(500);
    // arm.retract();
    //  chassis.waitUntilDone();
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