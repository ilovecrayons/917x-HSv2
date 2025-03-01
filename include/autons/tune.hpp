#include "lemlib/chassis/chassis.hpp"
#include "config.hpp"

inline void tune() {
    // chassis.setPose(0,0,0);
    // chassis.moveFor(24, 10000);
    // chassis.waitUntilDone();
    // chassis.moveFor(24, 10000, {.forwards=false, .maxSpeed = 70});
    // chassis.waitUntilDone();
    // chassis.moveFor(24, 10000);
    // chassis.waitUntilDone();
    // chassis.moveFor(24, 10000, {.forwards=false, .maxSpeed = 70});
    // chassis.waitUntilDone();
    // chassis.angularPID.setGains(5.2, 0, 45);
    // chassis.turnToHeading(90, 10000);
    // chassis.waitUntilDone();

    // chassis.turnToHeading(0,10000);
    // chassis.waitUntilDone();
    // chassis.lateralPID.setGains(26,0,150);
    chassis.setPose(0, 0, 0);
    //chassis.defaultConstants = {10, 0.01, 80, 4.7, 0.1, 51};
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);
    clamp.set_value(true);
    pros::delay(500);
    chassis.moveToPoint(0, 30, 10000,{.maxSpeed = 100});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 0, 10000, {.forwards = false,.maxSpeed = 100});
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 10000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 10000);
    chassis.waitUntilDone();
    chassis.turnToHeading(140, 10000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 10000);
    chassis.waitUntilDone();
    clamp.set_value(false);

    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    pros::delay(500);
    chassis.moveToPoint(0, 30, 10000,{.maxSpeed = 100});
    chassis.waitUntilDone();
    chassis.moveToPoint(0, 0, 10000, {.forwards = false,.maxSpeed = 100});
    chassis.waitUntilDone();
    chassis.turnToHeading(90, 10000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 10000);
    chassis.waitUntilDone();
    chassis.turnToHeading(140, 10000);
    chassis.waitUntilDone();
    chassis.turnToHeading(0, 10000);
    chassis.waitUntilDone();


    // chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    // pros::delay(500);

    // chassis.moveToPoint(0,30, 10000);
    // chassis.waitUntilDone();
    // chassis.moveToPoint(0,0, 10000, {.forwards = false});
    // chassis.waitUntilDone();
    // chassis.turnToHeading(90, 10000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(0,10000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(90, 10000);
    // chassis.waitUntilDone();
    // chassis.turnToHeading(0,10000);
    // chassis.waitUntilDone();
}