#include "config.hpp"

inline void raiseHook(){
    hook.set_value(true);
    cata.score(55,false,4);
}

void lowerHook(){
    pros::delay(200);
    hook.set_value(false);
}


inline void prog() {
    chassis.setPose(-63,0,90);
    cata.edge();
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-22,-24,2000,{.minSpeed = 50, .earlyExitRange = 10},false);
    chassis.moveToPoint(3,-58,2000,{},false); //3,-62
    chassis.turnToHeading(0,2000,{},false);
    chassis.moveFor(5,2000,{.forwards = false},false);

    //add the following back once inertial is fixed
    pros::delay(500);
    // raiseHook();
    // pros::delay(1000);
    // cata.edge();
    // lowerHook();
    chassis.moveToPoint(24,-50,2000,{},false);

    chassis.turnToPoint(35,-58,2000,{},false);
    chassis.moveToPoint(35,-58,2000,{.maxSpeed = 90,.minSpeed = 30, .earlyExitRange = 8},false);
    chassis.moveToPoint(52,-60,2000,{},false);

    chassis.turnToPoint(-55,-26,2000,{.forwards = false},false);   //not completely necessary, removable for time sake
    chassis.moveToPoint(-56,-24.25,3000,{.forwards = false,.maxSpeed = 80},true);
    chassis.waitUntil(105);
    clamp.set_value(true);
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);
    pros::delay(200);
    cata.edge();

    chassis.moveToPoint(-19,-52,2000,{},false);
    chassis.turnToPoint(-34,-64,2000,{},false);
    chassis.moveToPoint(-32,-62,2000,{.minSpeed = 40,.earlyExitRange = 3},false);
    chassis.moveToPoint(-47,-65,2000,{},false);

    pros::delay(400);
    cata.edge();
    
    chassis.swingToHeading(90,DriveSide::RIGHT,2000,{.direction = AngularDirection::CW_CLOCKWISE},false);
    chassis.moveFor(10,2000,{},false);
    pros::delay(400);
    cata.edge();
    // chassis.moveFor(10,2000,{.forwards = false},false);
    // chassis.moveToPoint(-61,-50,2000,{},false);
}