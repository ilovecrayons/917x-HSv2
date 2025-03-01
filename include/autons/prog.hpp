#include "config.hpp"

inline void raiseHook(){
    hook.set_value(true);
    cata.score(55, true, 4);
}

void lowerHook(){
    hook.set_value(false);
}


inline void prog() {
    chassis.setPose(-63,0,90);
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    cata.edge();
    pros::delay(300);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-22,-24,2000,{.maxSpeed = 105, .minSpeed = 50, .earlyExitRange = 10},false);
    chassis.moveToPoint(2.5,-58,2000,{.maxSpeed = 105},false); //3,-62
    chassis.turnToHeading(0,2000,{.minSpeed = 80},false);

    //uncomment once raise is fixed
    // raiseHook();
    // // hook.set_value(true);
    // pros::delay(300);
    chassis.moveFor(10,750,{.forwards = false}, false, true, 0);
    // pros::delay(200);
    // cata.edge();
    // lowerHook();
    pros::delay(500);

    chassis.moveToPoint(27,-50,2000,{},false);

    chassis.turnToPoint(34,-59,2000,{.minSpeed = 80},false);
    chassis.moveToPoint(35,-58,2000,{.maxSpeed = 90,.minSpeed = 30, .earlyExitRange = 6},false);

    chassis.moveToPoint(52,-63,2000,{},false);

    // chassis.turnToPoint(-55,-26,2000,{.forwards = false},false);   //not completely necessary, removable for time sake
    chassis.moveToPoint(-56,-24.25,3000,{.forwards = false,.maxSpeed = 80}, true);
    chassis.waitUntil(105);
    clamp.set_value(true);
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);
    
    chassis.turnToPoint(-22, -54, 2000, {.minSpeed = 80},false);
    cata.edge();
    chassis.moveToPoint(-22,-54,2000,{.maxSpeed = 105},false);
    pros::delay(500); // cant spin while intaking???
    chassis.turnToPoint(-34,-64,2000,{.minSpeed = 80}, false);
    chassis.moveToPoint(-32,-65,2000,{.minSpeed = 40,.earlyExitRange = 3},false);
    chassis.moveToPoint(-47,-67,2000,{},false);
    pros::delay(250);
    cata.edge();
    
    chassis.swingToHeading(90,DriveSide::RIGHT,2000,{.direction = AngularDirection::CW_CLOCKWISE,.minSpeed = 40},false);
    chassis.moveFor(10,2000,{},false);
    pros::delay(300);
    cata.edge();

    chassis.moveToPoint(-59,-64,2000,{.forwards = false,.minSpeed = 90},true);
    chassis.waitUntil(3);
    clamp.set_value(false);
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);


    chassis.moveToPoint(35,35,4000,{.maxSpeed = 100},false);
    chassis.moveToPoint(59,-7,2000,{.forwards = false,.maxSpeed = 80},true);
    chassis.waitUntil(35);
    clamp.set_value(true);
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);

    chassis.turnToPoint(24,-24,2000,{.minSpeed = 80},true);
    cata.edge();
    chassis.moveToPoint(24,-24,2000,{.minSpeed = 30,.earlyExitRange = 8},false);
    chassis.moveToPoint(46,-47,2000,{.maxSpeed = 60},false);
    cata.edge();
    



    // chassis.moveFor(10,2000,{.forwards = false},false);
    // chassis.moveToPoint(-61,-50,2000,{},false);
}