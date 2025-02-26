#include "config.hpp"

inline void raiseHook(){
    hook.set_value(true);
    cata.score(40,false);
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
    chassis.moveToPoint(2,-58,2000,{},false); //3,-62
    chassis.turnToPoint(3,-70,2000,{.forwards = false},false);
    chassis.moveFor(5,2000,{.forwards = false},false);
    chassis.turnToPoint(2,-70,2000,{.forwards = false},false);
    raiseHook();
    pros::delay(1000);
    cata.edge();
    lowerHook();

}