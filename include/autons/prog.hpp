#include "config.hpp"

inline void raiseLift(){
    pros::Task raise {[=]{lift.set_value(true);
    //pros::delay(300);
    //cata.score(55,true,3);
    }};
}

inline void lowerLift(){
    lift.set_value(false);
}


inline void prog() {
    chassis.setPose(-63,0,90);
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    cata.edge();
    pros::delay(300);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-22,-24,2000,{.maxSpeed = 105, .minSpeed = 50, .earlyExitRange = 10},false);
    chassis.moveToPoint(2,-58,2000,{.maxSpeed = 105},false); //3,-62
    chassis.turnToHeading(0,1500,{.maxSpeed = 80, .minSpeed = 20},false);

    //uncomment once raise is fixed
    raiseLift();
    chassis.moveFor(10, 750, {.forwards = false}, true, true, 0);
    pros::delay(500);
    cata.edge();
    lowerLift();

    chassis.moveToPoint(27,-50,2000,{},false);

    chassis.turnToPoint(34,-59,2000,{.minSpeed = 80},false);
    chassis.moveToPoint(35,-59,2000,{.maxSpeed = 90,.minSpeed = 30, .earlyExitRange = 6},false);

    chassis.moveToPoint(52,-64,2000,{},false);

    // chassis.turnToPoint(-55,-26,2000,{.forwards = false},false);   //not completely necessary, removable for time sake
    chassis.moveToPoint(-56,-24.25,3000,{.forwards = false,.maxSpeed = 90, .slowDownRange = 50, .slowDownSpeed = 60}, true);
    chassis.waitUntil(106);
    clamp.set_value(true);
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);
    
    chassis.turnToPoint(-22, -54, 2000, {.minSpeed = 80},true);
    cata.edge();
    chassis.moveToPoint(-22,-54,2000,{.maxSpeed = 105},false);
    pros::delay(500); // cant spin while intaking???
    chassis.turnToPoint(-34,-64,2000,{.minSpeed = 80}, false);
    chassis.moveToPoint(-32,-65,2000,{.minSpeed = 40,.earlyExitRange = 3},false);
    chassis.moveToPoint(-48,-67,2000,{},false); // from -47
    pros::delay(250);
    cata.edge();
    
    chassis.swingToHeading(90,DriveSide::RIGHT,2000,{.direction = AngularDirection::CW_CLOCKWISE,.minSpeed = 40},false);
    chassis.moveFor(13,2000,{},false);
    pros::delay(300);
    intake.set(Intake::IntakeState::OUTTAKE,100);
    cata.edge();

    chassis.moveToPoint(-59,-65,1000,{.forwards = false,.minSpeed = 90},true);
    chassis.waitUntil(10);
    clamp.set_value(false);
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    intake.set(Intake::IntakeState::INTAKING);

    //MOGO 2
    chassis.moveToPoint(35,35,4000,{.maxSpeed = 105, .slowDownRange = 24, .slowDownSpeed = 60},false);
    chassis.moveToPoint(59,-7,2000,{.forwards = false,.maxSpeed = 80},true);
    chassis.waitUntil(46);
    clamp.set_value(true);
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);

    chassis.turnToPoint(24,-24,2000,{.minSpeed = 60},false);
    cata.edge();
    chassis.moveToPoint(24,-24,2000,{},false);
    chassis.turnToPoint(50,-48,2000,{.minSpeed = 80},false);
    chassis.moveToPoint(50,-48,2000,{.maxSpeed = 100,.slowDownRange = 17,.slowDownSpeed = 30},false);
    pros::delay(500);
    cata.edge();
    chassis.moveFor(10,2000,{},false);
    chassis.turnToHeading(40,2000,{.minSpeed = 40},false);
    chassis.moveFor(16,2000,{},false);
    pros::delay(200);
    intake.set(Intake::IntakeState::STOPPED);
    cata.edge();
    
    chassis.moveToPoint(68,-60,2000,{.forwards = false,.minSpeed = 80},true);
    clamp.set_value(false);
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);

    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(30,48,2000,{.minSpeed = 30,.earlyExitRange = 10},false);
    chassis.moveToPoint(2,59,2000,{},false);
    chassis.turnToHeading(180,2000,{},false);
    
    raiseLift();
    chassis.moveFor(10, 750, {.forwards = false}, false, true, 0);
    cata.edge();
    lowerLift();

    chassis.moveToPoint(49, 47, 2000,{.minSpeed = 30, .earlyExitRange = 10},false);
    chassis.moveToPoint(61,48,2000,{},false);

    // chassis.moveFor(10,2000,{.forwards = false},false);
    // chassis.moveToPoint(-61,-50,2000,{},false);
}