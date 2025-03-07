#include "config.hpp"


inline void prog() {
    chassis.setPose(-63,0,90);
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    cata.edge();
    pros::delay(300);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-22,-24,2000,{.minSpeed = 50, .earlyExitRange = 10},false);
    chassis.moveToPoint(3,-59,2000,{.maxSpeed = 105},false); //3,-62
    chassis.turnToHeading(0,1500,{.maxSpeed = 80, .minSpeed = 20},false);

    //uncomment once raise is fixed
    chassis.moveFor(10, 750, {.forwards = false, .maxSpeed = 60}, true, true, 0);
    pros::delay(200);
    raiseLift();
    //pros::delay(200);
    cata.edge();
    pros::delay(300);
    lowerLift();

    chassis.moveToPoint(27,-50,2000,{},false);

    chassis.turnToPoint(34,-59,2000,{.minSpeed = 80},false);
    chassis.moveToPoint(35,-59,2000,{.maxSpeed = 90,.minSpeed = 50, .earlyExitRange = 10},false); //was .min 30

    chassis.moveToPoint(54,-63,2000,{.minSpeed = 50},false); //was .min 30

    // chassis.turnToPoint(-55,-26,2000,{.forwards = false},false);   //not completely necessary, removable for time sake
    chassis.moveToPoint(-56,-24.25,3000,{.forwards = false,.maxSpeed = 90, .slowDownRange = 50, .slowDownSpeed = 60}, true);
    chassis.waitUntil(106);
    clamp.set_value(true);
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);
    
    chassis.turnToPoint(-22, -54, 2000, {.minSpeed = 80},true);
    cata.edge();
    chassis.moveToPoint(-22,-54,2000,{.maxSpeed = 105},false);
    pros::delay(500); // cant spin while intaking???
    chassis.turnToPoint(-30,-62,2000,{.minSpeed = 80}, false);
    chassis.moveToPoint(-30,-62,2000,{.minSpeed = 40,.earlyExitRange = 3},false);
    chassis.moveToPoint(-48,-65,2000,{.minSpeed = 40},false); // was .min = 0
    pros::delay(250);
    cata.edge();
    
    chassis.swingToHeading(120,DriveSide::RIGHT,2000,{.direction = AngularDirection::CW_CLOCKWISE,.minSpeed = 50},false);
    chassis.moveFor(15,2000,{.minSpeed = 50},false);
    pros::delay(300);
    intake.set(Intake::IntakeState::OUTTAKE,100);

    // chassis.turnToPoint(-67,-65,2000,{.minSpeed = 80},false);
    chassis.moveToPoint(-65,-67,1000,{.forwards = false,.minSpeed = 127},true);
    cata.edge();
    chassis.waitUntil(10);
    clamp.set_value(false);
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    intake.set(Intake::IntakeState::INTAKING);

    //MOGO 2
    chassis.moveToPoint(35,35,4000,{.slowDownRange = 24, .slowDownSpeed = 60},false);
    chassis.moveToPoint(62,-7,2000,{.forwards = false,.maxSpeed = 80},true);
    chassis.waitUntil(46);
    clamp.set_value(true);
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);

    chassis.turnToPoint(24,-24,2000,{.minSpeed = 60},false);
    cata.edge();
    chassis.moveToPoint(26,-21,2000,{.minSpeed = 80},false);
    chassis.turnToPoint(50,-48,2000,{.minSpeed = 80},false);
    chassis.moveToPoint(55,-52,2000,{.maxSpeed = 95,.slowDownRange = 21,.slowDownSpeed = 30},false);
    pros::delay(500);
    intake.set(Intake::IntakeState::STOPPED);
    cata.edge();
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveFor(8,2000,{},false);
    chassis.turnToHeading(30,2000,{.minSpeed = 40},false);
    chassis.moveFor(16,2000,{},false);
    pros::delay(200);
    intake.set(Intake::IntakeState::OUTTAKE);
    
    chassis.turnToHeading(0,2000,{.minSpeed = 100},false);
    // chassis.moveToPoint(68,-60,1000,{.forwards = false,.minSpeed = 127},true);
    chassis.moveFor(20,1000,{.forwards = false,.minSpeed = 127},true);
    cata.edge();
    clamp.set_value(false);
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);

    chassis.moveToPoint(30,46,2000,{.minSpeed = 30,.earlyExitRange = 10},true);
    chassis.waitUntil(20);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(4,62,2000,{},false);
    chassis.turnToHeading(180,2000,{.minSpeed = 80},false);
    
    raiseLift();
    chassis.moveFor(15, 500, {.forwards = false,.minSpeed = 30}, false);
    cata.edge();
    lowerLift();

    chassis.moveToPoint(49, 51, 2000,{.minSpeed = 30, .earlyExitRange = 10},false);
    chassis.moveToPoint(64,50,2000,{.minSpeed = 50},false);
    chassis.moveToPoint(41,34,2000,{.forwards = false,.minSpeed = 30, .earlyExitRange = 10},false);
    // intake.set(Intake::IntakeState::OUTTAKE);
    chassis.moveToPoint(61,1,2000,{.forwards=false}, false);
    chassis.turnToHeading(-90,2000, {.minSpeed = 80},false);
    
    chassis.moveFor(10, 750, {.forwards = false}, false);
    cata.edge();

    intake.set(Intake::IntakeState::STOPPED);
    chassis.moveFor(20,2000,{.minSpeed = 30},false);
    chassis.turnToPoint(70,60,2000,{.forwards = false, .minSpeed = 80},false);
    chassis.moveToPoint(70,60,2000,{.forwards = false,.minSpeed = 120},false);
    chassis.turnToHeading(-90,2000,{.minSpeed = 80},false);

    
    

    // chassis.moveFor(10,2000,{.forwards = false},false);
    // chassis.moveToPoint(-61,-50,2000,{},false);
}