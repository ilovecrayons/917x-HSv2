#include "config.hpp"
#include <iostream>
using namespace lemlib;

inline void prog() {
    chassis.setPose(-63,0,90);
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    cata.edge();
    pros::delay(300);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-22,-24,2000,{.minSpeed = 50, .earlyExitRange = 10},false);
    chassis.moveToPoint(3,-59,2000,{.maxSpeed = 105},false); //3,-62
    chassis.turnToHeading(0,1500,{.maxSpeed = 80, .minSpeed = 20, .earlyExitRange = 1},false);

    //uncomment once raise is fixed
    chassis.moveFor(10, 750, {.forwards = false, .maxSpeed = 60}, true, true, 0);
    pros::delay(200);
    raiseLift();
    //pros::delay(200);
    cata.edge();
    pros::delay(300);
    lowerLift();

    chassis.moveToPoint(27,-50,2000,{.minSpeed = 30, .earlyExitRange = 3},false);

    chassis.turnToPoint(40,-59,2000,{.minSpeed = 80, .earlyExitRange = 2},false);
    chassis.moveToPoint(40,-59,2000,{.maxSpeed = 90,.minSpeed = 50, .earlyExitRange = 5},false); //was .min 30

    chassis.moveToPoint(51,-65,2000,{.minSpeed = 80},false); //was .min 30
    pros::delay(200);

    // chassis.turnToPoint(-55,-26,2000,{.forwards = false},false);   //not completely necessary, removable for time sake
    chassis.moveToPoint(-56,-24.25,3000,{.forwards = false,.maxSpeed = 110, .minSpeed = 50, .earlyExitRange = 2, .slowDownRange = 45, .slowDownSpeed = 60}, true);
    chassis.waitUntil(30);
    intake.set(Intake::IntakeState::OUTTAKE);
    chassis.waitUntil(108);
    clamp.set_value(true);
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);
    chassis.waitUntilDone();
    chassis.turnToPoint(-20, -54, 2000, {.minSpeed = 80},true);
    intake.set(Intake::IntakeState::INTAKING);
    cata.edge(true);
    chassis.moveToPoint(-20,-54,2000,{.maxSpeed = 100, .minSpeed = 30, .earlyExitRange = 2},false);
    pros::delay(400); // cant spin while intaking???
    chassis.turnToPoint(-30,-63,2000,{.minSpeed = 80}, false);
    chassis.moveToPoint(-30,-63,2000,{.minSpeed = 40,.earlyExitRange = 3},false);
    chassis.moveToPoint(-48,-66,2000,{.minSpeed = 40},false); // was .min = 0
    pros::delay(250);
    cata.edge(true);
    pros::delay(300);
    
    chassis.swingToHeading(120,DriveSide::RIGHT,2000,{.direction = AngularDirection::CW_CLOCKWISE,.minSpeed = 50},false);
    chassis.moveFor(10,700,{.minSpeed = 50},false);
    intake.set(Intake::IntakeState::OUTTAKE,100);

    chassis.turnToPoint(-67,-70,2000,{.forwards = false, .minSpeed = 100, .earlyExitRange = 2},false);
    
    chassis.moveToPoint(-67,-73,750,{.forwards = false},true);

    cata.edge();
    pros::delay(200);
    clamp.set_value(false);
    chassis.waitUntilDone();
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    intake.set(Intake::IntakeState::INTAKING);

    
    //MOGO 2
    chassis.moveToPoint(35,35,4000,{.minSpeed = 20, .earlyExitRange = 2, .slowDownRange = 40, .slowDownSpeed = 80},false);
    chassis.moveToPoint(59,-3,2000,{.forwards = false,.maxSpeed = 80},true);
    chassis.waitUntil(46);
    clamp.set_value(true);
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);
    chassis.waitUntilDone();

    //turn to da wall
    chassis.turnToHeading(180, 1000, {.minSpeed = 30}, false);
    Pose reset1 = chassis.getPose();

    cata.edge(true);

    std::pair<float, float> reset1x = distReset.getDistance(DistanceReset::Wall::BOTTOM);
    chassis.setPose(reset1x.first, reset1x.second, reset1.theta);
    reset1 = chassis.getPose();
    std::cout << "reset1: " << reset1.x << " " << reset1.y << " " << reset1.theta << std::endl;

    chassis.turnToPoint(24,-24,2000,{.minSpeed = 60, .earlyExitRange = 2},false);
    
    chassis.moveToPoint(24,-24,2000,{.minSpeed = 80},false);
    pros::delay(300);
    chassis.turnToPoint(48,-49,2000,{.minSpeed = 80, .earlyExitRange = 2},false);
    chassis.moveToPoint(48,-49,2000,{.maxSpeed = 95},false);
    chassis.moveFor(10,2000,{.forwards = false},false);
    pros::delay(200);
    intake.set(Intake::IntakeState::STOPPED);
    cata.edge();
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(59,-51,1000,{.minSpeed = 80},false);
    chassis.moveToPoint(59, -51, 1000, {.minSpeed = 65}, false);
    
    pros::delay(200);
    intake.set(Intake::IntakeState::OUTTAKE);
    
    
    chassis.turnToPoint(70, -67, 1000, {.forwards = false, .minSpeed = 80}, false);
    cata.edge();
    chassis.moveToPoint(70,-67,1000,{.forwards = false,.minSpeed = 110},true);
    clamp.set_value(false);
    chassis.waitUntilDone();
    
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);

    //return;

    //WALLSTAKE 2
    chassis.moveToPoint(28,50,3000,{.maxSpeed = 110, .minSpeed = 30,.earlyExitRange = 2},true); //was 30,50
    chassis.waitUntil(20);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(5, 62, 2000,{.minSpeed = 80, .earlyExitRange = 1.5},false);
    chassis.moveToPoint(5,62,2000,{},false);
    chassis.turnToHeading(-180,2000,{.minSpeed = 80},false);
    
    
    chassis.moveFor(15, 500, {.forwards = false,.minSpeed = 30}, true);
    Pose pose = chassis.getPose();
    chassis.setPose(pose.x, 66, pose.theta);
    pros::delay(200);
    raiseLift();
    chassis.waitUntilDone();
    cata.edge();
    lowerLift();

    chassis.moveToPoint(51, 48+7, 2000,{.minSpeed = 50},false);
    // chassis.moveToPoint(65,50,2000,{.minSpeed = 70,.earlyExitRange = 2},false); //64,50
    chassis.moveToPoint(41,34+7,2000,{.forwards = false,.minSpeed = 40, .earlyExitRange = 10},false);
    intake.set(Intake::IntakeState::OUTTAKE); 
    chassis.moveToPoint(61,-0.5,2000,{.forwards=false}, false);
    chassis.turnToHeading(-90,2000, {.minSpeed = 80},false);
    
    chassis.moveFor(13, 750, {.forwards = false,.earlyExitRange = 2}, false);
    cata.edge();

    // intake.set(Intake::IntakeState::STOPPED);
    chassis.moveFor(15,2000,{.minSpeed = 30},false);
    chassis.turnToPoint(70,60,2000,{.forwards = false, .minSpeed = 80,},false);
    chassis.moveToPoint(70,60,2000,{.forwards = false,.minSpeed = 120,.earlyExitRange = 2},false);
    chassis.turnToHeading(-60,2000,{.minSpeed = 80},false);


}