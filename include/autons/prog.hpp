//#include "chassis.hpp"
#include "config.hpp"
#include <iostream>
using namespace lemlib;

inline void prog() {
    chassis.setPose(-63,0,90);
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    cata.edge();
    pros::delay(150);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.moveToPoint(-22,-24,2000,{.minSpeed = 50, .earlyExitRange = 10},false);
    chassis.moveToPoint(2,-59,2000,{.maxSpeed = 105},false); //3,-62
    chassis.turnToHeading(0,1500,{.maxSpeed = 80, .minSpeed = 20, .earlyExitRange = 1},false);

    //uncomment once raise is fixed
    chassis.moveFor(10, 750, {.forwards = false, .maxSpeed = 60}, true, true, 0);
    pros::delay(400);
    raiseLift();
    cata.edge();
    pros::delay(550);
    lowerLift();

    chassis.moveToPoint(27,-50,2000,{.minSpeed = 30, .earlyExitRange = 3},false);

    chassis.turnToPoint(40,-59,2000,{.minSpeed = 80, .earlyExitRange = 2},false);
    chassis.moveToPoint(40,-59,2000,{.maxSpeed = 90,.minSpeed = 50, .earlyExitRange = 5},false); //was .min 30

    chassis.moveToPoint(51,-65,2000,{.minSpeed = 85},false); //was .min 30
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
    pros::delay(250); // cant spin while intaking???
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
    chassis.moveToPoint(34,34,4000,{.minSpeed = 20, .earlyExitRange = 2, .slowDownRange = 40, .slowDownSpeed = 80},false);
    chassis.moveToPoint(60,-4.5,2000,{.forwards = false,.maxSpeed = 70},true);
    chassis.waitUntil(47);
    clamp.set_value(true);
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);
    chassis.waitUntilDone();
    pros::delay(150);

    //turn to da wall
    chassis.turnToHeading(180, 1000, {}, false);
    Pose reset1 = chassis.getPose();

    cata.edge(true);

    std::pair<float, float> reset1x = distReset.getDistance(DistanceReset::Wall::BOTTOM);
    chassis.setPose(reset1x.first, reset1x.second, reset1.theta);
    reset1 = chassis.getPose();
    std::cout << "reset1: " << reset1.x << " " << reset1.y << " " << reset1.theta << std::endl;

    chassis.turnToPoint(24,-24,2000,{.minSpeed = 70},false);
    
    chassis.moveToPoint(20,-28,2000,{.minSpeed = 80},false);
    pros::delay(200);
    chassis.turnToPoint(45,-51,2000,{.minSpeed = 60},false);
    chassis.moveToPoint(45,-51,2000,{.maxSpeed = 95},false);
    chassis.moveFor(10,2000,{.forwards = false},false);
    pros::delay(200);
    intake.set(Intake::IntakeState::STOPPED);
    cata.edge();
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(54,-52,1000,{.minSpeed = 80},false);
    chassis.moveToPoint(54, -52, 1000, {.minSpeed = 65}, false);
    
    pros::delay(300);
    
    
    // chassis.swingToPoint(70, -67, lemlib::DriveSide::RIGHT, 1000, {.forwards = false,.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .maxSpeed = 80, .minSpeed = 80}, true);
    chassis.moveFor(16, 750, {.forwards = false, .minSpeed = 50}, true);
    pros::delay(300);
    intake.set(Intake::IntakeState::OUTTAKE, 100);
    cata.edge(true);
    chassis.waitUntilDone();
    chassis.turnToHeading(-19, 2000, {.minSpeed = 80}, false);
    
    
    // chassis.waitUntil(70);
    chassis.waitUntilDone();
    clamp.set_value(false);
    pros::delay(100);
    // chassis.moveToPoint(72,-64.5,750,{.forwards = false,.minSpeed = 120},false);
    chassis.moveFor(10, 750, {.minSpeed = 50}, false);
    chassis.tank(-127,-127);
    pros::delay(1000);
    chassis.cancelAllMotions();
    
    chassis.setConstantState(lemlib::Chassis::ConstantState::DEFAULT);
    intake.set(Intake::IntakeState::INTAKING);
    //return;

    //WALLSTAKE 2
    chassis.moveToPoint(39, 0, 2000, {.maxSpeed = 105, .minSpeed = 50, .earlyExitRange = 7}, false);
    chassis.moveToPoint(24,40,4000,{.maxSpeed = 105, .minSpeed = 30,.earlyExitRange = 1},true); //was 30,50 //10
    intake.set(Intake::IntakeState::INTAKING);
    chassis.turnToPoint(-2, 56, 1000, {.minSpeed = 80}, false);
    chassis.moveToPoint(-2,56,2000,{},false);
    chassis.turnToHeading(-180,2000,{.minSpeed = 80},false);
    
    
    chassis.moveFor(15, 1000, {.forwards = false,.minSpeed = 30}, true);
    
    pros::delay(400);
    intake.set(Intake::IntakeState::OUTTAKE);
    raiseLift();
    chassis.waitUntilDone();
    Pose pose = chassis.getPose();
    chassis.setPose(pose.x, 69, pose.theta);
    cata.edge();
    pros::delay(550);
    lowerLift();

    
    chassis.moveToPoint(40, 51, 2000,{.minSpeed = 50},true);
    chassis.waitUntil(10);
    intake.set(Intake::IntakeState::INTAKING);
    // chassis.moveToPoint(65,50,2000,{.minSpeed = 70,.earlyExitRange = 2},false); //64,50
    chassis.waitUntilDone();

    chassis.moveToPoint(28,42,2000,{.forwards = false,.minSpeed = 40, .earlyExitRange = 5},false);
    intake.set(Intake::IntakeState::OUTTAKE); 
    chassis.moveToPoint(54,0,2000,{.forwards=false}, false);
    chassis.turnToHeading(-90,2000, {.minSpeed = 80},false);
    
    chassis.moveFor(13, 750, {.forwards = false}, false);
    cata.edge();

    // intake.set(Intake::IntakeState::STOPPED);
    chassis.moveFor(8,2000,{.minSpeed = 30},false);
    chassis.turnToPoint(65,60,2000,{.forwards = false, .minSpeed = 80,},false);
    chassis.moveToPoint(65,60,2000,{.forwards = false,.minSpeed = 120,.earlyExitRange = 2},false);
    chassis.turnToHeading(-80,2000,{.minSpeed = 80},false);


    // chassis.moveFor(6,2000,{.minSpeed = 50,.earlyExitRange = 3},false);
    // chassis.moveToPoint(0,65,2000,{.minSpeed = 60,.earlyExitRange = 15},false);
    chassis.moveToPoint(-25,60-8,2000,{.maxSpeed = 110, .minSpeed = 30, .earlyExitRange = 1, .slowDownRange = 50, .slowDownSpeed = 80},true);
    intake.set(Intake::IntakeState::INTAKING);
    chassis.waitUntil(24);
    cata.edge();
    chassis.turnToPoint(-30,38-8,2000,{.minSpeed = 80},false);
    chassis.moveToPoint(-34,38-8,2000,{.minSpeed = 30},false);
    chassis.turnToPoint(-67,38-6,2000,{.forwards = false,.minSpeed = 40},false);
    chassis.moveToPoint(-63,38-6,2000,{.forwards = false,.maxSpeed = 70},true);
    chassis.waitUntil(30);
    clamp.set_value(true);
    cata.edge();
    chassis.setConstantState(lemlib::Chassis::ConstantState::MOGO);

    chassis.moveToPoint(-50, 67, 2000, {.minSpeed = 80, .earlyExitRange = 2}, false);

    chassis.turnToHeading(130, 2000, {.minSpeed = 80, .earlyExitRange = 2}, false);
    pros::delay(200);
    
    chassis.moveFor(25, 750, {.forwards = false, .maxSpeed = 80}, true);
    cata.edge();
    pros::delay(200);

    clamp.set_value(false);
    chassis.waitUntilDone();
    chassis.moveFor(10, 1000,{.minSpeed = 127},false);



}