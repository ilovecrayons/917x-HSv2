#pragma once
#include "lemlib/api.hpp"
#include "pros/rotation.hpp"
#include "subsystem/distanceReset.hpp"
#include "subsystem/intake.hpp"
#include "subsystem/cata.hpp"

// ports
constexpr int RIGHT_F = 18;
constexpr int RIGHT_M = 19;
constexpr int RIGHT_B = 17;

constexpr int LEFT_F = -16;
constexpr int LEFT_M = -19;
constexpr int LEFT_B = -14;

constexpr int VERTI_ROT = 1;
constexpr int HORI_ROT = 6;

constexpr int INTAKE = 3;

constexpr int CATA = -2;
constexpr int CATA_ROT = 7;

constexpr char CLAMP = 'A';
constexpr char HOOK = 'B';
constexpr char LIFT = 'C';

constexpr char IMU = 21;
constexpr int HORI_DISTANCE = 4;
constexpr int VERTI_DISTANCE = 8;

// controller
inline pros::Controller controller(pros::E_CONTROLLER_MASTER);

// drivetrain
inline pros::MotorGroup rightMotors({RIGHT_F, RIGHT_M, RIGHT_B}, pros::MotorGearset::blue);
inline pros::MotorGroup leftMotors({LEFT_F, LEFT_M, LEFT_B}, pros::MotorGearset::blue);

// cata
inline pros::Motor cataMotor(CATA, pros::MotorGearset::green);
inline pros::Rotation cataRot(CATA_ROT);
inline Cata cata(&cataMotor, &cataRot, 2.1, 0.1, 3.5);

// intake
inline pros::Motor intakeMotor(INTAKE, pros::MotorGearset::blue);
inline Intake intake(intakeMotor);

// Clamp mechanism Piston
inline pros::adi::DigitalOut clamp(CLAMP);

// lift piston
inline pros::adi::DigitalOut lift(LIFT);

// hook mechanism
inline pros::adi::DigitalOut hook(HOOK);

// Inertial Sensor
inline pros::Imu imu(IMU);

// distance sensor
inline pros::Distance horiDistance(HORI_DISTANCE);
inline pros::Distance vertiDistance(VERTI_DISTANCE);

inline pros::Rotation vertiRot(VERTI_ROT);
inline pros::Rotation horiRot(HORI_ROT);
inline lemlib::TrackingWheel vertiTrackingWheel(&vertiRot, lemlib::Omniwheel::NEW_2, 0);
inline lemlib::TrackingWheel horiTrackingWheel(&horiRot, lemlib::Omniwheel::NEW_2, -2.874);

// drivetrain settings
inline lemlib::Drivetrain drivetrain(&leftMotors, // left motor group
                                     &rightMotors, // right motor group
                                     12, // 14 inch track width
                                     lemlib::Omniwheel::NEW_325,
                                     450, // drivetrain rpm is 450
                                     8 // If you have a drift drive, we recommend starting with a value of 2, while a
                                       // drivetrain with center traction wheels should start with a value of 8.
);

// lateral motion controller
inline lemlib::ControllerSettings linearController(8, // 12      proportional gain (kP)
                                                   0.03, //       integral gain (kI)
                                                   90, // 65     derivative gain (kD)
                                                   3, //       anti windup
                                                   1, // 1     small error range, in inches
                                                   100, // 100    small error range timeout, in milliseconds
                                                   3, // 3      large error range, in inches
                                                   300, // 500  large error range timeout, in milliseconds
                                                   10 // 20     maximum acceleration (slew)
);

// angular motion controller
inline lemlib::ControllerSettings angularController(5.78, // 4.1  proportional gain (kP)
                                                    0.1, // 0     integral gain (kI)
                                                    62, // 38   derivative gain (kD)
                                                    5, //       anti windup
                                                    1, // 1     small error range, in degrees
                                                    100, // 100  small error range timeout, in milliseconds
                                                    3, // 3      large error range, in degrees
                                                    500, //  large error range timeout, in milliseconds
                                                    0 //        maximum acceleration (slew)
);

// sensors for odometry
inline lemlib::OdomSensors sensors(&vertiTrackingWheel, // vertical tracking wheel
                                   nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                                   &horiTrackingWheel, // horizontal tracking wheel
                                   nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                                   &imu // inertial sensor
);

// input curve for throttle input during driver control
inline lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                            10, // minimum output where drivetrain will move out of 127
                                            1.019 // expo curve gain
);

// input curve for steer input during driver control
inline lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                         10, // minimum output where drivetrain will move out of 127
                                         1.019 // expo curve gain
);

// create the chassis
inline lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

inline DistanceReset distReset(&horiDistance, &vertiDistance, 5.55, 6.4);

inline void raiseLift() {
    // pros::Task raise {[=] {
    //     lift.set_value(true);
    //     pros::delay(500);
    //     cata.score(37, false, 60);
    // }};
    lift.set_value(true);
    pros::delay(200);
    cata.score(35, true, 60);
    pros::delay(400);
}

inline void lowerLift(){
  lift.set_value(false);
  cata.load(1, true);
}