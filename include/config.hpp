#pragma once
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "subsystem/arm.hpp"
#include "subsystem/intake.hpp"

extern pros::Controller controller;
extern Intake intake;
extern pros::adi::DigitalOut clamp;
extern pros::adi::DigitalOut dGate;
extern pros::Optical topSort;
extern pros::Optical bottomSort;
extern pros::Imu imu;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup leftMotors;
extern pros::Distance distance;
extern lemlib::Chassis chassis;
extern Arm arm;

