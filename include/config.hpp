#pragma once
#include "main.h"
#include "lemlib/api.hpp"
#include "pros/motor_group.hpp"

extern pros::Controller controller;
extern pros::MotorGroup intake;
extern pros::adi::DigitalOut clamp;
extern pros::adi::DigitalOut dGate;
extern pros::Optical topSort;
extern pros::Optical bottomSort;
extern pros::Imu imu;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup leftMotors;

extern lemlib::Chassis chassis;

