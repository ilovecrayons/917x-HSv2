#pragma once
#include "pros/colors.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
void intakeControl(pros::Motor& stage1Motor, pros::Motor& stage2Motor, pros::Optical& opticalSensor);