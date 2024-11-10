#pragma once
#include "lemlib/pid.hpp"
#include "lemlib/exitcondition.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

class Arm{
    public:
        Arm(pros::MotorGroup* motors, pros::Rotation* rotation, float kP, float kI, float kD, float exitRange = 10, float exitTime =150);

        void setPower(float power);
        void reset();
        void moveTo(float position, bool async = false);
        void loadWallstake(float position = 1200, bool async = false);
        void scoreWallstake(float position = 0, bool async = false);
    private:
        pros::MotorGroup* motors;
        pros::Rotation* rotation;
        lemlib::PID pid;
        lemlib::ExitCondition exitCondition;
};