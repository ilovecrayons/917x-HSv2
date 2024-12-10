#pragma once
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"

class autonSeparation {
    public:
        autonSeparation(pros::Motor* intakeMotor,pros::MotorGroup* intake, pros::Optical* topSort);
        enum Ring { BLUE, RED };
        void removeRings(Ring ring);
        void stop();
    private:
        pros::MotorGroup* intake;
        pros::Optical* topSort;
        pros::Motor* intakeMotor;
        double INITIAL_POSITION = 0;
        double SEPARATION_MOVEMENT = 45;  //Degrees
        bool sort = false;
};