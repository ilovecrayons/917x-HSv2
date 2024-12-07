#pragma once
#include "pros/motor_group.hpp"

class autonSeparation {
    public:
        autonSeparation(pros::MotorGroup* intake);
        enum Ring { BLUE, RED };
        void removeRings(Ring ring);
        void stop();
    private:
        pros::MotorGroup* intake;
};