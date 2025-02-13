#pragma once
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include <cmath>

class Intake {
    public:

        Intake(pros::Motor& motor);

        enum IntakeState{ STOPPED , INTAKING , OUTTAKE };

        void intakeControl();

        void set(IntakeState state, int speed = 127);

        pros::Motor& motor;
        IntakeState state = STOPPED;
        int speed = 127;
};
