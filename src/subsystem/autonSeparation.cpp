#include "subsystem/autonSeparation.hpp"

autonSeparation::autonSeparation(pros::Motor* intakeMotor,pros::MotorGroup* intake, pros::Optical* topSort):
    intake(intake),
    topSort(topSort),
    intakeMotor(intakeMotor)
    {};