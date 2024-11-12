#include "subsystem/intake.hpp"

Intake::Intake(pros::Motor& motor): motor(motor) {};

void Intake::intakeControl(){
    while(true){
        if(this->state == STOPPED){
            this->motor.move(0);
        } else if(this->state == INTAKING){
            this->motor.move(127);
            pros::delay(250);
            if(fabs(this->motor.get_actual_velocity()) > 10){
                this->motor.move(127);
            } else {
                this->motor.move(-120);
                pros::delay(500);
            }
        } else if(this->state == OUTTAKE){
            this->motor.move(-127);
        }
        pros::delay(20);
    }
}

void Intake::set(IntakeState state){
    this->state = state;
}

