#include "subsystem/intake.hpp"
#include "pros/misc.hpp"

Intake::Intake(pros::Motor& motor): motor(motor) {};

void Intake::intakeControl(){
    while(true){
        if(this->state == STOPPED){
            this->motor.move(0);
        } else if(this->state == INTAKING){
            this->motor.move(this->speed);
            pros::delay(250);
            while(this->state == INTAKING){
            if(pros::competition::is_autonomous()){
                if(fabs(this->motor.get_actual_velocity()) > 10){
                    this->motor.move(this->speed);
                } else {
                    this->motor.move(-120);
                    pros::delay(500);
                }
            } else {
                if(fabs(this->motor.get_actual_velocity()) < 10){
                    pros::delay(400);
                    this->motor.move(45);
                }
            }
            pros::delay(20);
            }
        } else if(this->state == OUTTAKE){
            this->motor.move(-this->speed);
        }
        pros::delay(20);
    }
}

void Intake::set(IntakeState state, int speed){
    this->state = state;
    this->speed = speed;
}

