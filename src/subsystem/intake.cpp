#include "subsystem/intake.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

Intake::Intake(pros::Motor& motor): motor(motor){};

void Intake::intakeControl(){
    this->motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    while(true){
        if(this->state == STOPPED){
            this->motor.brake();
        } else if(this->state == INTAKING){
            this->motor.move(this->speed);
        } else if(this->state == OUTTAKE){
            this->motor.move(-this->speed);
        }
        pros::delay(10);
    }
}

void Intake::set(IntakeState state, int speed){
    this->state = state;
    this->speed = speed;
}

