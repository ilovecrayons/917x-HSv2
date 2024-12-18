#include "subsystem/intake.hpp"
#include "pros/misc.hpp"

Intake::Intake(pros::Motor& motor, pros::Optical& topSort): motor(motor) , topSort(topSort){};

void Intake::setSeparation(Ring ring){
    this->ring = ring;
}

void Intake::checkForSort(){
    if (ring == Ring::NONE){
       sort = false; 
    }
    else if (ring == Ring::BLUE){
        if (topSort.get_hue()>200 && topSort.get_hue()<=270){
            sort = true;
            INITIAL_POSITION = motor.get_position();
        }
    }
    else if (ring == Ring::RED){
        if (topSort.get_hue()<30 && topSort.get_hue()>=0) {
            sort = true;
            INITIAL_POSITION = motor.get_position();
        }
    }
}

void Intake::intakeControl(){
    while(true){
        checkForSort();
        bool COMPLETED_MOVEMENT = (INITIAL_POSITION+SEPARATION_MOVEMENT) <= motor.get_position();

        bool COMPLETED_SEPARATION_WAIT = SEPARATION_WAIT >= TIME_TO_COMPLETE_SEP;

        if (sort && !COMPLETED_MOVEMENT) {
            state = IntakeState::INTAKING;
        }
        else if (sort && COMPLETED_MOVEMENT && !COMPLETED_SEPARATION_WAIT){
            state = IntakeState::STOPPED;
            SEPARATION_WAIT++;
        }
        else if (sort && COMPLETED_MOVEMENT && COMPLETED_SEPARATION_WAIT) {
            state = IntakeState::STOPPED;
            sort = !sort;
            SEPARATION_WAIT = 0;
        }


        if(this->state == STOPPED){
            this->motor.move(0);
        } else if(this->state == INTAKING){
            this->motor.move(this->speed);
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

