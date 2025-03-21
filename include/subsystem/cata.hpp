#pragma once
#include "lemlib/pid.hpp"
#include "lemlib/exitcondition.hpp"
#include "pros/motors.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

class Cata {
    public:
        Cata(pros::Motor* motor, pros::Rotation* rotation, float kP, float kI, float kD, float exitRange = 2,
            float exitTime = 150);

        void setPower(float power);

        void initialize();

        void moveTo(int position, bool async = false, int timeout = 1000, float maxSpeed = 127);

        void load(float position = 1, bool async = false);

        void score(float position = 235, bool async = false, float maxSpeed = 127); 

        void edge(bool async = false);

        void toggle();

        int getPosition();

        void brake();
        
        
        void scoreOverride();




    private:
        bool toggleState = false; 
        pros::Rotation* rotation;
        pros::Motor* motor;
        lemlib::PID pid;
        lemlib::ExitCondition exitCondition;
        int state = 0;
};