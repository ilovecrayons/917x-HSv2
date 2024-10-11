#include "pid.hpp"
#include "util.hpp"

namespace lemlib {
PID::PID(float kP, float kI, float kD, float windupRange, bool signFlipReset)
    : kP(kP),
      kI(kI),
      kD(kD),
      windupRange(windupRange),
      signFlipReset(signFlipReset) {}

float PID::update(const float error) {
    // calculate integral
    integral += error;
    if (sgn(error) != sgn((prevError)) && signFlipReset) integral = 0;
    if (fabs(error) > windupRange && windupRange != 0) integral = 0;

    // calculate derivative
    const float derivative = error - prevError;
    prevError = error;

    // calculate output
    return error * kP + integral * kI + derivative * kD;
}

void PID::reset() {
    integral = 0;
    prevError = 0;
}


void PID::setGains(float kP, float kI, float kD){
    this->kP = kP;
    this->kI = kI;
    this->kD = kD;

    reset();
}
void PID::constantChanger(pros::Controller& controller) {
    while (true){
    controller.clear();
    controller.print(0,0,"x: +kP, b: -kP | up: +kI(0.5), down: -kI(0.5) | L1: +kD, L2: -kD");
    controller.print(2, 0, "Current Gains: %f,%f,%f", getGains()[0], getGains()[1], getGains()[2]);
    std::ofstream myfile; // Change to use the fully qualified name std::ofstream
    if(runNum == 1){
        myfile.open("LateralPIDVals.txt", std::ios::app);;
    }
    float kP = getGains()[0];
    float kI = getGains()[1];
    float kD = getGains()[2];

    myfile << std::to_string(runNum)+": "+std::to_string(getGains()[0])+","+std::to_string(getGains()[1])+","+std::to_string(getGains()[2])+"\n";
    
    while (true){
    
        if (controller.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_X)){
            setGains(kP++,kI,kD);
            break;
        }
        if (controller.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_B)){
            setGains(kP--,kI,kD);
            break;
        }
        if (controller.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_UP)){
            setGains(kP,kI+0.5,kD);
            break;
        }
        if (controller.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_DOWN)){
            setGains(kP,kI-0.5,kD);
            break;
        }
        if (controller.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L1)){
            setGains(kP,kI,kD++);
            break;
        }
        if (controller.get_digital(pros::controller_digital_e_t::E_CONTROLLER_DIGITAL_L2)){
            setGains(kP,kI,kD--);
            break;
        }
        else{ continue; }
        runNum++;
    }
    }
}

std::vector<float> PID::getGains(){
    return {kP,kI,kD};
}

} // namespace lemlib