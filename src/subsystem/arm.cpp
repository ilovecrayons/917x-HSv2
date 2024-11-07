#include "subsystem/arm.hpp"

Arm::Arm(pros::MotorGroup* motors, pros::Rotation* rotation, float kP, float kI, float kD, float exitRange,
         float exitTime)
    : motors(motors),
      rotation(rotation),
      pid(kP, kI, kD),
      exitCondition(exitRange, exitTime) {
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    rotation->reset();
};

void Arm::setPower(float power) { motors->move(power); }

void Arm::reset() {
    rotation->reset();
    motors->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    motors->move(0);
}

void Arm::moveTo(float position, bool async) {
    if (async) {
        pros::Task task {[=, this] { moveTo(position, false); }};
    } else {
        pid.reset();
        exitCondition.reset();
        while (exitCondition.getExit() == false) {
            float error = pid.update(position - rotation->get_position());
            motors->move(error);
            exitCondition.update(error);
            pros::delay(10);
        }
    }
    motors->move(0);
}
