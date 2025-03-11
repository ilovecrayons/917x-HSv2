#include "subsystem/cata.hpp"
#include "lemlib/util.hpp"
#include "pros/motors.h"
#include "pros/screen.hpp"
#include "lemlib/timer.hpp"

/**
 * @brief Construct a new Cata object
 *
 * @param motor a pointer to a Motor object for the catapult
 * @param rotation a pointer to a Rotation object for the catapult
 * @param kP proportional gain for the PID
 * @param kI integral gain for the PID
 * @param kD derivative gain for the PID
 * @param exitRange range of the exit condition for the PID
 * @param exitTime time for the exit condition for the PID
 *
 * @note The motor's brake mode is set to HOLD when a Cata object is constructed
 */
Cata::Cata(pros::Motor* motor, pros::Rotation* rotation, float kP, float kI, float kD, float exitRange, float exitTime)
    : motor(motor),
      rotation(rotation),
      pid(kP, kI, kD, 15, true),
      exitCondition(exitRange, exitTime) {
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

void Cata::setPower(float power) { motor->move(power); }

/**
 * @brief Initializes the Cata object
 *
 * This function is meant to be called in initialize() and is used to
 * reset the rotation sensor and to set the zero position of the motor.
 * It will subtract 360 from the rotation sensor's position if it is
 * greater than 250, and then set the motor's position to the new
 * rotation sensor position.
 */
void Cata::initialize() {
    if ((rotation->get_position() / 100) > 250) {
        rotation->set_position((rotation->get_position() / 100 - 360) * 100);
    }
    pros::Task task {[=, this] {movementClamp();}};

}

/**
 * @brief Moves the catapult to the specified position.
 *
 * This function moves the catapult to a given position using PID control.
 * The movement can be executed asynchronously or synchronously based on the
 * async parameter. If run asynchronously, the function returns immediately
 * and the movement continues in a separate task.
 *
 * @param position The target position in encoder units.
 * @param async If true, the movement runs asynchronously; otherwise, it is blocking.
 * @param timeout The maximum time in milliseconds to wait for the movement to complete.
 * @param slewrate The maximum change in motor output per cycle to prevent sudden changes.
 */

void Cata::moveTo(int position, bool async, int timeout, float maxSpeed) {
    if (async) {
        pros::Task task {[=, this] { moveTo(position, false, timeout, maxSpeed); }};
    } else {
        // reset controllers
        pid.reset();
        exitCondition.reset();
        lemlib::Timer timer(timeout);
        // main loop
        while (exitCondition.getExit() == false && !timer.isDone()) {
            int error = position - this->getPosition(); // get the error between target and current in DEGREES
            float motorOutput =
                std::clamp(pid.update(error), -maxSpeed, maxSpeed); // calculate PID output constrained by max speed
            motor->move(motorOutput); // move the motor according to PID output
            exitCondition.update(error); // update the exit condition

            pros::delay(10); // delay to save resources
            pros::screen::print(pros::E_TEXT_MEDIUM, 8, "error: %d", error); // prints error to screen for debugging
        }
        motor->brake(); // stops motor
    }
}

/**
 * @brief Loads the catapult to the specified position.
 *
 * This function moves the catapult to a given position with the motor brake mode set to coast.
 * The movement can be executed asynchronously or synchronously based on the async parameter.
 *
 * @param position The target position in encoder units.
 * @param async If true, the movement runs asynchronously; otherwise, it is blocking.
 */

void Cata::load(float position, bool async) {
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

    if (async) {
        moveTo(position, true);
    } else {
        moveTo(position, false);
    }
}

/**
 * @brief Scores the ring
 *
 * This function moves the catapult to a given position, scoring the disc.
 * The movement can be executed asynchronously or synchronously based on the
 * async parameter.
 *
 * @param position The target position in encoder units
 * @param async If true, the movement runs asynchronously; otherwise, it is blocking
 * @param slewrate The maximum change in motor output per cycle to prevent sudden changes
 */
void Cata::score(float position, bool async, float maxSpeed) {
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    if (async) {
        moveTo(position, true, 750, maxSpeed);
    } else {
        moveTo(position, false, 750, maxSpeed);
    }
}

void Cata::edge(bool async) {
    if (async) {
        pros::Task goon {[=, this] { edge(false); }};
    } else {
        this->score();
        this->load(1, true);
    }
}

void Cata::toggle() {
    toggleState = !toggleState;
    if (toggleState) {
        this->score();
    } else {
        this->load();
    }
}

void Cata::movementClamp(){
    if (rotation->get_position()>=235){
        motor->move(0);
    }
}

void Cata::pidLessMovement(){
    motor->set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    motor->move(127);
}

void Cata::brake() { motor->brake(); }

int Cata::getPosition() { return rotation->get_position() / 100; }