#include <cmath>
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/logger/logger.hpp"
#include "lemlib/timer.hpp"
#include "lemlib/util.hpp"
#include "pros/misc.hpp"

void lemlib::Chassis::moveFor(float distance, int timeout, MoveForParams params, bool async) {
    params.earlyExitRange = fabs(params.earlyExitRange);
    this->requestMotionStart();
    // were all motions cancelled?
    if (!this->motionRunning) return;
    // if the function is async, run it in a new task
    if (async) {
        pros::Task task([&]() { moveFor(distance, timeout, params, false); });
        this->endMotion();
        pros::delay(10); // delay to give the task time to start
        return;
    }

    // reset PIDs and exit conditions
    lateralPID.reset();
    lateralLargeExit.reset();
    lateralSmallExit.reset();
    angularPID.reset();

    // initialize vars used between iterations
    Pose lastPose = getPose();
    distTraveled = 0;
    Timer timer(timeout);
    bool close = false;
    float prevLateralOut = 0; // previous lateral power
    float prevAngularOut = 0; // previous angular power
    const int compState = pros::competition::get_status();
    std::optional<bool> prevSide = std::nullopt;

    // main loop
    while (!timer.isDone() && !lateralLargeExit.getExit() && !lateralSmallExit.getExit() && this->motionRunning) {
        // update variables
        const Pose pose = getPose(true, true);

        // update distance traveled
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        // calculate distnace to the target
        const float distTarget = distance - distTraveled;
        // check if the robot is close enough to the target to start settling
        if(distTarget < 7.5 && !close){
            close = true;
            params.maxSpeed = fmax(fabs(prevLateralOut), 60); // slows down robot

        }

        // motion chaining
        const float lateralMovement = pose.x - lastPose.x;
        const bool side = lateralMovement > 0;
        if (prevSide == std::nullopt) prevSide = side;
        const bool sameSide = side == prevSide;
        // exit if close
        if (!sameSide && params.minSpeed != 0) break;
        prevSide = side;

        // calculate error
        const float adjustedRobotTheta = params.forwards ? pose.theta : pose.theta + M_PI;
        const float angularError = angleError(
        )

}