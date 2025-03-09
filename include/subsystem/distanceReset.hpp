#pragma once
#include "pros/distance.hpp"
#include <utility>

class DistanceReset {
    public:
        /**
         * @brief distanceReset constructor
         *
         * @param dist the distance sensor
         */
        DistanceReset(pros::Distance* horiDist, pros::Distance* vertiDist, float horiOffset = 0, float vertiOffset = 0) {
            this->horiDist = horiDist;
            this->vertiDist = vertiDist;
            this->horiOffset = horiOffset;
            this->vertiOffset = vertiOffset;
        }

        enum class Wall { TOP, BOTTOM, LEFT, RIGHT };

        /**
         * @brief get the reading of the distance sensor
         *
         * @param wall the wall to reset the distance sensor to
         */
        std::pair<float, float> getDistance(Wall wall);
    protected:
        /**
         * @brief convert millimeters to inches
         *
         * @param mm the distance in millimeters
         * @return int the distance in inches
         */
        float toInches(int mm) { return mm / 25.4; }
    private:
        float horiOffset;
        float vertiOffset;
        pros::Distance* horiDist;
        pros::Distance* vertiDist;
};