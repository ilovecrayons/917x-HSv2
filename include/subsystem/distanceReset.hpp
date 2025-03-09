#pragma once
#include "pros/distance.hpp"

class DistanceReset {
    public:
        /**
         * @brief distanceReset constructor
         *
         * @param dist the distance sensor
         */
        DistanceReset(pros::Distance* dist, int offset = 0);

        enum class Wall { TOP, BOTTOM, LEFT, RIGHT };

        /**
         * @brief get the reading of the distance sensor
         *
         * @param wall the wall to reset the distance sensor to
         */
        int getDistance(Wall wall);
    protected:
        /**
         * @brief convert millimeters to inches
         *
         * @param mm the distance in millimeters
         * @return int the distance in inches
         */
        int toInches(int mm);
    private:
        int offset;
        pros::Distance* dist;
};