#include "subsystem/distanceReset.hpp"

DistanceReset::DistanceReset(pros::Distance* dist, int offset) {
    this->dist = dist;
    this->offset = offset;
}

int DistanceReset::getDistance(Wall wall) {
    switch (wall) {
        case Wall::RIGHT:
            return 71 - toInches(dist->get_distance()) - offset;
        case Wall::BOTTOM:
            return -71 + toInches(dist->get_distance()) + offset;
        case Wall::LEFT:
            break;
        case Wall::TOP:
            break;
    }
    return 0;
}
