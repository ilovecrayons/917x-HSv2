#include "subsystem/distanceReset.hpp"
#include <utility>

std::pair<float, float> DistanceReset::getDistance(Wall wall) {
    std::vector<int> horiDistances;
    std::vector<int> vertiDistances;
    for(int i = 0; i < 15; i++){
        horiDistances.push_back(horiDist->get_distance());
        vertiDistances.push_back(vertiDist->get_distance());
        pros::delay(40);
    }
    float horiMedian = horiDistances[horiDistances.size()/2];
    float vertiMedian = vertiDistances[vertiDistances.size()/2];

    switch (wall) {
        case Wall::RIGHT:
            break;
        case Wall::BOTTOM:
            return std::make_pair(71 - toInches(horiDist->get_distance()) - horiOffset, -71 + toInches(vertiMedian) + vertiOffset);
        case Wall::LEFT:
            break;
        case Wall::TOP:
            break;
    }
    return std::make_pair(0,0);
}
