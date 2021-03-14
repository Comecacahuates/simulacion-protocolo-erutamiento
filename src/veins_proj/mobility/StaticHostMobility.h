//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#pragma once

#include <GeographicLib/GeoCoords.hpp>
#include <omnetpp.h>
#include "inet/mobility/base/MovingMobilityBase.h"
#include "inet/common/geometry/common/GeographicCoordinateSystem.h"
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/roadnetwork/RoadNetworkDatabase.h"

namespace veins_proj {

// TODO: Cambiar herencia de MovingMobilityBase.
class StaticHostMobility: public inet::MovingMobilityBase {

protected:
    // Context
    omnetpp::cModule *host;
    RoadNetworkDatabase *roadNetworkDatabase = nullptr;

    // Internal
    GeohashLocation geohashLocation;
    RoadNetwork *roadNetwork = nullptr;

public:
    StaticHostMobility();

    const RoadNetwork *getRoadNetwork() const {
        return roadNetwork;
    }

    virtual void initialize(int stage) override;

    const GeohashLocation& getGeohashLocation() const {
        return geohashLocation;
    }

    virtual void move() override {
    }
};

}    // namespace veins_proj
