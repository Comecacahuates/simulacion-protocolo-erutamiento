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

#include <omnetpp.h>
#include <string>
#include <map>
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "veins_proj/geohash/GeohashLocation.h"

namespace veins_proj {

class RoadNetworkDatabase: public omnetpp::cSimpleModule {

private:
    /*
     * Parámetros.
     */
    std::string databaseDirectory;

    std::map<std::string, RoadNetwork*> roadNetworksMap;

public:
    virtual ~RoadNetworkDatabase();

protected:
    virtual void initialize() override;
    virtual void handleMessage(omnetpp::cMessage *message) override {
    }

public:
    RoadNetwork* getRoadNetwork(const GeohashLocation &geohashLocation);
};

}    // namespace veins_proj
