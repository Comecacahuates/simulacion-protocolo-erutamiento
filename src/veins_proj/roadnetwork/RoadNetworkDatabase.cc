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

#include <vector>
#include "veins_proj/roadnetwork/RoadNetworkDatabase.h"

using namespace veins_proj;


Define_Module(RoadNetworkDatabase);


RoadNetworkDatabase::~RoadNetworkDatabase() {
    for (auto it = roadNetworksMap.begin(); it != roadNetworksMap.end(); it++)
        delete it->second;
}


void RoadNetworkDatabase::initialize() {
    std::vector<std::string> keys = { "9g3qxk", "9g3qxs" };
    std::string directory = DATABASE_DIRECTORY;

    RoadNetwork *roadNetwork;

    for (auto it = std::begin(keys); it != std::end(keys); it++) {
        roadNetwork = new RoadNetwork(*it, directory + *it + ".xml");
        roadNetworksMap.insert(std::pair<std::string, RoadNetwork *>(*it, roadNetwork));
    }
}


RoadNetwork *RoadNetworkDatabase::getRoadNetwork(const GeohashLocation &geohashLocation) {
    if (geohashLocation.getPrecision() < 6)
        return nullptr;

    std::string geohashString = geohashLocation.getGeohashString().substr(0, 6);
    if (roadNetworksMap.find(geohashString) != roadNetworksMap.end())
        return roadNetworksMap[geohashString];

    return nullptr;
}
