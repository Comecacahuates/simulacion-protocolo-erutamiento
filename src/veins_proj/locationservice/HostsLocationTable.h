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
#include <map>
#include <vector>
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"

namespace veins_proj {

class HostsLocationTable: public omnetpp::cSimpleModule {

public:
    GeohashLocation geohashLocation;

protected:
    typedef std::pair<inet::Ipv6Address, GeohashLocation> HostLocation;
    typedef std::map<inet::Ipv6Address, GeohashLocation> HostsLocationMap;
    typedef HostsLocationMap::iterator HostsLocationIterator;
    typedef HostsLocationMap::const_iterator HostsLocationConstIterator;
    HostsLocationMap hostsLocation;

protected:
    virtual void initialize() override {
    }
    virtual void handleMessage(omnetpp::cMessage *message) override {
    }

public:
    void registerHostLocation(const inet::Ipv6Address &address,
            const GeohashLocation &geohashLocation) {
        hostsLocation[address] = geohashLocation;
    }
    const GeohashLocation& getHostLocation(const inet::Ipv6Address &address) {
        return hostsLocation[address];
    }
};

}    // namespace veins_proj
