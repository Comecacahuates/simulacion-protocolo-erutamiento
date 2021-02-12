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


class HostsLocationTable : public omnetpp::cSimpleModule {

public:
    struct HostLocationEntry {
        GeohashLocation geohashLocation;
        LocationOnRoadNetwork locationOnRoadNetwork;
    };

protected:
    typedef std::pair<inet::Ipv6Address, HostLocationEntry> HostLocation;
    typedef std::map<inet::Ipv6Address, HostLocationEntry> HostsLocationMap;
    typedef HostsLocationMap::iterator HostsLocationIterator;
    typedef HostsLocationMap::const_iterator HostsLocationConstIterator;
    HostsLocationMap hostsLocation;

protected:
    virtual void initialize() {}
    virtual void handleMessage(omnetpp::cMessage *message) override {}

public:
    void registerHostLocation(const inet::Ipv6Address &address, const GeohashLocation &geohashLocation, const LocationOnRoadNetwork &locationOnRoadNetwork);
    const HostLocationEntry &getHostLocation(const inet::Ipv6Address &address) { return hostsLocation[address]; }
    const std::vector<inet::Ipv6Address> getAddresses() const;
    int getNumHosts() const { return hostsLocation.size(); }
    bool hasAddress(const inet::Ipv6Address &address) const { return hostsLocation.find(address) != hostsLocation.end(); }
};


} // namespace veins_proj
