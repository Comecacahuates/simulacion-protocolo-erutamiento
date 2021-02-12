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

#include "veins_proj/locationservice/HostsLocationTable.h"
#include <utility>

using namespace veins_proj;


Define_Module(HostsLocationTable);



void HostsLocationTable::registerHostLocation(const inet::Ipv6Address &address, const GeohashLocation &geohashLocation, const LocationOnRoadNetwork &locationOnRoadNetwork) {
    hostsLocation[address] = { geohashLocation, locationOnRoadNetwork };
}

const std::vector<inet::Ipv6Address> HostsLocationTable::getAddresses() const {
    std::vector<inet::Ipv6Address> addresses;

    for (auto const &entry: hostsLocation)
        addresses.push_back(entry.first);

    return addresses;
}
