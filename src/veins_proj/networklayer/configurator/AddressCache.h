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

#include "veins_proj/geohash/GeohashLocation.h"
#include <omnetpp.h>
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"

#define PRIMARY_ADDRESS   0
#define SECONDARY_ADDRESS 1

namespace veins_proj {


class AddressCache : public cSimpleModule {

private:
    inet::Ipv6Address unicastAddresses[2];
    inet::Ipv6Address multicastAddresses[2];

public:
    AddressCache();

    const inet::Ipv6Address &getUnicastAddress(const int &addressType) const { return unicastAddresses[addressType]; }
    const inet::Ipv6Address &getMulticastAddress(const int &addressType) const { return multicastAddresses[addressType]; }
    void setUnicastAddress(const inet::Ipv6Address &unicastAddress, const int &addressType) { unicastAddresses[addressType] = unicastAddress; }
    void setMulticastAddress(const inet::Ipv6Address &multicastAddress, const int &addressType) { multicastAddresses[addressType] = multicastAddress; }
    void swapAddresses();
    void clear(const int &addressType);
};


}
