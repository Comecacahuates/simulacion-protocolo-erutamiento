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

#include "veins_proj/networklayer/configurator/AddressCache.h"

using namespace veins_proj;


Define_Module(AddressCache);


AddressCache::AddressCache() {
    unicastAddresses[0] = inet::Ipv6Address::UNSPECIFIED_ADDRESS;
    unicastAddresses[1] = inet::Ipv6Address::UNSPECIFIED_ADDRESS;
    multicastAddresses[0] = inet::Ipv6Address::UNSPECIFIED_ADDRESS;
    multicastAddresses[1] = inet::Ipv6Address::UNSPECIFIED_ADDRESS;
}


void AddressCache::swapAddresses() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    EV_INFO << "AddressCache::swapNetworks" << std::endl;

    std::swap(unicastAddresses[PRIMARY_ADDRESS], unicastAddresses[SECONDARY_ADDRESS]);
    std::swap(multicastAddresses[PRIMARY_ADDRESS], multicastAddresses[SECONDARY_ADDRESS]);
}


void AddressCache::clear(const int &addressType) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    EV_INFO << "AddressCache::clear" << std::endl;

    unicastAddresses[addressType] = inet::Ipv6Address::UNSPECIFIED_ADDRESS;
    multicastAddresses[addressType] = inet::Ipv6Address::UNSPECIFIED_ADDRESS;
}
