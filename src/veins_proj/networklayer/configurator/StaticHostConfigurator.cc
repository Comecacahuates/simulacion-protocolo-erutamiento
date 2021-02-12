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

#include "veins_proj/networklayer/configurator/StaticHostConfigurator.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/geometry/common/Coord.h"
#include "veins/base/utils/Coord.h"
#include <utility>
#include "veins_proj/networklayer/ipv6/Ipv6GeohashAddress.h"
#include "veins_proj/roadnetwork/RoadNetwork.h"

using namespace veins_proj;


Define_Module(StaticHostConfigurator);


void StaticHostConfigurator::initialize(int stage) {
    OperationalBase::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        // Parameters
        interface = par("interface").stdstringValue();

        // Context
        host = inet::getContainingNode(this);

        interfaceTable = inet::L3AddressResolver().interfaceTableOf(host);

        if (!interfaceTable)
            throw omnetpp::cRuntimeError("No interface table found");

        mobility = omnetpp::check_and_cast<StaticHostMobility *>(host->getSubmodule("mobility"));

        if (!mobility)
            throw omnetpp::cRuntimeError("No mobility module found");

        addressCache = omnetpp::check_and_cast<AddressCache *>(host->getSubmodule("addressCache"));

        if (!addressCache)
            throw omnetpp::cRuntimeError("No address cache module found");

        hostsLocationTable = omnetpp::check_and_cast<HostsLocationTable *>(getModuleByPath(par("hostsLocationTableModule")));

        if (!hostsLocationTable)
            throw omnetpp::cRuntimeError("No hosts location table module found");

    } else if (stage == inet::INITSTAGE_NETWORK_INTERFACE_CONFIGURATION) {
        networkInterface = interfaceTable->findInterfaceByName(interface.c_str());

        if (!networkInterface)
            throw omnetpp::cRuntimeError("No such interface '%s'", interface.c_str());

        if (networkInterface->isLoopback())
            throw omnetpp::cRuntimeError("Interface %s is loopback", interface.c_str());
    }
}


void StaticHostConfigurator::handleStartOperation(inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    EV_INFO << "StaticHostConfigurator::handleStartOperation" << std::endl;

    initInterface();

    EV_INFO << "Geohash: " << mobility->getGeohashLocation().getGeohashString() << std::endl;

    // El vehículo se une a la subred de la región donde se encuentra
    joinNetwork(mobility->getGeohashLocation());
}


void StaticHostConfigurator::joinNetwork(const GeohashLocation &geohashRegion) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    EV_INFO << "StaticHostConfigurator::joinNetwork" << std::endl;
    EV_INFO << "Network geohash: " << geohashRegions[PRIMARY_ADDRESS].getGeohashString() << std::endl;
    EV_INFO << "New network geohash: " << geohashRegion.getGeohashString() << std::endl;

    // Si la región geohash es igual a la nueva región geohash, no se hace nada
    if (geohashRegions[PRIMARY_ADDRESS] == geohashRegion)
        return;

    inet::Ipv6InterfaceData* ipv6Data = networkInterface->findProtocolDataForUpdate<inet::Ipv6InterfaceData>();

    inet::Ipv6Address unicastAddress = Ipv6GeohashAddress::ipv6UnicastAddress(geohashRegion, networkInterface->getInterfaceToken());
    EV_INFO << "Dirección unicast: " << unicastAddress.str() << std::endl;
    addressCache->setUnicastAddress(unicastAddress, PRIMARY_ADDRESS);
    ipv6Data->assignAddress(unicastAddress, false, SIMTIME_ZERO, SIMTIME_ZERO);

    // Se remplaza la dirección multicast
    inet::Ipv6Address multicastAddress = Ipv6GeohashAddress::ipv6MulticastAddress(geohashRegion);
    EV_INFO << "Dirección mulicast: " << multicastAddress.str() << std::endl;
    addressCache->setMulticastAddress(multicastAddress, PRIMARY_ADDRESS);
    ipv6Data->joinMulticastGroup(multicastAddress);
    ipv6Data->assignAddress(multicastAddress, false, SIMTIME_ZERO, SIMTIME_ZERO);

    geohashRegions[PRIMARY_ADDRESS].setGeohash(geohashRegion.getGeohashString());

    hostsLocationTable->registerHostLocation(unicastAddress, mobility->getGeohashLocation(), mobility->getLocationOnRoadNetwork());

    EV_INFO << "Registers in hosts location table module (" << unicastAddress << ", " << mobility->getGeohashLocation().getGeohashString() << ")" << std::endl;
    EV_INFO << "Number of hosts: " << hostsLocationTable->getNumHosts() << std::endl;

    EV_INFO << "Unicast address: " << unicastAddress.str() << std::endl;
    EV_INFO << "Multicast address: " << multicastAddress.str() << std::endl;

    showAddresses();
}


void StaticHostConfigurator::leaveNetwork() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    EV_INFO << "StaticHostConfigurator::leaveNetwork" << std::endl;

    // Si la región geohash primaria es nula, no se hace nada
    if (geohashRegions[PRIMARY_ADDRESS].isNull())
        return;

    inet::Ipv6InterfaceData *ipv6Data = networkInterface->findProtocolDataForUpdate<inet::Ipv6InterfaceData>();

    const inet::Ipv6Address &unicastAddress = addressCache->getUnicastAddress(PRIMARY_ADDRESS);

    // Se elimina la dirección unicast
    if (ipv6Data->hasAddress(unicastAddress)) {
        ipv6Data->removeAddress(unicastAddress);
    };

    const inet::Ipv6Address &multicastAddress = addressCache->getMulticastAddress(PRIMARY_ADDRESS);

    // Se elimina la dirección multicast
    if (ipv6Data->isMemberOfMulticastGroup(multicastAddress)) {
        ipv6Data->leaveMulticastGroup(multicastAddress);
        ipv6Data->removeAddress(multicastAddress);
    }

    addressCache->clear(PRIMARY_ADDRESS);

    showAddresses();
}


void StaticHostConfigurator::initInterface() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    EV_INFO << "StaticHostConfigurator::initInterface" << std::endl;

    inet::Ipv6InterfaceData *ipv6Data = networkInterface->findProtocolDataForUpdate<inet::Ipv6InterfaceData>();
    ipv6Data->setAdvSendAdvertisements(false);
}


void StaticHostConfigurator::showAddresses() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    EV_INFO << "StaticHostConfigurator::showAddresses" << std::endl;

    const inet::Ipv6InterfaceData *ipv6Data = networkInterface->findProtocolData<inet::Ipv6InterfaceData>();

    EV_INFO << "Unicast addresses" << std::endl;

    for (int i = 0; i < ipv6Data->getNumAddresses(); i++)
        EV_INFO << ipv6Data->getAddress(i).str() << std::endl;

    EV_INFO << "Multicast addresses" << std::endl;

    for (const inet::Ipv6Address &multicastAddress: ipv6Data->getJoinedMulticastGroups())
        EV_INFO << multicastAddress.str() << std::endl;
}
