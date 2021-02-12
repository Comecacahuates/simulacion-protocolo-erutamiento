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

#include "veins_proj/networklayer/configurator/CarConfigurator.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/geometry/common/Coord.h"
#include "veins/base/utils/Coord.h"
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/networklayer/ipv6/Ipv6GeohashAddress.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "boost/tuple/tuple.hpp"
#include "boost/swap.hpp"

using namespace veins_proj;


Define_Module(CarConfigurator);


void CarConfigurator::initialize(int stage) {
    OperationalBase::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        // Parameters
        interface = par("interface").stdstringValue();
        locationUpdateInterval = par("locationUpdateInterval");

        // Context
        host = inet::getContainingNode(this);

        interfaceTable = inet::L3AddressResolver().interfaceTableOf(host);

        if (!interfaceTable)
            throw omnetpp::cRuntimeError("No interface table found");

        mobility = omnetpp::check_and_cast<CarMobility *>(host->getSubmodule("mobility"));

        if (!mobility)
            throw omnetpp::cRuntimeError("No mobility module found");

        addressCache = omnetpp::check_and_cast<AddressCache *>(host->getSubmodule("addressCache"));

        if (!addressCache)
            throw omnetpp::cRuntimeError("No address cache module found");

        // Self messages
        locationUpdateTimer = new omnetpp::cMessage("LocationUpdateTimer");

    } else if (stage == inet::INITSTAGE_NETWORK_INTERFACE_CONFIGURATION) {
        networkInterface = interfaceTable->findInterfaceByName(interface.c_str());

        if (!networkInterface)
            throw omnetpp::cRuntimeError("No such interface '%s'", interface.c_str());

        if (networkInterface->isLoopback())
            throw omnetpp::cRuntimeError("Interface %s is loopback", interface.c_str());
    }
}


void CarConfigurator::handleMessageWhenUp(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    EV_INFO << "CarConfigurator::handleMessageWhenUp" << std::endl;

    if (message->isSelfMessage())
        processSelfMessage(message);
}


void CarConfigurator::processSelfMessage(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    EV_INFO << "CarConfigurator::processSelfMessage" << std::endl;

    if (message == locationUpdateTimer)
        processLocationUpdateTimer();
}


void CarConfigurator::scheduleLocationUpdateTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    EV_INFO << "CarConfigurator::scheduleLocationUpdateTimer" << std::endl;

    scheduleAt(omnetpp::simTime() + locationUpdateInterval, locationUpdateTimer);
}


void CarConfigurator::processLocationUpdateTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    EV_INFO << "CarConfigurator::processLocationUpdateTimer" << std::endl;

    if (mobility->locationChanged()) {
        EV_INFO << "Cambió la ubicación" << std::endl;

        const Graph &graph = mobility->getRoadNetwork()->getGraph();

        Vertex gatewayVertex;
        bool isAtGateway;
        boost::tie(gatewayVertex, isAtGateway) = mobility->isAtGateway();

        // Si el vehículo se encuentra en un gateway, se une a la red secundaria
        if (isAtGateway) {
            GeohashLocation::Direction gatewayType = graph[gatewayVertex].gatewayType;
            GeohashLocation neighbourGeohashRegion;
            mobility->getRoadNetwork()->getGeohashRegion().getNeighbour(gatewayType, neighbourGeohashRegion);

            joinNetwork(neighbourGeohashRegion, SECONDARY_NETWORK);

        // Si el vehículo no se encuentra en un gateway, se sale de la red secundaria
        } else
            leaveNetwork(SECONDARY_NETWORK);
    } else
        EV_INFO << "No cambió la ubicación" << std::endl;

    // Si el vehículo cambió de una región a otra, se intercambian las subredes primaria y secundaria
    if (mobility->regionChanged()) {
        swapNetworks();
    }

    scheduleLocationUpdateTimer();
}


void CarConfigurator::handleStartOperation(inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("CarConfigurator::handleStartOperation");

    initInterface();

    // El vehículo se une a la subred de la región donde se encuentra
    joinNetwork(mobility->getGeohashLocation(), PRIMARY_NETWORK);

    scheduleLocationUpdateTimer();
}


void CarConfigurator::handleStopOperation(inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("CarConfigurator::handleStartOperation");

    cancelAndDelete(locationUpdateTimer);
}


void CarConfigurator::handleCrashOperation(inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("CarConfigurator::handleStartOperation");

    cancelAndDelete(locationUpdateTimer);
}


void CarConfigurator::initInterface() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("CarConfigurator::initInterface");

    inet::Ipv6InterfaceData *ipv6Data = networkInterface->findProtocolDataForUpdate<inet::Ipv6InterfaceData>();
    ipv6Data->setAdvSendAdvertisements(false);
}


void CarConfigurator::joinNetwork(const GeohashLocation &geohashRegion, const int &networkType) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("CarConfigurator::joinNetwork");
    EV_INFO << "Network geohash: " << geohashRegions[networkType].getGeohashString() << std::endl;
    EV_INFO << "New network geohash: " << geohashRegion.getGeohashString() << std::endl;

    // Si la región geohash es igual a la nueva región geohash, no se hace nada
    if (geohashRegions[networkType] == geohashRegion)
        return;

    inet::Ipv6InterfaceData* ipv6Data = networkInterface->findProtocolDataForUpdate<inet::Ipv6InterfaceData>();

    inet::Ipv6Address unicastAddress = Ipv6GeohashAddress::ipv6UnicastAddress(geohashRegion, networkInterface->getInterfaceToken());
    addressCache->setUnicastAddress(unicastAddress, networkType);
    ipv6Data->assignAddress(unicastAddress, false, SIMTIME_ZERO, SIMTIME_ZERO);

    // Se remplaza la dirección multicast
    inet::Ipv6Address multicastAddress = Ipv6GeohashAddress::ipv6MulticastAddress(geohashRegion);
    addressCache->setMulticastAddress(multicastAddress, networkType);

    ipv6Data->joinMulticastGroup(multicastAddress);
    ipv6Data->assignAddress(multicastAddress, false, SIMTIME_ZERO, SIMTIME_ZERO);

    geohashRegions[networkType].setGeohash(geohashRegion.getGeohashString());

    EV_INFO << "Unicast address: " << unicastAddress.str() << std::endl;
    EV_INFO << "Multicast address: " << multicastAddress.str() << std::endl;

    showAddresses();
}


void CarConfigurator::leaveNetwork(const int &networkType) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("CarConfigurator::leaveNetwork");

    // Si la región geohash primaria es nula, no se hace nada
    if (geohashRegions[networkType].isNull())
        return;

    inet::Ipv6InterfaceData *ipv6Data = networkInterface->findProtocolDataForUpdate<inet::Ipv6InterfaceData>();

    const inet::Ipv6Address &unicastAddress = addressCache->getUnicastAddress(networkType);

    // Se elimina la dirección unicast
    if (ipv6Data->hasAddress(unicastAddress)) {
        ipv6Data->removeAddress(unicastAddress);
    };

    const inet::Ipv6Address &multicastAddress = addressCache->getMulticastAddress(networkType);

    // Se elimina la dirección multicast
    if (ipv6Data->isMemberOfMulticastGroup(multicastAddress)) {
        ipv6Data->leaveMulticastGroup(multicastAddress);
        ipv6Data->removeAddress(multicastAddress);
    }

    addressCache->clear(networkType);

    showAddresses();
}


void CarConfigurator::swapNetworks() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("CarConfigurator::swapNetworks");

    boost::swap(geohashRegions[PRIMARY_NETWORK], geohashRegions[SECONDARY_NETWORK]);
    addressCache->swapAddresses();
}


void CarConfigurator::showAddresses() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("CarConfigurator::showAddresses");

    const inet::Ipv6InterfaceData* ipv6Data = networkInterface->findProtocolData<inet::Ipv6InterfaceData>();

    EV_INFO << "Unicast addresses" << std::endl;

    for (int i = 0; i < ipv6Data->getNumAddresses(); i++)
        EV_INFO << ipv6Data->getAddress(i).str() << std::endl;

    EV_INFO << "Multicast addresses" << std::endl;

    for (const inet::Ipv6Address &multicastAddress: ipv6Data->getJoinedMulticastGroups())
        EV_INFO << multicastAddress.str() << std::endl;
}
