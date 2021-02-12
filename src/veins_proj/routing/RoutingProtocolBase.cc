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

#include "veins_proj/routing/RoutingProtocolBase.h"
#include "inet/common/INETUtils.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/IProtocolRegistrationListener.h"
#include "inet/common/Protocol.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/packet/chunk/Chunk.h"
#include "inet/networklayer/common/HopLimitTag_m.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/networklayer/common/NextHopAddressTag_m.h"
#include "inet/networklayer/contract/ipv6/Ipv6AddressType.h"
#include "inet/networklayer/ipv6/Ipv6Route.h"
#include "inet/networklayer/nexthop/NextHopForwardingHeader_m.h"
#include "inet/transportlayer/udp/UdpHeader_m.h"
#include "veins_proj/routing/Routing_m.h"
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include <cmath>
#include <vector>
#include <limits>
#include <boost/stacktrace.hpp>

using namespace veins_proj;


Register_Abstract_Class(RoutingProtocolBase);


void RoutingProtocolBase::initialize(int stage) {
    inet::RoutingProtocolBase::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        // Routing parameters
        helloCarInterval = par("helloCarInterval");
        neighbouringCarValidityTime = par("neighbouringCarValidityTime");
        helloHostInterval = par("helloHostInterval");
        neighbouringHostValidityTime = par("neighbouringHostValidityTime");
        pingInterval = par("pingInterval");
        pongTimeout = par("pongTimeout");
        activeEdgeValidityTime = par("activeEdgeValidityTime");
        inactiveEdgeValidityTime = par("inactiveEdgeValidityTime");
        routeValidityTime = par("routeValidityTime");
        vertexProximityRadius = par("vertexProximityRadius");

        // Context
        host = inet::getContainingNode(this);

        interfaceTable = inet::L3AddressResolver().interfaceTableOf(host);

        if (!interfaceTable)
            throw omnetpp::cRuntimeError("No interface table found");

        routingTable = inet::L3AddressResolver().findIpv6RoutingTableOf(host);

        if (!routingTable)
            throw omnetpp::cRuntimeError("No routing table found");

        networkProtocol = inet::getModuleFromPar<inet::INetfilter>(par("networkProtocolModule"), this);

        if (!networkProtocol)
            throw omnetpp::cRuntimeError("No network protocol module found");

        addressCache = omnetpp::check_and_cast<AddressCache *>(host->getSubmodule("addressCache"));

        if (!addressCache)
            throw omnetpp::cRuntimeError("No host configurator module found");

        roadNetworkDatabase = omnetpp::check_and_cast<RoadNetworkDatabase *>(getModuleByPath(par("roadNetworkDatabaseModule")));

        if (!roadNetworkDatabase)
            throw omnetpp::cRuntimeError("No roadway database module found");

        // Self messages
        purgeNeighbouringCarsTimer = new omnetpp::cMessage("purgeNeighbouringCarsTimer");

    } else if (stage == inet::INITSTAGE_NETWORK_INTERFACE_CONFIGURATION) {
        networkInterface = interfaceTable->findInterfaceByName(par("outputInterface"));

        if (!networkInterface)
            throw omnetpp::cRuntimeError("Output interface not found");

    } else if (stage == inet::INITSTAGE_ROUTING_PROTOCOLS) {
        inet::registerService(inet::Protocol::manet, nullptr, gate("ipIn"));
        inet::registerProtocol(inet::Protocol::manet, gate("ipOut"), nullptr);
        host->subscribe(inet::linkBrokenSignal, this);
        networkProtocol->registerHook(0, this);
    }
}


void RoutingProtocolBase::handleMessageWhenUp(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::handleMessageWhenUp");
    EV_INFO << message->getName() << std::endl;
    //EV_INFO << boost::stacktrace::stacktrace() << std::endl;

    if (message->isSelfMessage())
        processSelfMessage(message);

    else
        processMessage(message);
}


void RoutingProtocolBase::processSelfMessage(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::processSelfMessage");

    if (message == purgeNeighbouringCarsTimer)
        processPurgeNeighbouringCarsTimer();

    else
        throw omnetpp::cRuntimeError("Unknown self message");
}


void RoutingProtocolBase::processMessage(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::processMessage");

    inet::Packet *packet = omnetpp::check_and_cast<inet::Packet *>(message);

    if (packet != nullptr)
        processUdpPacket(packet);

    else
        throw omnetpp::cRuntimeError("Unknown message");

    delete packet;
}


void RoutingProtocolBase::schedulePurgeNeighbouringCarsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::schedulePurgeNeighbouringCarsTimer");

    omnetpp::simtime_t nextExpirationTime = getNextNeighbouringCarExpirationTime();

    EV_INFO << "Next expiration time: " << nextExpirationTime << std::endl;

    if (nextExpirationTime == omnetpp::SimTime::getMaxTime()) {
        if (purgeNeighbouringCarsTimer->isScheduled())
            cancelEvent(purgeNeighbouringCarsTimer);

    } else {
        if (!purgeNeighbouringCarsTimer->isScheduled())
            scheduleAt(nextExpirationTime, purgeNeighbouringCarsTimer);

        else if (purgeNeighbouringCarsTimer->getArrivalTime() != nextExpirationTime) {
                cancelEvent(purgeNeighbouringCarsTimer);
                scheduleAt(nextExpirationTime, purgeNeighbouringCarsTimer);
            }
    }
}


void RoutingProtocolBase::processPurgeNeighbouringCarsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::processPurgeNeighbouringCarsTimer");

    purgeNeighbouringCars();
    schedulePurgeNeighbouringCarsTimer();
}


void RoutingProtocolBase::sendUdpPacket(inet::Packet *packet) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::sendUdpPacket");
    send(packet, "ipOut");
}


void RoutingProtocolBase::processUdpPacket(inet::Packet *udpPacket) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::processUdpPacket");
    EV_INFO << "UDP packet: " << udpPacket->getName() << std::endl;

    udpPacket->popAtFront<inet::UdpHeader>();

    const inet::Ptr<const RoutingPacket> &routingPacket = udpPacket->popAtFront<RoutingPacket>();

    PacketType packetType = routingPacket->getPacketType();

    switch (packetType) {

    case PacketType::ACK: {
        inet::Ptr<Ack> ack = inet::dynamicPtrCast<Ack>(routingPacket->dupShared());
        processAck(ack);

        break;
    }

    case PacketType::HELLO_CAR: {
        inet::Ptr<HelloCar> helloCar = inet::dynamicPtrCast<HelloCar>(routingPacket->dupShared());
        processHelloCar(helloCar);

        break;
    }

    case PacketType::HELLO_HOST: {
        inet::Ptr<HelloHost> helloHost = inet::dynamicPtrCast<HelloHost>(routingPacket->dupShared());
        processHelloHost(helloHost);

        break;
    }

    case PacketType::PING: {
        inet::Ptr<Ping> ping = inet::dynamicPtrCast<Ping>(routingPacket->dupShared());
        processPing(ping);

        break;
    }

    default:
        throw omnetpp::cRuntimeError("Routing packet arrived with undefined packet type: %d", packetType);
    }
}


const inet::Ptr<Ack> RoutingProtocolBase::createAck(const inet::Ipv6Address &address) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::createAck");

    const inet::Ptr<Ack> &ack = inet::makeShared<Ack>();

    EV_INFO << "Address: " << address.str() << std::endl;

    ack->setAddress(address);

    return ack;
}


void RoutingProtocolBase::sendAck(const inet::Ptr<Ack> &ack, const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::sendAck");
    EV_INFO << "Destination address: " << destAddress.str() << std::endl;

    inet::Packet *udpPacket = new inet::Packet("Ack");
    udpPacket->insertAtBack(ack);

    inet::Ptr<inet::UdpHeader> udpHeader = inet::makeShared<inet::UdpHeader>();
    udpHeader->setSourcePort(ROUTING_PROTOCOL_UDP_PORT);
    udpHeader->setDestinationPort(ROUTING_PROTOCOL_UDP_PORT);
    udpPacket->insertAtFront(udpHeader);

    inet::Ptr<inet::L3AddressReq> addresses = udpPacket->addTagIfAbsent<inet::L3AddressReq>();
    addresses->setSrcAddress(ack->getAddress());
    addresses->setDestAddress(inet::L3Address(destAddress));

    inet::Ptr<inet::HopLimitReq> hopLimit = udpPacket->addTagIfAbsent<inet::HopLimitReq>();
    hopLimit->setHopLimit(255);

    inet::Ptr<inet::PacketProtocolTag> packetProtocol = udpPacket->addTagIfAbsent<inet::PacketProtocolTag>();
    packetProtocol->setProtocol(&inet::Protocol::manet);

    inet::Ptr<inet::DispatchProtocolReq> dispatchProtocol = udpPacket->addTagIfAbsent<inet::DispatchProtocolReq>();
    dispatchProtocol->setProtocol(&inet::Protocol::ipv6);

    sendUdpPacket(udpPacket);
}


void RoutingProtocolBase::processAck(const inet::Ptr<Ack> &ack) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::processAck");
    EV_INFO << "Address: " << ack->getAddress().str() << std::endl;
}


void RoutingProtocolBase::showNeighbouringCars() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::showNeighbouringCars");

    for (const NeighbouringCar &neighbouringCar: neighbouringCars) {
        EV_INFO << "Address: " << neighbouringCar.first << std::endl;
        EV_INFO << "Edge: " << neighbouringCar.second.locationOnRoadNetwork.edge << std::endl;
        EV_INFO << "Distance to vertex A: " << neighbouringCar.second.locationOnRoadNetwork.distanceToVertexA << std::endl;
        EV_INFO << "Distance to vertex B: " << neighbouringCar.second.locationOnRoadNetwork.distanceToVertexB << std::endl;
    }
}


void RoutingProtocolBase::removeOldNeighbouringCars(omnetpp::simtime_t time) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::removeOldNeighbouringCars");

    NeighbouringCarsIterator it = neighbouringCars.begin();
    while (it != neighbouringCars.end())
        if (it->second.lastUpdateTime <= time) {
            //purgeNextHopRoutes(it->first); // Se elimina las rutas
            neighbouringCars.erase(it++); // Se elimina del directorio de veh��culos vecinos

        } else
            it++;
}


omnetpp::simtime_t RoutingProtocolBase::getOldestNeighbouringCarTime() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::getOldestNeighbouringCarTime");

    omnetpp::simtime_t oldestTime = omnetpp::SimTime::getMaxTime();

    for (const NeighbouringCar &neighbouringCar: neighbouringCars) {
        const omnetpp::simtime_t &time = neighbouringCar.second.lastUpdateTime;

        if (time < oldestTime)
            oldestTime = time;
    }

    return oldestTime;
}


omnetpp::simtime_t RoutingProtocolBase::getNextNeighbouringCarExpirationTime() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::getNextNeighbouringCarExpirationTime");

    omnetpp::simtime_t oldestEntryTime = getOldestNeighbouringCarTime();

    if (oldestEntryTime == omnetpp::SimTime::getMaxTime())
        return oldestEntryTime;

    else
        return oldestEntryTime + neighbouringCarValidityTime;
}


void RoutingProtocolBase::purgeNeighbouringCars() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::purgeNeighbouringCars");

    removeOldNeighbouringCars(omnetpp::simTime() - neighbouringCarValidityTime);
    removeOldRoutes(omnetpp::simTime());
}


inet::Ipv6Address RoutingProtocolBase::getClosestNeighbouringCarAddress(const GeohashLocation &geohashLocation) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::getClosestNeighbouringCarAddress");

    double minDistance = std::numeric_limits<double>::infinity();
    inet::Ipv6Address closestNeighbourAddress = inet::Ipv6Address::UNSPECIFIED_ADDRESS;

    for (const NeighbouringCar &neighbouringCar: neighbouringCars) {
        double distance = geohashLocation.getDistance(neighbouringCar.second.geohashLocation);

        if (distance < minDistance) {
            minDistance = distance;
            closestNeighbourAddress = neighbouringCar.first;
        }
    }

    return closestNeighbourAddress;
}


void RoutingProtocolBase::addRoute(const inet::Ipv6Address &destPrefix, const short prefixLength, const inet::Ipv6Address &nextHop, int metric, omnetpp::simtime_t expiryTime) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::addRoute");

    // Se revisa si ya existe una ruta
    bool routeExists = false;
    inet::Ipv6Route *route;
    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        route = routingTable->getRoute(i);
        if (route->getDestPrefix().matches(destPrefix, prefixLength) && route->getNextHop() == nextHop) {
            routeExists = true;
            break;
        }
    }

    // Si existe la ruta, se actualiza
    if (routeExists) {
        route->setMetric(metric);
        route->setExpiryTime(expiryTime);

    // Si no existe la ruta, se grega
    } else {
        route = new inet::Ipv6Route(destPrefix, prefixLength, inet::IRoute::MANET);
        route->setNextHop(nextHop);
        route->setMetric(metric);
        route->setSource(this);
        route->setInterface(networkInterface);
        route->setExpiryTime(expiryTime);
        routingTable->addRoute(route);

        EV_INFO << "Route: " << route->str() << std::endl;
    }
}


void RoutingProtocolBase::purgeNextHopRoutes(const inet::Ipv6Address &nextHopAddress) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::purgeNextHopRoutes");

    inet::Ipv6Route *route;

    for (int i = routingTable->getNumRoutes() - 1; i >= 0 ; i--) {
        route = routingTable->getRoute(i);

        if (route != nullptr && route->getSourceType() == inet::IRoute::MANET)
            if (nextHopAddress == route->getNextHop())
                routingTable->deleteRoute(route);
    }
}


void RoutingProtocolBase::removeOldRoutes(omnetpp::simtime_t time) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::purgeNextHopRoutes");

    inet::Ipv6Route *route;

    for (int i = routingTable->getNumRoutes() - 1; i >= 0; i--) {
        route = routingTable->getRoute(i);

        if (route != nullptr && route->getSourceType() == inet::IRoute::MANET)
            if (route->getExpiryTime() <= time)
                routingTable->deleteRoute(route);
    }
}


void RoutingProtocolBase::showRoutes() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::showRoutes");

    for (int i = 0; i < routingTable->getNumRoutes(); i++)
        EV_INFO << "Route: " << routingTable->getRoute(i) << std::endl;
}


void RoutingProtocolBase::setTlvOption(inet::Packet *datagram, inet::TlvOptionBase *tlvOption) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::setTlvOption");

    datagram->trimFront();

    inet::Ptr<inet::Ipv6Header> ipv6Header = inet::removeNetworkProtocolHeader<inet::Ipv6Header>(datagram);
    inet::B oldHeaderLength = ipv6Header->calculateHeaderByteLength();
    inet::Ipv6ExtensionHeader *extensionHeader = ipv6Header->findExtensionHeaderByTypeForUpdate(inet::IpProtocolId::IP_PROT_IPv6EXT_HOP);
    inet::Ipv6HopByHopOptionsHeader *optionsHeader = omnetpp::check_and_cast_nullable<inet::Ipv6HopByHopOptionsHeader *>(extensionHeader);

    if (!optionsHeader) {
        optionsHeader = new inet::Ipv6HopByHopOptionsHeader();
        optionsHeader->setByteLength(inet::B(8));
        ipv6Header->addExtensionHeader(optionsHeader);
    }

    optionsHeader->getTlvOptionsForUpdate().insertTlvOption(tlvOption);
    optionsHeader->setByteLength(inet::B(inet::utils::roundUp(2 + inet::B(optionsHeader->getTlvOptions().getLength()).get(), 8)));
    inet::B newHeaderLength = ipv6Header->calculateHeaderByteLength();
    ipv6Header->addChunkLength(newHeaderLength - oldHeaderLength);
    inet::insertNetworkProtocolHeader(datagram, inet::Protocol::ipv6, ipv6Header);
}


TlvDestGeohashLocationOption *RoutingProtocolBase::createTlvDestGeohashLocationOption(uint64_t geohashLocationBits) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::createTlvDestGeohashLocationOption");

    TlvDestGeohashLocationOption *tlvOption = new TlvDestGeohashLocationOption();
    tlvOption->setGeohash(geohashLocationBits);
    tlvOption->setLength(computeTlvOptionLength(tlvOption));
    tlvOption->setType(IPV6TLVOPTION_TLV_DEST_GEOHASH_LOCATION);
    return tlvOption;
}


int RoutingProtocolBase::computeTlvOptionLength(TlvDestGeohashLocationOption *tlvOption) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::computeTlvOptionLength");

    int geohashBytes = 8;
    return geohashBytes;
}


TlvDestLocationOnRoadNetworkOption *RoutingProtocolBase::createTlvDestLocationOnRoadNetworkOption(Vertex vertexA, Vertex vertexB, double distanceToVertexA) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::createTlvDestLocationOnRoadNetworkOption");

    TlvDestLocationOnRoadNetworkOption *tlvOption = new TlvDestLocationOnRoadNetworkOption();
    tlvOption->setVertexA(vertexA);
    tlvOption->setVertexB(vertexB);
    tlvOption->setDistanceToVertexA(distanceToVertexA);
    tlvOption->setLength(computeTlvOptionLength(tlvOption));
    tlvOption->setType(IPV6TLVOPTION_TLV_DEST_ON_ROAD_NETWORK_LOCATION);
    return tlvOption;
}


void RoutingProtocolBase::setTlvDestLocationOnRoadNetworkOption(inet::Packet *datagram, const GeohashLocation &destGeohashLocation) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::setTlvDestLocationOnRoadNetworkOption");

    RoadNetwork *roadNetwork = roadNetworkDatabase->getRoadNetwork(destGeohashLocation);
    LocationOnRoadNetwork locationOnRoadNetwork;
    roadNetwork->getLocationOnRoadNetwork(destGeohashLocation.getLocation(), 0, 0, locationOnRoadNetwork);
    const Graph &graph = roadNetwork->getGraph();
    Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);
    double &distanceToVertexA = locationOnRoadNetwork.distanceToVertexA;

    TlvDestLocationOnRoadNetworkOption *destLocationOnRoadNetworkOption = createTlvDestLocationOnRoadNetworkOption(vertexA, vertexB, distanceToVertexA);
    setTlvOption(datagram, destLocationOnRoadNetworkOption);
}


int RoutingProtocolBase::computeTlvOptionLength(TlvDestLocationOnRoadNetworkOption *tlvOption) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::computeTlvOptionLength");

    int vertexABytes = 2;
    int vertexBBytes = 2;
    int distanceToVertexABytes = 2;
    return vertexABytes + vertexBBytes + distanceToVertexABytes;
}


TlvRoutingOption *RoutingProtocolBase::createTlvRoutingOption() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::createTlvRoutingOption");

    TlvRoutingOption *tlvOption = new TlvRoutingOption();
    tlvOption->setVisitedVerticesArraySize(0);
    return tlvOption;
}


void RoutingProtocolBase::setTlvRoutingOption(inet::Packet *datagram) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::setTlvRoutingOption");

    TlvRoutingOption *routingOption = createTlvRoutingOption();
    setTlvOption(datagram, routingOption);
}


int RoutingProtocolBase::computeTlvOptionLength(TlvRoutingOption *tlvOption) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::computeTlvOptionLength");

    int visitedVerticesBytes = 2 * tlvOption->getVisitedVerticesArraySize();
    return visitedVerticesBytes;
}


void RoutingProtocolBase::handleStartOperation(inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::handleStartOperation");
}


void RoutingProtocolBase::handleStopOperation(inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::handleStopOperation");

    cancelAndDelete(purgeNeighbouringCarsTimer);
    neighbouringCars.clear();
}


void RoutingProtocolBase::handleCrashOperation(inet::LifecycleOperation *operation)  {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolBase::handleCrashOperation");

    cancelAndDelete(purgeNeighbouringCarsTimer);
    neighbouringCars.clear();
}
