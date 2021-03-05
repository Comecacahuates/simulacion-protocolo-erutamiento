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

#include "veins_proj/routing/RoutingProtocolCar.h"
#include "inet/common/Protocol.h"
#include "inet/common/ProtocolTag_m.h"
#include "inet/common/IProtocolRegistrationListener.h"
#include "inet/common/packet/chunk/Chunk.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/ipv6/Ipv6InterfaceData.h"
#include "inet/networklayer/ipv6/Ipv6Route.h"
#include "inet/networklayer/contract/ipv6/Ipv6AddressType.h"
#include "inet/networklayer/common/HopLimitTag_m.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/networklayer/common/NextHopAddressTag_m.h"
#include "inet/transportlayer/udp/UdpHeader_m.h"
#include "inet/networklayer/common/L3Tools.h"
#include "veins_proj/routing/Routing_m.h"
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "veins_proj/mobility/CarMobility.h"
#include "veins_proj/mobility/StaticHostMobility.h"
#include <cmath>
#include <vector>
#include <limits>
#include <boost/stacktrace.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/tuple/tuple.hpp>

using namespace veins_proj;


Define_Module(RoutingProtocolCar);
//Register_Abstract_Class(RoutingProtocolCar);


void RoutingProtocolCar::initialize(int stage) {
    RoutingProtocolBase::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        // Context
        mobility = omnetpp::check_and_cast<CarMobility *>(host->getSubmodule("mobility"));

        if (!mobility)
            throw omnetpp::cRuntimeError("No mobility module found");

        // Self messages
        helloCarTimer = new omnetpp::cMessage("helloCarTimer");
        pongTimeoutTimer = new omnetpp::cMessage("pongTimeoutTimer");
        purgeNeighbouringHostsTimer = new omnetpp::cMessage("purgeNeighbouringHostsTimer");
        purgeActiveEdgesTimer = new omnetpp::cMessage("purgeActiveEdgesTimer");
        purgeInactiveEdgesTimer = new omnetpp::cMessage("purgeInactiveEdgesTimer");
    }
}


void RoutingProtocolCar::processSelfMessage(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::processSelfMessage");

    if (message == helloCarTimer)
        processHelloCarTimer();

    else if (message == pongTimeoutTimer)
        processPongTimeoutTimer();

    else if (message == purgeNeighbouringHostsTimer)
        processPurgeNeighbouringHostsTimer();

    else if (message == purgeActiveEdgesTimer)
        processPurgeActiveEdgesTimer();

    else
        RoutingProtocolBase::processSelfMessage(message);
}


void RoutingProtocolCar::scheduleHelloCarTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::scheduleHelloCarTimer");

    scheduleAt(omnetpp::simTime() + helloCarInterval + uniform(0, 1), helloCarTimer);
}


void RoutingProtocolCar::processHelloCarTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::processHelloCarTimer");

    const inet::Ipv6Address &primaryUnicastAddress = addressCache->getUnicastAddress(PRIMARY_ADDRESS);
    const inet::Ipv6Address &primaryMulticastAddress = addressCache->getMulticastAddress(PRIMARY_ADDRESS);
    const inet::Ipv6Address &secondaryUnicastAddress = addressCache->getUnicastAddress(SECONDARY_ADDRESS);
    const inet::Ipv6Address &secondaryMulticastAddress = addressCache->getMulticastAddress(SECONDARY_ADDRESS);

    const inet::Ipv6InterfaceData *ipv6Data = networkInterface->findProtocolData<inet::Ipv6InterfaceData>();

    if (ipv6Data->hasAddress(primaryUnicastAddress))
        sendHelloCar(createHelloCar(primaryUnicastAddress), primaryMulticastAddress);

    if (ipv6Data->hasAddress(secondaryUnicastAddress))
        sendHelloCar(createHelloCar(secondaryUnicastAddress), secondaryMulticastAddress);

    scheduleHelloCarTimer();
}


void RoutingProtocolCar::schedulePongTimeoutTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::schedulePongTimeoutTimer");

    scheduleAt(omnetpp::simTime() + pongTimeout, pongTimeoutTimer);
}


void RoutingProtocolCar::processPongTimeoutTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::processPongTimeoutTimer");

    Edge edge = mobility->getLocationOnRoadNetwork().edge;

    // Eliminar la arista de las aristas activas
    if (activeEdges.find(edge) != activeEdges.end()) {
        activeEdges.erase(edge);
        schedulePurgeActiveEdgesTimer();
    }

    // Enviar mensaje de estatus de arista inactiva
}


void RoutingProtocolCar::schedulePurgeNeighbouringHostsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::schedulePurgeNeighbouringHostsTimer");

    omnetpp::simtime_t nextExpirationTime = getNextNeighbouringHostExpirationTime();

    EV_INFO << "Next expiration time: " << nextExpirationTime << std::endl;

    if (nextExpirationTime == omnetpp::SimTime::getMaxTime()) {
        if (purgeNeighbouringHostsTimer->isScheduled())
            cancelEvent(purgeNeighbouringHostsTimer);

    } else {
        if (!purgeNeighbouringHostsTimer->isScheduled())
            scheduleAt(nextExpirationTime, purgeNeighbouringHostsTimer);

        else if (purgeNeighbouringHostsTimer->getArrivalTime() != nextExpirationTime) {
            cancelEvent(purgeNeighbouringHostsTimer);
            scheduleAt(nextExpirationTime, purgeNeighbouringHostsTimer);
        }
    }
}


void RoutingProtocolCar::processPurgeNeighbouringHostsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::processPurgeNeighbouringHostsTimer");

    purgeNeighbouringHosts();
    schedulePurgeNeighbouringHostsTimer();
}


void RoutingProtocolCar::schedulePurgeActiveEdgesTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::schedulePurgeActiveEdgesTimer");

    omnetpp::simtime_t nextExpirationTime = getNextActiveEdgeExpirationTime();

    EV_INFO << "Next expiration time: " << nextExpirationTime << std::endl;

    if (nextExpirationTime == omnetpp::SimTime::getMaxTime()) {
        if (purgeActiveEdgesTimer->isScheduled())
            cancelEvent(purgeActiveEdgesTimer);

    } else {
        if (!purgeActiveEdgesTimer->isScheduled())
            scheduleAt(nextExpirationTime, purgeActiveEdgesTimer);

        else if (purgeActiveEdgesTimer->getArrivalTime() != nextExpirationTime) {
            cancelEvent(purgeActiveEdgesTimer);
            scheduleAt(nextExpirationTime, purgeActiveEdgesTimer);
        }
    }
}


void RoutingProtocolCar::processPurgeActiveEdgesTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::processPurgeActiveEdgesTimer");

    purgeActiveEdges();
    schedulePurgeActiveEdgesTimer();
}


void RoutingProtocolCar::schedulePurgeInactiveEdgesTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::schedulePurgeInactiveEdgesTimer");

    omnetpp::simtime_t nextExpirationTime = getNextInactiveEdgeExpirationTime();

    EV_INFO << "Next expiration time: " << nextExpirationTime << std::endl;

    if (nextExpirationTime == omnetpp::SimTime::getMaxTime()) {
        if (purgeInactiveEdgesTimer->isScheduled())
            cancelEvent(purgeInactiveEdgesTimer);

    } else {
        if (!purgeInactiveEdgesTimer->isScheduled())
            scheduleAt(nextExpirationTime, purgeInactiveEdgesTimer);

        else if (purgeInactiveEdgesTimer->getArrivalTime() != nextExpirationTime) {
            cancelEvent(purgeInactiveEdgesTimer);
            scheduleAt(nextExpirationTime, purgeInactiveEdgesTimer);
        }
    }
}


void RoutingProtocolCar::processPurgeInactiveEdgesTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::processPurgeInactiveEdgesTimer");

    purgeInactiveEdges();
    schedulePurgeInactiveEdgesTimer();
}


const inet::Ptr<HelloCar> RoutingProtocolCar::createHelloCar(const inet::Ipv6Address &carAddress) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::createHelloCar");

    const inet::Ptr<HelloCar> &helloCar = inet::makeShared<HelloCar>();

    const Graph &graph = mobility->getRoadNetwork()->getGraph();

    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);
    const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertex1;
    const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertex2;
    const GeohashLocation &geohashLocation = mobility->getGeohashLocation();
    double speed = mobility->getSpeed();
    double direction = mobility->getDirection();

    EV_INFO << "Address: " << carAddress.str() << std::endl;
    EV_INFO << "Geohash location: " << geohashLocation.getGeohashString() << std::endl;
    EV_INFO << "                : " << geohashLocation.getBits() << std::endl;
    EV_INFO << "Speed: " << speed << std::endl;
    EV_INFO << "Direction: " << direction << std::endl;
    EV_INFO << "Vertex A: " << vertexA << std::endl;
    EV_INFO << "Vertex B: " << vertexB << std::endl;
    EV_INFO << "Distance to vertex A: " << distanceToVertexA << std::endl;
    EV_INFO << "Distance to vertex B: " << distanceToVertexB << std::endl;

    helloCar->setAddress(carAddress);
    helloCar->setGeohash(geohashLocation.getBits());
    helloCar->setSpeed(speed);
    helloCar->setDirection(direction);
    helloCar->setVertexA(vertexA);
    helloCar->setVertexB(vertexB);
    helloCar->setDistanceToVertexA(distanceToVertexA);

    return helloCar;
}


void RoutingProtocolCar::sendHelloCar(const inet::Ptr<HelloCar> &helloCar, const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::sendHelloCar");

    inet::Packet *udpPacket = new inet::Packet("ANC_VEHIC");
    udpPacket->insertAtBack(helloCar);

    inet::Ptr<inet::UdpHeader> udpHeader = inet::makeShared<inet::UdpHeader>();
    udpHeader->setSourcePort(ROUTING_PROTOCOL_UDP_PORT);
    udpHeader->setDestinationPort(ROUTING_PROTOCOL_UDP_PORT);
    udpPacket->insertAtFront(udpHeader);

    inet::Ptr<inet::L3AddressReq> addresses = udpPacket->addTagIfAbsent<inet::L3AddressReq>();
    addresses->setSrcAddress(inet::L3Address(helloCar->getAddress()));
    addresses->setDestAddress(inet::L3Address(destAddress));

    inet::Ptr<inet::HopLimitReq> hopLimit = udpPacket->addTagIfAbsent<inet::HopLimitReq>();
    hopLimit->setHopLimit(255);

    inet::Ptr<inet::PacketProtocolTag> packetProtocol = udpPacket->addTagIfAbsent<inet::PacketProtocolTag>();
    packetProtocol->setProtocol(&inet::Protocol::manet);

    inet::Ptr<inet::DispatchProtocolReq> dispatchProtocol = udpPacket->addTagIfAbsent<inet::DispatchProtocolReq>();
    dispatchProtocol->setProtocol(&inet::Protocol::ipv6);

    sendUdpPacket(udpPacket);
}


void RoutingProtocolCar::processHelloCar(const inet::Ptr<HelloCar> &helloCar) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::processHelloHost");

    const inet::Ipv6Address &carAddress = helloCar->getAddress();
    GeohashLocation geohashLocation(helloCar->getGeohash(), 12);
    double speed = helloCar->getSpeed();
    double direction = helloCar->getDirection();
    Vertex vertexA = (Vertex) helloCar->getVertexA();
    Vertex vertexB = (Vertex) helloCar->getVertexB();
    double distanceToVertexA = helloCar->getDistanceToVertexA();

    const Graph &graph = roadNetworkDatabase->getRoadNetwork(geohashLocation)->getGraph();
    Edge edge = boost::edge(vertexA, vertexB, graph).first;
    double distanceToVertexB = graph[edge].length - distanceToVertexA;
    LocationOnRoadNetwork locationOnRoadNetwork = { edge, 0, distanceToVertexA, distanceToVertexB };

    EV_INFO << "Address: " << carAddress.str() << std::endl;
    EV_INFO << "Geohash location: " << geohashLocation.getGeohashString() << std::endl;
    EV_INFO << "Speed: " << speed << std::endl;
    EV_INFO << "Direction: " << direction << std::endl;
    EV_INFO << "Vertex A: " << vertexA << std::endl;
    EV_INFO << "Vertex B: " << vertexB << std::endl;
    EV_INFO << "Edge: " << edge << std::endl;
    EV_INFO << "Distance to vertex A: " << distanceToVertexA << std::endl;
    EV_INFO << "Distance to vertex B: " << distanceToVertexB << std::endl;

    neighbouringCars[carAddress] = { omnetpp::simTime(), geohashLocation, speed, direction, locationOnRoadNetwork };
    neighbouringCarsByEdge.insert(NeighbouringCarByEdge(edge, carAddress));

    addRoute(carAddress, 128, carAddress, 1, omnetpp::simTime() + neighbouringCarValidityTime);

    EV_INFO << "Number of car neighbours: " << neighbouringCars.size() << std::endl;

    showRoutes();
    schedulePurgeNeighbouringCarsTimer();
}


void RoutingProtocolCar::processHelloHost(const inet::Ptr<HelloHost> &helloHost) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::processHelloHost");

    inet::Ipv6Address hostAddress = helloHost->getAddress();
    GeohashLocation geohashLocation(helloHost->getGeohash(), 12);

    EV_INFO << "Address: " << hostAddress.str() << std::endl;
    EV_INFO << "Geohash location: " << helloHost->getGeohash() << std::endl;
    EV_INFO << "                : " << geohashLocation.getGeohashString() << std::endl;

    neighbouringHosts[hostAddress] = { omnetpp::simTime(), geohashLocation };

    addRoute(hostAddress, 128, hostAddress, 1, omnetpp::simTime() + neighbouringHostValidityTime);

    EV_INFO << "Number of host neighbours: " << neighbouringHosts.size() << std::endl;

    showRoutes();

    schedulePurgeNeighbouringHostsTimer();
}


const inet::Ptr<Ping> RoutingProtocolCar::createPing(const inet::Ipv6Address &carAddress, Vertex vertexA, Vertex vertexB) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::createPing");

    const inet::Ptr<Ping> &ping = inet::makeShared<Ping>();

    EV_INFO << "Address: " << carAddress.str() << std::endl;
    EV_INFO << "Target vertex: " << vertexB << std::endl;
    EV_INFO << "Source vertex: " << vertexA << std::endl;

    int neighbouringCarsCount = getNeighbouringCarsOnEdgeCount();

    // Si no hay vecinos en la misma arista
    if (neighbouringCarsCount == 0)
        return nullptr;

    inet::Ipv6Address nextHopAddress = getRandomNeighbouringCarAddressAheadOnEdge(vertexB);

    EV_INFO << "Next hop: " << nextHopAddress.str() << std::endl;

    if (nextHopAddress.isUnspecified())
        return nullptr;

    ping->setAddress(carAddress);
    ping->setNextHopAddress(nextHopAddress);
    ping->setHopCount(0);
    ping->setNeighboursSum(neighbouringCarsCount);
    ping->setVertexA(vertexA);
    ping->setVertexB(vertexB);

    return ping;
}


void RoutingProtocolCar::sendPing(const inet::Ptr<Ping> &ping, const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::sendPing");

    inet::Packet *udpPacket = new inet::Packet("Ping");
    udpPacket->insertAtBack(ping);

    inet::Ptr<inet::UdpHeader> udpHeader = inet::makeShared<inet::UdpHeader>();
    udpHeader->setSourcePort(ROUTING_PROTOCOL_UDP_PORT);
    udpHeader->setDestinationPort(ROUTING_PROTOCOL_UDP_PORT);
    udpPacket->insertAtFront(udpHeader);

    inet::Ptr<inet::L3AddressReq> addresses = udpPacket->addTagIfAbsent<inet::L3AddressReq>();
    addresses->setSrcAddress(inet::L3Address(ping->getAddress()));
    addresses->setDestAddress(inet::L3Address(destAddress));

    inet::Ptr<inet::HopLimitReq> hopLimit = udpPacket->addTagIfAbsent<inet::HopLimitReq>();
    hopLimit->setHopLimit(255);

    inet::Ptr<inet::PacketProtocolTag> packetProtocol = udpPacket->addTagIfAbsent<inet::PacketProtocolTag>();
    packetProtocol->setProtocol(&inet::Protocol::manet);

    inet::Ptr<inet::DispatchProtocolReq> dispatchProtocol = udpPacket->addTagIfAbsent<inet::DispatchProtocolReq>();
    dispatchProtocol->setProtocol(&inet::Protocol::ipv6);

    sendUdpPacket(udpPacket);
}


void RoutingProtocolCar::processPing(const inet::Ptr<Ping> &ping) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::processPing");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();

    // Obtener datos del mensaje PING
    const inet::Ipv6Address &pingAddress = ping->getAddress();
    const inet::Ipv6Address &nextHopAddress = ping->getNextHopAddress();
    int hopCount = (int) ping->getHopCount();
    int neighboursSum = (int) ping->getNeighboursSum();
    Vertex pingVertexA = (Vertex) ping->getVertexA();
    Vertex pingVertexB = (Vertex) ping->getVertexB();
    Edge pingEdge = boost::edge(pingVertexA, pingVertexB, graph).first;

    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;

    EV_INFO << "Address: " << pingAddress.str() << std::endl;
    EV_INFO << "Next hop address: " << nextHopAddress.str() << std::endl;
    EV_INFO << "Hop count: " << hopCount << std::endl;
    EV_INFO << "Neighbours sum: " << neighboursSum << std::endl;
    EV_INFO << "Edge: " << pingEdge << std::endl;
    EV_INFO << "Target vertex: " << pingVertexB << std::endl;

    const inet::Ipv6Address &primaryUnicastAddress = addressCache->getUnicastAddress(PRIMARY_ADDRESS);
    const inet::Ipv6Address &primaryMulticastAddress = addressCache->getMulticastAddress(PRIMARY_ADDRESS);

    // Si el mensaje PING no corresponde a la arista donde circula el veh������culo, se descarta
    if (pingEdge != edge) {
        EV_INFO << "Otra arista" << std::endl;
        return;
    }

    hopCount++;
    neighboursSum += getNeighbouringCarsOnEdgeCount();

    // Se verifica si la dirección de destino es la dirección local
    if (interfaceTable->isLocalAddress(nextHopAddress)) {
        // Si se encuentra en el vértice de destino, se responde con un mensaje PONG
        if (inVertexProximityRadius(locationOnRoadNetwork, pingVertexB, graph)) {
            EV_INFO << "PONG" << std::endl;

            // Responder con mensaje PONG
            int neighboursAverage = (int) ((double) neighboursSum / hopCount);
            inet::Ipv6Address newNextHopAddress = getNeighbouringCarAddressOnEdgeClosestToVertex(pingVertexA);

            const inet::Ptr<Pong> pong = createPong(pingAddress, true, SIMTIME_ZERO, neighboursAverage, pingVertexA, pingVertexB);
            sendPong(pong, newNextHopAddress);

            // Agregar la arista a la lista de aristas activas
            activeEdges[edge] = omnetpp::simTime();
            schedulePurgeActiveEdgesTimer();

            // Enviar mensaje de estatus de arista activa

        // Si no se encuentra en el v������rtice de destino
        } else {
            inet::Ipv6Address newNextHopAddress = getRandomNeighbouringCarAddressAheadOnEdge(pingVertexB);
            neighboursSum += getNeighbouringCarsOnEdgeCount();

            ping->setNextHopAddress(newNextHopAddress);
            ping->setHopCount(hopCount);
            ping->setNeighboursSum(neighboursSum);

            // Se env������a a la direcci������n multicast
            ASSERT(primaryUnicastAddress.matches(pingAddress, 64));
            sendPing(ping, primaryMulticastAddress);
        }
    }
}


const inet::Ptr<Pong> RoutingProtocolCar::createPong(const inet::Ipv6Address &destAddress, bool N, omnetpp::simtime_t validityTime, int neighboursAverage, Vertex vertexA, Vertex vertexB) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::createPong");

    const inet::Ptr<Pong> &pong = inet::makeShared<Pong>();

    EV_INFO << "Destination address: " << destAddress.str() << std::endl;
    EV_INFO << "N: " << N << std::endl;
    EV_INFO << "Validity time: " << validityTime << std::endl;
    EV_INFO << "Neighbours average: " << neighboursAverage << std::endl;
    EV_INFO << "Target vertex: " << vertexA << std::endl;
    EV_INFO << "Source vertex: " << vertexB << std::endl;

    pong->setN(N);
    pong->setDestAddress(destAddress);
    pong->setValidityTime(validityTime);
    pong->setHopCount(0);
    pong->setNeighboursAverage(neighboursAverage);
    pong->setVertexA(vertexA);
    pong->setVertexB(vertexB);

    return pong;
}


void RoutingProtocolCar::sendPong(const inet::Ptr<Pong> &pong, const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::sendPong");

    inet::Packet *udpPacket = new inet::Packet("Pong");
    udpPacket->insertAtBack(pong);

    inet::Ptr<inet::UdpHeader> udpHeader = inet::makeShared<inet::UdpHeader>();
    udpHeader->setSourcePort(ROUTING_PROTOCOL_UDP_PORT);
    udpHeader->setDestinationPort(ROUTING_PROTOCOL_UDP_PORT);
    udpPacket->insertAtFront(udpHeader);

    inet::Ptr<inet::L3AddressReq> addresses = udpPacket->addTagIfAbsent<inet::L3AddressReq>();
    addresses->setSrcAddress(inet::L3Address(addressCache->getUnicastAddress(PRIMARY_ADDRESS)));
    addresses->setDestAddress(inet::L3Address(destAddress));

    inet::Ptr<inet::HopLimitReq> hopLimit = udpPacket->addTagIfAbsent<inet::HopLimitReq>();
    hopLimit->setHopLimit(255);

    inet::Ptr<inet::PacketProtocolTag> packetProtocol = udpPacket->addTagIfAbsent<inet::PacketProtocolTag>();
    packetProtocol->setProtocol(&inet::Protocol::manet);

    inet::Ptr<inet::DispatchProtocolReq> dispatchProtocol = udpPacket->addTagIfAbsent<inet::DispatchProtocolReq>();
    dispatchProtocol->setProtocol(&inet::Protocol::ipv6);

    sendUdpPacket(udpPacket);
}


void RoutingProtocolCar::processPong(const inet::Ptr<Pong> &pong) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::processPong");

    const inet::Ipv6Address &destAddress = pong->getDestAddress();
    bool N = pong->getN();
    omnetpp::simtime_t validityTime = pong->getValidityTime();
    int hopCount = pong->getHopCount();
    int neighboursAverage = pong->getNeighboursAverage();
    Vertex vertexA = (Vertex) pong->getVertexA();
    Vertex vertexB = (Vertex) pong->getVertexB();

    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    Edge edge = boost::edge(vertexA, vertexB, graph).first;

    EV_INFO << "Destination address: " << destAddress.str() << std::endl;
    EV_INFO << "N: " << N << std::endl;
    EV_INFO << "Hop count: " << hopCount << std::endl;
    EV_INFO << "Neighbours average: " << neighboursAverage << std::endl;
    EV_INFO << "Edge: " << edge;
    EV_INFO << "Target vertex: " << vertexA << std::endl;

    hopCount++;

    // Si lleg������ al veh������culo que origin������ el mensaje PING
    if (interfaceTable->isLocalAddress(destAddress)) {
        // Si ya se termin������ el tiempo de espera, se ignora el mensaje
        if (!pongTimeoutTimer->isScheduled())
            return;

        // Si todav������a no terminaba el tiempo de espera, se cancela
        cancelEvent(pongTimeoutTimer);

        // Agregar la arista a la lista de aristas activas
        activeEdges[edge] = omnetpp::simTime();
        schedulePurgeActiveEdgesTimer();

        // Enviar mensaje de estatus de la arista activa

    } else {
        inet::Ipv6Address newNextHopAddress;

        // Si el destino est������ en el directorio de vecinos, se selecciona como siguiente salto
        if (neighbouringCars.find(destAddress) != neighbouringCars.end())
            newNextHopAddress = destAddress;

        // Si no, se selecciona un nuevo siguiente salto
        else
            newNextHopAddress = getNeighbouringCarAddressOnEdgeClosestToVertex(vertexA);

        pong->setHopCount(hopCount);

        sendPong(pong, newNextHopAddress);
    }
}


void RoutingProtocolCar::show() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::show");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertex1;
    const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertex2;

    EV_INFO << "Address: " << addressCache->getUnicastAddress(PRIMARY_ADDRESS) << std::endl;
    EV_INFO << "Edge: " << edge << std::endl;
    EV_INFO << "Distance to vertex A: " << distanceToVertexA << std::endl;
    EV_INFO << "Distance to vertex B: " << distanceToVertexB << std::endl;
}


void RoutingProtocolCar::removeOldNeighbouringCars(omnetpp::simtime_t time) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::removeOldNeighbouringCars");

    NeighbouringCarsIterator it = neighbouringCars.begin();
    while (it != neighbouringCars.end())
        if (it->second.lastUpdateTime <= time) {
            //purgeNextHopRoutes(it->first); // Se elimina las rutas
            neighbouringCars.erase(it++); // Se elimina del directorio de veh������culos vecinos

            inet::Ipv6Address neighbouringCarAddress = it->first;
            Edge &edge = it->second.locationOnRoadNetwork.edge;
            NeighbouringCarsByEdgeIterator neighbouringCarIt = neighbouringCarsByEdge.lower_bound(edge);
            NeighbouringCarsByEdgeIterator neighbouringCarEndIt = neighbouringCarsByEdge.upper_bound(edge);
            while (neighbouringCarIt != neighbouringCarEndIt) {
                if (neighbouringCarIt->second == neighbouringCarAddress)
                    neighbouringCarsByEdge.erase(neighbouringCarIt);
                neighbouringCarIt++;
            }

        } else
            it++;
}


inet::Ipv6Address RoutingProtocolCar::getRandomNeighbouringCarAddressAheadOnEdge(Vertex targetVertex) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::getRandomNeighbouringCarOnEdgeAddress");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);

    double distanceToTargetVertex = getDistanceToVertex(mobility->getLocationOnRoadNetwork(), targetVertex, graph);
    ASSERT(distanceToTargetVertex != std::numeric_limits<double>::infinity());

    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    std::vector<inet::Ipv6Address> addresses;

    // Se agregan las direcciones de los veh������culos que est������n adelante en la misma arista
    NeighbouringCarsByEdgeConstIterator neighbouringCarIt = neighbouringCarsByEdge.lower_bound(edge);
    NeighbouringCarsByEdgeConstIterator neighbouringCarEndIt = neighbouringCarsByEdge.upper_bound(edge);
    while (neighbouringCarIt != neighbouringCarEndIt) {
        inet::Ipv6Address neighbouringCarAddress = neighbouringCarIt->second;
        NeighbouringCarsConstIterator neighbouringCarIt = neighbouringCars.find(neighbouringCarAddress);
/*        if (neighbouringCarIt != neighbouringCars.end()) {
            LocationOnRoadNetwork &locationOnRoadNetwork = neighbouringCarIt->second.locationOnRoadNetwork;
            double neighbouringCarDistanceToTargetVertex = getDistanceToVertex(targetVertex, locationOnRoadNetwork, graph);
            ASSERT(neighbouringCarDistanceToTargetVertex != std::numeric_limits<double>::infinity());

            if (neighbouringCarDistanceToTargetVertex < distanceToTargetVertex)
                addresses.push_back(neighbouringCarAddress);
        }*/

    }

    // Se devuelve una direcci������n aleatoria
    if (addresses.size() > 0)
        return addresses[rand() % addresses.size()];

    else
        return inet::Ipv6Address::UNSPECIFIED_ADDRESS;
}


inet::Ipv6Address RoutingProtocolCar::getNeighbouringCarAddressOnEdgeClosestToVertex(Vertex targetVertex) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::getNeighbouringCarAddressOnEdgeClosestToVertex");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);
    const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertex1;
    const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertex1;

    double minDistanceToTargetVertex = std::numeric_limits<double>::infinity();
    inet::Ipv6Address address;

    NeighbouringCarsByEdgeIterator neighbouringCarIt = neighbouringCarsByEdge.lower_bound(edge);
    NeighbouringCarsByEdgeIterator neighbouringCarEndIt = neighbouringCarsByEdge.upper_bound(edge);
    while (neighbouringCarIt != neighbouringCarEndIt) {
        const inet::Ipv6Address neighbouringCarAddress = neighbouringCarIt->second;
        const NeighbouringCarEntry &neighbouringCar = neighbouringCars[neighbouringCarAddress];
        const LocationOnRoadNetwork &locationOnRoadNetwork = neighbouringCar.locationOnRoadNetwork;
        double neighbouringCarDistanceToTargetVertex = getDistanceToVertex(locationOnRoadNetwork, targetVertex, graph);
        ASSERT(neighbouringCarDistanceToTargetVertex != std::numeric_limits<double>::infinity());

        if (neighbouringCarDistanceToTargetVertex < minDistanceToTargetVertex)
            minDistanceToTargetVertex = neighbouringCarDistanceToTargetVertex;
            address = neighbouringCarAddress;
    }

    return address;
}


int RoutingProtocolCar::getNeighbouringCarsOnEdgeCount() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::getNeighbouringCarsOnEdgeCount");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    int n = 0;

    show();
    showNeighbouringCars();

    for (const NeighbouringCar &neighbouringCar: neighbouringCars)
        if (neighbouringCar.second.locationOnRoadNetwork.edge == edge)
            n++;

    return n;
}


void RoutingProtocolCar::removeOldNeighbouringHosts(omnetpp::simtime_t time) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::removeOldNeighbouringHosts");

    NeighbouringHostsIterator it = neighbouringHosts.begin();
    while (it != neighbouringHosts.end())
        if (it->second.lastUpdateTime <= time) {
            //purgeNextHopRoutes(it->first);
            neighbouringHosts.erase(it++);

        } else
            it++;
}


omnetpp::simtime_t RoutingProtocolCar::getOldestNeighbouringHostTime() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::getOldestNeighbouringHostTime");

    omnetpp::simtime_t oldestUpdateTime = omnetpp::SimTime::getMaxTime();

    for (const NeighbouringHost &neighbouringHost: neighbouringHosts) {
        const omnetpp::simtime_t &time = neighbouringHost.second.lastUpdateTime;

        if (time < oldestUpdateTime)
            oldestUpdateTime = time;
    }

    return oldestUpdateTime;
}


omnetpp::simtime_t RoutingProtocolCar::getNextNeighbouringHostExpirationTime() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::getNextNeighbouringHostExpirationTime");

    omnetpp::simtime_t nextExpirationTime = getOldestNeighbouringHostTime();

    if (nextExpirationTime == omnetpp::SimTime::getMaxTime())
        return nextExpirationTime;

    else
        return nextExpirationTime + neighbouringHostValidityTime;
}


void RoutingProtocolCar::purgeNeighbouringHosts() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::purgeNeighbouringHosts");

    removeOldNeighbouringHosts(omnetpp::simTime() - neighbouringHostValidityTime);
    removeOldRoutes(omnetpp::simTime());
}


void RoutingProtocolCar::removeOldActiveEdges(omnetpp::simtime_t time) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::removeOldActiveEdges");

    ActiveEdgesIterator it = activeEdges.begin();
    while(it != activeEdges.end())
        if (it->second <= time)
            activeEdges.erase(it++);

        else
            it++;
}


omnetpp::simtime_t RoutingProtocolCar::getOldestActiveEdgeTime() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::getOldestActiveEdgeTime");

    omnetpp::simtime_t oldestUpdateTime = omnetpp::SimTime::getMaxTime();

    for (const ActiveEdge &activeEdge: activeEdges) {
        const omnetpp::simtime_t &time = activeEdge.second;

        if (time < oldestUpdateTime)
            oldestUpdateTime = time;
    }

    return oldestUpdateTime;
}


omnetpp::simtime_t RoutingProtocolCar::getNextActiveEdgeExpirationTime() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::getNextActiveEdgeExpirationTime");

    omnetpp::simtime_t nextExpirationTime = getOldestActiveEdgeTime();

    if (nextExpirationTime == omnetpp::SimTime::getMaxTime())
        return nextExpirationTime;

    else
        return nextExpirationTime + activeEdgeValidityTime;
}


void RoutingProtocolCar::purgeActiveEdges() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::purgeActiveEdges");

    removeOldActiveEdges(omnetpp::simTime() - activeEdgeValidityTime);
}



void RoutingProtocolCar::removeOldInactiveEdges(omnetpp::simtime_t time) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::removeOldInactiveEdges");

    InactiveEdgesIterator it = inactiveEdges.begin();
    while(it != inactiveEdges.end())
        if (it->second <= time)
            inactiveEdges.erase(it++);

        else
            it++;
}


omnetpp::simtime_t RoutingProtocolCar::getOldestInactiveEdgeTime() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::getOldestInactiveEdgeTime");

    omnetpp::simtime_t oldestUpdateTime = omnetpp::SimTime::getMaxTime();

    for (const InactiveEdge &inactiveEdge: inactiveEdges) {
        const omnetpp::simtime_t &time = inactiveEdge.second;

        if (time < oldestUpdateTime)
            oldestUpdateTime = time;
    }

    return oldestUpdateTime;
}


omnetpp::simtime_t RoutingProtocolCar::getNextInactiveEdgeExpirationTime() const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::getNextInactiveEdgeExpirationTime");

    omnetpp::simtime_t nextExpirationTime = getOldestInactiveEdgeTime();

    if (nextExpirationTime == omnetpp::SimTime::getMaxTime())
        return nextExpirationTime;

    else
        return nextExpirationTime + inactiveEdgeValidityTime;
}


void RoutingProtocolCar::purgeInactiveEdges() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::inactiveEdgeValidityTime");

    removeOldInactiveEdges(omnetpp::simTime() - activeEdgeValidityTime);
}


inet::INetfilter::IHook::Result RoutingProtocolCar::routeDatagram(inet::Packet *datagram, const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::routeDatagram");

    /*********************************************************************************
     * Validaci��n de la cabecera de opciones de salto por salto
     *********************************************************************************/
    if (!validateHopByHopOptionsHeader(datagram))
        return inet::INetfilter::IHook::Result::DROP;

    /*********************************************************************************
     * Verificar si ya existe una ruta
     *********************************************************************************/
    const inet::Ipv6Route *route = routingTable->doLongestPrefixMatch(destAddress);
    if (route != nullptr) {
        if (omnetpp::simTime() < route->getExpiryTime())
            return inet::INetfilter::IHook::ACCEPT;

        else
            routingTable->deleteRoute(const_cast<inet::Ipv6Route *>(route));
    }

    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const Graph &graph = roadNetwork->getGraph();

    // Se obtiene la ubicaci��n vial del veh��culo
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;

    /*********************************************************************************
     * Obtener vértices visitados
     *********************************************************************************/
    const TlvRoutingOption *routingOption = findTlvOption<TlvRoutingOption>(datagram);
    ASSERT(routingOption != nullptr);
    int numVisitedVertices = routingOption->getVisitedVerticesArraySize();
    VertexSet visitedVertices;
    for (int i = 0; i < numVisitedVertices; i++)
        visitedVertices.insert(routingOption->getVisitedVertices(i));

    /*********************************************************************************
     * Obtener aristas inactivas
     *********************************************************************************/
    int numActiveEdges = this->activeEdges.size();
    ActiveEdgesConstIterator activeEdgeIt = this->activeEdges.begin();
    EdgeSet activeEdges;
    for (int i = 0; i < numActiveEdges && activeEdgeIt != this->activeEdges.end(); i++, activeEdgeIt++)
        activeEdges.insert(activeEdgeIt->first);

    /*********************************************************************************
     * Calcular ruta más corta
     *********************************************************************************/
    ShortestPath shortestPath;
    shortestPath.computeShortestPath(edge, graph, visitedVertices, activeEdges);

    /*********************************************************************************
     * Obtener v��rtice destino local
     *********************************************************************************/
    Vertex destVertex = getLocalDestVertex(datagram, shortestPath);

    // Se obtiene la ruta m��s corta al v��rtice de destino
    VertexVector shortestPathToDestVertex = shortestPath.getShortestPathToVertex(destVertex, graph);

    // Se busca el siguiente salto
    inet::Ipv6Address nextHopAddress = findNextHop(shortestPathToDestVertex, shortestPath);

    if (!nextHopAddress.isUnspecified()) {
        addRoute(destAddress, 128, nextHopAddress, 1, omnetpp::simTime() + routeValidityTime);
        return inet::INetfilter::IHook::Result::ACCEPT;
    }

    return inet::INetfilter::IHook::Result::DROP;
}


bool RoutingProtocolCar::validateHopByHopOptionsHeader(inet::Packet *datagram) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::validateHopByHopOptionsHeader");

    // Se obtiene la ubicaci��n de destino del datagrama
    const TlvDestGeohashLocationOption *destGeohashLocationOption = findTlvOption<TlvDestGeohashLocationOption>(datagram);

    if (destGeohashLocationOption == nullptr)
        return false;

    GeohashLocation destGeohashLocation(destGeohashLocationOption->getGeohash(), 12);

    // Si el destino est�� en la misma regi��n Geohash
    const GeohashLocation &geohashRegion = mobility->getRoadNetwork()->getGeohashRegion();
    if (geohashRegion.contains(destGeohashLocation)) {
        // Si el datagrama no incluye la opci��n de ubicaci��n vial del destino, se agrega
        const TlvDestLocationOnRoadNetworkOption *destLocationOnRoadNetworkOption = findTlvOption<TlvDestLocationOnRoadNetworkOption>(datagram);
        if (destLocationOnRoadNetworkOption == nullptr) {
            setTlvDestLocationOnRoadNetworkOption(datagram, destGeohashLocation);
        }
    }

    // Si el datagrama no incluye la opci��n de enrutamiento, se agrega
    const TlvRoutingOption *routingOption = findTlvOption<TlvRoutingOption>(datagram);
    if (routingOption == nullptr) {
        setTlvRoutingOption(datagram);
    }

    return true;
}


Vertex RoutingProtocolCar::getLocalDestVertex(inet::Packet *datagram, const ShortestPath &shortestPath) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::getLocalDestVertex");

    const TlvDestGeohashLocationOption *destGeohashLocationOption = findTlvOption<TlvDestGeohashLocationOption>(datagram);
    ASSERT(destGeohashLocationOption != nullptr);

    const TlvRoutingOption *routingOption = findTlvOption<TlvRoutingOption>(datagram);
    ASSERT(routingOption != nullptr);

    GeohashLocation destGeohashLocation(destGeohashLocationOption->getGeohash(), 12);
    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const GeohashLocation &geohashRegion = roadNetwork->getGeohashRegion();

    Vertex destVertex;

    // Si el destino se encuentra en la misma subred
    if (geohashRegion.contains(destGeohashLocation)) {
        const TlvDestLocationOnRoadNetworkOption *locationOnRoadNetworkOption = findTlvOption<TlvDestLocationOnRoadNetworkOption>(datagram);
        ASSERT(locationOnRoadNetworkOption != nullptr);

        Vertex vertexA = (Vertex) locationOnRoadNetworkOption->getVertexA();
        Vertex vertexB = (Vertex) locationOnRoadNetworkOption->getVertexB();
        double routeDistanceA = shortestPath.getRouteDistance(vertexA);
        double routeDistanceB = shortestPath.getRouteDistance(vertexB);

        // Se selecciona como v������rtice destino local el v������rtice con menor distancia de ruta
        if (routeDistanceA < routeDistanceB)
            destVertex = vertexA;

        else
            destVertex = vertexB;

    // Si el destino se encuentra en una subred distinta
    } else {
        const GeographicLib::GeoCoords &destLocation = destGeohashLocation.getLocation();
        const Bounds &localRegionBounds = geohashRegion.getBounds();

        VertexVector gatewayVertices;

        // Si el v��rtice se encuentra al norte de la regi��n local
        if (destLocation.Latitude() > localRegionBounds.getNorth()) {
            // Se obtienen los v��rtices gateway del norte
            const VertexVector &northGatewayVertices = roadNetwork->getGatewayVertices(GeohashLocation::Direction::NORTH);
            gatewayVertices.insert(gatewayVertices.end(), northGatewayVertices.begin(), northGatewayVertices.end());

        // Si el v��rtice se encuentra al sur de la regi��n local
        } else if (destLocation.Longitude() < localRegionBounds.getSouth()) {
            // Se obtienen los v��rtices gateway del sur
            const VertexVector &southGatewayVertices = roadNetwork->getGatewayVertices(GeohashLocation::Direction::SOUTH);
            gatewayVertices.insert(gatewayVertices.end(), southGatewayVertices .begin(), southGatewayVertices .end());
        }

        // Si el v��rtice se encuentra al este de la regi��n local
        if (destLocation.Longitude() > localRegionBounds.getEast()) {
            // Se obtienen los v��rtices gateway del este
            const VertexVector &eastGatewayVertices = roadNetwork->getGatewayVertices(GeohashLocation::Direction::EAST);
            gatewayVertices.insert(gatewayVertices.end(), eastGatewayVertices.begin(), eastGatewayVertices.end());

        // Si el v��rtice se encuentra al oeste de la regi��n local
        } else if (destLocation.Latitude() < localRegionBounds.getWest()) {
            // Se obtienen los v��rtices gateway del oeste
            const VertexVector &westGatewayVertices = roadNetwork->getGatewayVertices(GeohashLocation::Direction::WEST);
            gatewayVertices.insert(gatewayVertices.end(), westGatewayVertices.begin(), westGatewayVertices.end());
        }

        // Se busca el v��rtice gateway cuya distancia sea m��nima
        double minRouteDistance = std::numeric_limits<double>::infinity();
        VertexVectorIterator it = gatewayVertices.begin();
        while (it != gatewayVertices.end()) {
            double routeDistance = shortestPath.getRouteDistance(*it);
            if (routeDistance < minRouteDistance)
                destVertex = *it;

            it++;
        }
    }

    return destVertex;
}


VertexVector RoutingProtocolCar::getUnavailableVertices(TlvRoutingOption *routingOption) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::getUnavailableVertices");

    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const Graph &graph = roadNetwork->getGraph();
    int numVisitedVertices = routingOption->getVisitedVerticesArraySize(); // N������mero de v������rtices que el paquete ha visitado
    int numInactiveEdges = inactiveEdges.size(); // N������mero de aristas inactivas
    VertexVector unavailableVertices(numVisitedVertices + numInactiveEdges);

    // Se agregan los v������rtices que el paquete ha visitado
    VertexVectorIterator it = unavailableVertices.begin();
    for (int i = 0; i < numVisitedVertices; i++) {
        *it = (Vertex) routingOption->getVisitedVertices(i);
        it++;
    }

    // Se agregan los v������rtices de las aristas inactivas
    InactiveEdgesConstIterator inactiveEdgesIt = inactiveEdges.begin();
    while (inactiveEdgesIt != inactiveEdges.end()) {
        *it = boost::target(inactiveEdgesIt->first, graph);
        inactiveEdgesIt++;
        it++;
    }

    return unavailableVertices;
}


Vertex RoutingProtocolCar::getDestVertex(const GeohashLocation &destGeohashLocation, Edge destEdge, const ShortestPath &shortestPath) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::destLocationOnRoadNetwork");

    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const Graph &graph = roadNetwork->getGraph();
    Vertex destVertexA = boost::source(destEdge, graph);
    Vertex destVertexB = boost::target(destEdge, graph);
    Vertex destVertex;

    // Si el destino est������ en otra subred
    if (!roadNetwork->getGeohashRegion().contains(destGeohashLocation)) {
        const GeohashLocation &geohashLocation = roadNetwork->getGeohashRegion();
        const GeographicLib::GeoCoords &A = mobility->getLocation();
        const GeographicLib::GeoCoords &B = destGeohashLocation.getLocation();

        // Se determina en qu������ direcci������n en latitud se encuentra la regi������n de destino
        GeohashLocation::Direction directionLat = GeohashLocation::Direction::NONE;
        if (B.Latitude() < A.Latitude())
            directionLat = GeohashLocation::Direction::SOUTH;
        else if (B.Latitude() > A.Longitude())
            directionLat = GeohashLocation::Direction::NORTH;

        // Se determina en qu������ direcci������n en longitud se encuentra la regi������n de destino
        GeohashLocation::Direction directionLon = GeohashLocation::Direction::NONE;
        if (B.Longitude() < A.Longitude())
            directionLon = GeohashLocation::Direction::WEST;
        else if (B.Longitude() > A.Longitude())
            directionLon = GeohashLocation::Direction::EAST;

        // Se obtienen los v������rtices gateway posibles
        VertexVector gatewayVertices;
        if (directionLat != GeohashLocation::Direction::NONE) {
            const VertexVector &gatewayVerticesLat = roadNetwork->getGatewayVertices(directionLat);
            gatewayVertices.insert(gatewayVertices.end(), gatewayVerticesLat.begin(), gatewayVerticesLat.end());
        }
        if (directionLat != GeohashLocation::Direction::NONE) {
            const VertexVector &gatewayVerticesLon = roadNetwork->getGatewayVertices(directionLon);
            gatewayVertices.insert(gatewayVertices.end(), gatewayVerticesLon.begin(), gatewayVerticesLon.end());
        }

        // Buscar el v������rtice gateway con distancia m������nima
        double minDistance = std::numeric_limits<double>::infinity();
        for (const Vertex &vertex: gatewayVertices) {
            double distance = shortestPath.getRouteDistance(vertex);
            if (distance < minDistance) {
                minDistance = distance;
                destVertex = vertex;
            }
        }

    // Si el destino est������ en la misma subred
    } else {
        // Se obtiene la ubicaci������n vial del destino
        Vertex destVertexA = boost::source(destEdge, graph);
        Vertex destVertexB = boost::target(destEdge, graph);

        double distanceToDestVertexA = shortestPath.getRouteDistance(destVertexA);
        double distanceToDestVertexB = shortestPath.getRouteDistance(destVertexB);

        destVertex = distanceToDestVertexA < distanceToDestVertexB ? destVertexA : destVertexB;
    }

    return destVertex;
}


EdgeVector RoutingProtocolCar::getReachableEdges(const VertexVector &shortestPathToDestVertex, const ShortestPath &shortestPath) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::getReachableEdges");

    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const Graph &graph = roadNetwork->getGraph();

    EdgeVector reachableEdges;
    VertexVectorConstIterator vertexIt = shortestPathToDestVertex.begin();
    VertexVectorConstIterator vertexEndIt = shortestPathToDestVertex.end();

    // Se agrega la primera arista
    Vertex vertexA = *vertexIt++;
    Vertex vertexB = *vertexIt++;
    Edge edge = boost::edge(vertexA, vertexB, graph).first;
    reachableEdges.push_back(edge);

    if (vertexIt != vertexEndIt) {
        // Si el veh��culo se encuentra en la siguiente arista
        if (mobility->isAtVertex(vertexB)) {
            vertexA = vertexB;
            vertexB = *vertexIt++;
            edge = boost::edge(vertexA, vertexB, graph).first;
            reachableEdges.push_back(edge);
        }

        while (vertexIt != vertexEndIt) {
            vertexA = vertexB;
            vertexB = *vertexIt++;

            double distance = shortestPath.getRouteDistance(vertexB);

            // Si la distancia hacia el siguiente v��rtice es aceptable
            if (distance < 30) {
                // Se agrega la siguiente arista
                edge = boost::edge(vertexA, vertexB, graph).first;
                reachableEdges.push_back(edge);

            } else
                break;
        }
    }

    return reachableEdges;
}


inet::Ipv6Address  RoutingProtocolCar::findNextHop(const VertexVector &shortestPathToDestVertex, const ShortestPath &shortestPath) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::findNextHop");

    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const Graph &graph = roadNetwork->getGraph();

    EdgeVector reachableEdges = getReachableEdges(shortestPathToDestVertex, shortestPath);
    ASSERT(reachableEdges.size() > 0);
    EdgeVectorIterator edgeIt = reachableEdges.end() - 1;
    Edge edge;
    Vertex vertexB;
    inet::Ipv6Address nextHopAddress;

    // Si hay ������nicamente una arista alcanzable
    if (reachableEdges.size() == 1) {
        edge = reachableEdges[0];
        vertexB = boost::target(edge, graph);
        GeohashLocation::Direction gatewayType = graph[vertexB].gatewayType;

        // Si el v������rtice de destino de la arista es un gateway
        if (gatewayType != GeohashLocation::Direction::NONE) {
            const GeohashLocation &geohashRegion = roadNetwork->getGeohashRegion();
            GeohashLocation neighbourGeohashRegion;
            geohashRegion.getNeighbour(gatewayType, neighbourGeohashRegion);
            nextHopAddress = findNeighbourInNeighbourinRegion(neighbourGeohashRegion);

            if (!nextHopAddress.isUnspecified())
                return nextHopAddress;
        }
    }

    // Mientras no se encuentre un veh������culo vecino en la arista
    while (neighbouringCarsByEdge.find(*edgeIt) == neighbouringCarsByEdge.end())
        // Se obtiene la siguiente arista
        edgeIt--;

    edge = *edgeIt; // Primera arista por donde circulan veh������culos vecinos
    vertexB = boost::target(edge, graph); // V������rtice de destino de la arista
    double minDistanceToVertexB = std::numeric_limits<double>::infinity(); // Distancia m������nima delos vecinos al v������rtice de destino de la arista
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    double distanceToVertexB = getDistanceToVertex(locationOnRoadNetwork, vertexB, graph); // Distancia del veh������culo al v������rtice B (puede ser infinito si el v������rtice est������ en otra arista)
    nextHopAddress = inet::Ipv6Address::UNSPECIFIED_ADDRESS; // Direcci������n del vecino m������s cercano al v������rtice de destino de la arista
    NeighbouringCarsByEdgeConstIterator neighbouringCarIt = neighbouringCarsByEdge.lower_bound(edge);
    NeighbouringCarsByEdgeConstIterator neighbouringCarEndIt = neighbouringCarsByEdge.upper_bound(edge);

    while (neighbouringCarIt != neighbouringCarEndIt) {
        const inet::Ipv6Address &neighbouringCarAddress = neighbouringCarIt->second;
        const NeighbouringCarEntry &neighbouringCar = neighbouringCars.at(neighbouringCarAddress);
        const LocationOnRoadNetwork &locationOnRoadNetwork = neighbouringCar.locationOnRoadNetwork;
        double neighbouringCarDistanceToVertexB = getDistanceToVertex(locationOnRoadNetwork, vertexB, graph);

        // Si se encontr������ un vecino que est������ a una distancia menor del v������rtice de destino
        if (neighbouringCarDistanceToVertexB < minDistanceToVertexB) {
            // Si se encuentra en la misma arista del veh������culo
            if (edge == locationOnRoadNetwork.edge) {
                // Si se encuentra a una distancia menor del v������rtice de destino, se considera como siguiente salto
                if (neighbouringCarDistanceToVertexB < distanceToVertexB) {
                    minDistanceToVertexB = neighbouringCarDistanceToVertexB;
                    nextHopAddress = neighbouringCarAddress;
                }

            // Si se encuentra en otra arista, se considera como siguiente salto
            } else {
                minDistanceToVertexB = neighbouringCarDistanceToVertexB;
                nextHopAddress = neighbouringCarAddress;
            }
        }

        neighbouringCarIt++;
    }

    return nextHopAddress;
}


inet::Ipv6Address RoutingProtocolCar::findNeighbourInNeighbourinRegion(const GeohashLocation &neighbouringGeohashRegion) const {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::findNeighbourInNeighbourinRegion");

    NeighbouringCarsConstIterator it = neighbouringCars.begin();
    NeighbouringCarsConstIterator endIt = neighbouringCars.end();

    while (it != endIt) {
        const GeohashLocation &neighbouringCarGeohashLocation = it->second.geohashLocation;
        if (neighbouringGeohashRegion.contains(neighbouringCarGeohashLocation))
            return it->first;
    }

    return inet::Ipv6Address::UNSPECIFIED_ADDRESS;
}



inet::INetfilter::IHook::Result RoutingProtocolCar::datagramPreRoutingHook(inet::Packet *datagram) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::datagramPreRoutingHook");
    EV_INFO << "Datagram: " << datagram->str() << std::endl;

    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader = inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address srcAddress = networkHeader->getSourceAddress().toIpv6();
    inet::Ipv6Address destAddress = networkHeader->getDestinationAddress().toIpv6();

    EV_INFO << "Source address: " << srcAddress.str() << std::endl;
    EV_INFO << "Destination address: " << destAddress.str() << std::endl;

    if (interfaceTable->isLocalAddress(inet::L3Address(destAddress)) || !destAddress.isSiteLocal() || destAddress.isMulticast())
        return inet::INetfilter::IHook::ACCEPT;

    //return routeDatagram(datagram, destAddress);
    return inet::INetfilter::IHook::ACCEPT;
}


inet::INetfilter::IHook::Result RoutingProtocolCar::datagramLocalOutHook(inet::Packet *datagram) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::datagramLocalOutHook");
    EV_INFO << "Datagram: " << datagram->str() << std::endl;

    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader = inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address srcAddress = networkHeader->getSourceAddress().toIpv6();
    inet::Ipv6Address destAddress = networkHeader->getDestinationAddress().toIpv6();

    EV_INFO << "Source address: " << srcAddress.str() << std::endl;
    EV_INFO << "Destination address: " << destAddress.str() << std::endl;

    if (interfaceTable->isLocalAddress(inet::L3Address(destAddress)) || !destAddress.isSiteLocal() || destAddress.isMulticast())
        return inet::INetfilter::IHook::ACCEPT;

    return inet::INetfilter::IHook::ACCEPT;
}


void RoutingProtocolCar::handleStartOperation(inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::handleStartOperation");

    RoutingProtocolBase::handleStartOperation(operation);
    scheduleHelloCarTimer();
}


void RoutingProtocolCar::handleStopOperation(inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::handleStopOperation");

    RoutingProtocolBase::handleStopOperation(operation);
    cancelAndDelete(helloCarTimer);
    cancelAndDelete(pongTimeoutTimer);
    cancelAndDelete(purgeNeighbouringHostsTimer);
    cancelAndDelete(purgeActiveEdgesTimer);
    neighbouringHosts.clear();
    activeEdges.clear();
}


void RoutingProtocolCar::handleCrashOperation(inet::LifecycleOperation *operation)  {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("RoutingProtocolCar::handleCrashOperation");

    RoutingProtocolBase::handleCrashOperation(operation);
    cancelAndDelete(helloCarTimer);
    cancelAndDelete(pongTimeoutTimer);
    cancelAndDelete(purgeNeighbouringHostsTimer);
    cancelAndDelete(purgeActiveEdgesTimer);
    neighbouringHosts.clear();
    activeEdges.clear();
}


void RoutingProtocolCar::receiveSignal(omnetpp::cComponent *source, omnetpp::simsignal_t signalID, omnetpp::cObject *obj, cObject *details) {

}
