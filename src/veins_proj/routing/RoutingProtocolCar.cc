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

/*!
 * @file RoutingProtocolCar.cc
 * @author Adrián Juárez Monroy
 */

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

/*
 * Interfaz del módulo.
 */

//! Inicialización.
void RoutingProtocolCar::initialize(int stage) {
    RoutingProtocolBase::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        /*
         * Contexto.
         */
        mobility = omnetpp::check_and_cast<CarMobility*>(
                host->getSubmodule("mobility"));
        if (!mobility)
            throw omnetpp::cRuntimeError("No mobility module found");

        /*
         * Mensajes propios.
         */
        helloCarTimer = new omnetpp::cMessage("helloCarTimer");
        purgeNeighbouringHostsTimer = new omnetpp::cMessage(
                "purgeNeighbouringHostsTimer");
        purgeActiveEdgesTimer = new omnetpp::cMessage("purgeActiveEdgesTimer");
        purgePendingPongsTimer = new omnetpp::cMessage(
                "purgePendingPongsTimer");
    }
}

/*
 * Manejo de mensajes.
 */

//! Manejo de mensajes propios.
/*!
 * @param message [in] Mensaje a procesar.
 */
void RoutingProtocolCar::processSelfMessage(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processSelfMessage");

    if (message == helloCarTimer)
        processHelloCarTimer();

    else if (message == purgeNeighbouringHostsTimer)
        processPurgeNeighbouringHostsTimer();

    else if (message == purgeActiveEdgesTimer)
        processPurgeActiveEdgesTimer();

    else if (message == purgePendingPongsTimer)
        processPurgePendingPongsTimer();

    else
        RoutingProtocolBase::processSelfMessage(message);
}

/*
 * Mensajes HOLA_VEHIC.
 */

//! Programar el temporizador de transmisión de mensajes HOLA_VEHIC.
void RoutingProtocolCar::scheduleHelloCarTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::scheduleHelloCarTimer");

    scheduleAt(omnetpp::simTime() + helloCarInterval + uniform(0, 1),
            helloCarTimer);
}

//! Procesar el temporizador de transmisión de mensajes HOLA_VEIH.
void RoutingProtocolCar::processHelloCarTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processHelloCarTimer");

    const inet::Ipv6Address &primaryUnicastAddress = addressCache->getUnicastAddress(
    PRIMARY_ADDRESS);
    const inet::Ipv6Address &primaryMulticastAddress = addressCache->getMulticastAddress(
    PRIMARY_ADDRESS);
    const inet::Ipv6Address &secondaryUnicastAddress = addressCache->getUnicastAddress(
    SECONDARY_ADDRESS);
    const inet::Ipv6Address &secondaryMulticastAddress = addressCache->getMulticastAddress(
    SECONDARY_ADDRESS);

    const inet::Ipv6InterfaceData *ipv6Data = networkInterface->findProtocolData<
            inet::Ipv6InterfaceData>();

    if (ipv6Data->hasAddress(primaryUnicastAddress))
        sendHelloCar(createHelloCar(primaryUnicastAddress),
                primaryMulticastAddress);

    if (ipv6Data->hasAddress(secondaryUnicastAddress))
        sendHelloCar(createHelloCar(secondaryUnicastAddress),
                secondaryMulticastAddress);

    scheduleHelloCarTimer();
}

/*
 * Mensajes HOLA_VEHIC.
 */

//! Crear mensaje HOLA_VEHIC.
/*!
 * @param carAddress [in] Dirección del vehículo remitente.
 * @return Mensaje HOLA_VEHIC.
 */
const inet::Ptr<HelloCar> RoutingProtocolCar::createHelloCar(
        const inet::Ipv6Address &carAddress) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::createHelloCar");

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
    EV_INFO << "Geohash location: " << geohashLocation.getGeohashString()
            << std::endl;
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

//! Enviar mensaje HOLA_VEHIC.
/*!
 * Encapsula un mensaje HOLA_VEHIC en un datagrama UDP y lo envía
 * a la dirección indicada.
 *
 * @param helloCar [in] Mensaje a enviar.
 * @param destAddress [in] Dirección de destino del mensaje.
 */
void RoutingProtocolCar::sendHelloCar(const inet::Ptr<HelloCar> &helloCar,
        const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::sendHelloCar");

    inet::Packet *udpPacket = new inet::Packet("ANC_VEHIC");
    udpPacket->insertAtBack(helloCar);

    inet::Ptr<inet::UdpHeader> udpHeader = inet::makeShared<inet::UdpHeader>();
    udpHeader->setSourcePort(ROUTING_PROTOCOL_UDP_PORT);
    udpHeader->setDestinationPort(ROUTING_PROTOCOL_UDP_PORT);
    udpPacket->insertAtFront(udpHeader);

    inet::Ptr<inet::L3AddressReq> addresses = udpPacket->addTagIfAbsent<
            inet::L3AddressReq>();
    addresses->setSrcAddress(inet::L3Address(helloCar->getAddress()));
    addresses->setDestAddress(inet::L3Address(destAddress));

    inet::Ptr<inet::HopLimitReq> hopLimit = udpPacket->addTagIfAbsent<
            inet::HopLimitReq>();
    hopLimit->setHopLimit(255);

    inet::Ptr<inet::PacketProtocolTag> packetProtocol = udpPacket->addTagIfAbsent<
            inet::PacketProtocolTag>();
    packetProtocol->setProtocol(&inet::Protocol::manet);

    inet::Ptr<inet::DispatchProtocolReq> dispatchProtocol = udpPacket->addTagIfAbsent<
            inet::DispatchProtocolReq>();
    dispatchProtocol->setProtocol(&inet::Protocol::ipv6);

    sendUdpPacket(udpPacket);
}

//! Procesar mensaje HOLA_VEHIC.
/*!
 * @param helloCar [in] Mensaje a procesar.
 */
void RoutingProtocolCar::processHelloCar(const inet::Ptr<HelloCar> &helloCar) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processHelloHost");

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
    LocationOnRoadNetwork locationOnRoadNetwork = { edge, 0, distanceToVertexA,
            distanceToVertexB };

    EV_INFO << "Address: " << carAddress.str() << std::endl;
    EV_INFO << "Geohash location: " << geohashLocation.getGeohashString()
            << std::endl;
    EV_INFO << "Speed: " << speed << std::endl;
    EV_INFO << "Direction: " << direction << std::endl;
    EV_INFO << "Vertex A: " << vertexA << std::endl;
    EV_INFO << "Vertex B: " << vertexB << std::endl;
    EV_INFO << "Edge: " << edge << std::endl;
    EV_INFO << "Distance to vertex A: " << distanceToVertexA << std::endl;
    EV_INFO << "Distance to vertex B: " << distanceToVertexB << std::endl;

    neighbouringCars.getMap()[carAddress].expiryTime = omnetpp::simTime();
    neighbouringCars.getMap()[carAddress].value = { geohashLocation, speed,
            direction, locationOnRoadNetwork };
    neighbouringCarsByEdge.insert(NeighbouringCarByEdge(edge, carAddress));

    addRoute(carAddress, 128, carAddress, 1,
            omnetpp::simTime() + neighbouringCarValidityTime);

    EV_INFO << "Number of car neighbours: " << neighbouringCars.getMap().size()
            << std::endl;

    showRoutes();
    schedulePurgeNeighbouringCarsTimer();
}

/*
 * Mensajes HOLA_HOST.
 */

//! Procesar mensaje HOLA_HOST.
/*!
 * @param helloHost [in] Mensaje a procesar.
 */
void RoutingProtocolCar::processHelloHost(
        const inet::Ptr<HelloHost> &helloHost) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processHelloHost");

    inet::Ipv6Address hostAddress = helloHost->getAddress();
    GeohashLocation geohashLocation(helloHost->getGeohash(), 12);

    EV_INFO << "Address: " << hostAddress.str() << std::endl;
    EV_INFO << "Geohash location: " << helloHost->getGeohash() << std::endl;
    EV_INFO << "                : " << geohashLocation.getGeohashString()
            << std::endl;

    neighbouringHosts.getMap()[hostAddress] = { omnetpp::simTime(),
            geohashLocation };

    addRoute(hostAddress, 128, hostAddress, 1,
            omnetpp::simTime() + neighbouringHostValidityTime);

    EV_INFO << "Number of host neighbours: "
            << neighbouringHosts.getMap().size() << std::endl;

    showRoutes();

    schedulePurgeNeighbouringHostsTimer();
}

/*
 * Mensajes PING.
 */

//! Crear mensaje PING.
/*!
 * @param carAddress [in] Dirección del vehículo remitente.
 * @param srcVertex [in] Vértice de origen de la arista.
 * @param destVertex [in] Vértice de destino de la arista.
 * @return Mensaje PING.
 */
const inet::Ptr<Ping> RoutingProtocolCar::createPing(
        const inet::Ipv6Address &carAddress, Vertex srcVertex,
        Vertex destVertex) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::createPing");

    const inet::Ptr<Ping> &ping = inet::makeShared<Ping>();

    EV_INFO << "Address: " << carAddress.str() << std::endl;
    EV_INFO << "Target vertex: " << destVertex << std::endl;
    EV_INFO << "Source vertex: " << srcVertex << std::endl;

    int neighbouringCarsCount = getNeighbouringCarsOnEdgeCount();

    // Si no hay vecinos en la misma arista
    if (neighbouringCarsCount == 0)
        return nullptr;

    inet::Ipv6Address nextHopAddress = getRandomNeighbouringCarAddressAheadOnEdge(
            destVertex);

    EV_INFO << "Next hop: " << nextHopAddress.str() << std::endl;

    if (nextHopAddress.isUnspecified())
        return nullptr;

    ping->setAddress(carAddress);
    ping->setSrcVertex(srcVertex);
    ping->setDestVertex(destVertex);

    return ping;
}

//! Enviar mensaje PING.
/*!
 * Encapsula un mensaje PING en un datagrama UDP y lo envía
 * a la dirección indicada.
 *
 * @param ping [in] Mensaje a enviar.
 * @param destAddress [in] Dirección de destino del mensaje.
 */
void RoutingProtocolCar::sendPing(const inet::Ptr<Ping> &ping,
        const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::sendPing");

    inet::Packet *udpPacket = new inet::Packet("Ping");
    udpPacket->insertAtBack(ping);

    inet::Ptr<inet::UdpHeader> udpHeader = inet::makeShared<inet::UdpHeader>();
    udpHeader->setSourcePort(ROUTING_PROTOCOL_UDP_PORT);
    udpHeader->setDestinationPort(ROUTING_PROTOCOL_UDP_PORT);
    udpPacket->insertAtFront(udpHeader);

    inet::Ptr<inet::L3AddressReq> addresses = udpPacket->addTagIfAbsent<
            inet::L3AddressReq>();
    addresses->setSrcAddress(inet::L3Address(ping->getAddress()));
    addresses->setDestAddress(inet::L3Address(destAddress));

    inet::Ptr<inet::HopLimitReq> hopLimit = udpPacket->addTagIfAbsent<
            inet::HopLimitReq>();
    hopLimit->setHopLimit(255);

    inet::Ptr<inet::PacketProtocolTag> packetProtocol = udpPacket->addTagIfAbsent<
            inet::PacketProtocolTag>();
    packetProtocol->setProtocol(&inet::Protocol::manet);

    inet::Ptr<inet::DispatchProtocolReq> dispatchProtocol = udpPacket->addTagIfAbsent<
            inet::DispatchProtocolReq>();
    dispatchProtocol->setProtocol(&inet::Protocol::ipv6);

    sendUdpPacket(udpPacket);
}

//! Procesar mensaje PING.
/*!
 * @param ping [in] Mensaje a procesar.
 */
void RoutingProtocolCar::processPing(const inet::Ptr<Ping> &ping) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processPing");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();

    // Obtener datos del mensaje PING
    const inet::Ipv6Address &pingAddress = ping->getAddress();
    Vertex pingSrcVertex = (Vertex) ping->getSrcVertex();
    Vertex pingDestVertex = (Vertex) ping->getDestVertex();
    Edge pingEdge = boost::edge(pingSrcVertex, pingDestVertex, graph).first;

    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;

    EV_INFO << "Address: " << pingAddress.str() << std::endl;
    EV_INFO << "Edge: " << pingEdge << std::endl;
    EV_INFO << "Target vertex: " << pingDestVertex << std::endl;

    const inet::Ipv6Address &primaryUnicastAddress = addressCache->getUnicastAddress(
    PRIMARY_ADDRESS);
    const inet::Ipv6Address &primaryMulticastAddress = addressCache->getMulticastAddress(
    PRIMARY_ADDRESS);

    // Si el mensaje PING no corresponde a la arista donde circula el vehículo, se descarta
    if (pingEdge != edge) {
        EV_INFO << "Otra arista" << std::endl;
        return;
    }

    // Se verifica si la dirección de destino es la dirección local
    if (interfaceTable->isLocalAddress(
            inet::Ipv6Address::UNSPECIFIED_ADDRESS)) {    // TODO Verificar si el vehículo se encuentra en el vértice 2
        // Si se encuentra en el vértice de destino, se responde con un mensaje PONG
        if (inVertexProximityRadius(locationOnRoadNetwork, pingDestVertex,
                graph)) {
            EV_INFO << "PONG" << std::endl;

            // Responder con mensaje PONG
            inet::Ipv6Address newNextHopAddress = getNeighbouringCarAddressOnEdgeClosestToVertex(
                    pingSrcVertex);

            const inet::Ptr<Pong> pong = createPong(pingAddress, false,
                    pingSrcVertex, pingDestVertex);
            sendPong(pong, newNextHopAddress);

            // Agregar la arista a la lista de aristas activas
            activeEdges.getMap()[edge].expiryTime = omnetpp::simTime();
            schedulePurgeActiveEdgesTimer();

            // Enviar mensaje de estatus de arista activa

            // Si no se encuentra en el vértice de destino
        } else {
            inet::Ipv6Address newNextHopAddress = getRandomNeighbouringCarAddressAheadOnEdge(
                    pingDestVertex);

            // Se env������a a la direcci������n multicast
            ASSERT(primaryUnicastAddress.matches(pingAddress, 64));
            sendPing(ping, primaryMulticastAddress);
        }
    }
}

/*
 * Mensajes PONG.
 */

//! Crear mensaje PONG.
/*!
 * @param destAddress [in] Dirección del vehículo que originó
 * el mensaje PING.
 * @param E [in] Bandera E. Si vale `true`, indica un error en la
 * operación ping-pong.
 * @param vertex1 [in]
 * @param vertex2 [in]
 * @return
 */
const inet::Ptr<Pong> RoutingProtocolCar::createPong(
        const inet::Ipv6Address &destAddress, bool E, Vertex srcVertex,
        Vertex destVertex) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::createPong");

    const inet::Ptr<Pong> &pong = inet::makeShared<Pong>();

    EV_INFO << "Destination address: " << destAddress.str() << std::endl;
    EV_INFO << "N: " << E << std::endl;
    EV_INFO << "Target vertex: " << srcVertex << std::endl;
    EV_INFO << "Source vertex: " << destVertex << std::endl;

    pong->setE(E);
    pong->setDestAddress(destAddress);
    pong->setSrcVertex(srcVertex);
    pong->setDestVertex(destVertex);

    return pong;
}

//! Enviar mensaje PONG.
/*!
 * Encapsula un mensaje PONG en un datagrama UDP y lo envía
 * a la dirección indicada.
 *
 * @param pong [in] Mensaje a enviar.
 * @param destAddress [in] Dirección de destino del mensaje.
 */
void RoutingProtocolCar::sendPong(const inet::Ptr<Pong> &pong,
        const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::sendPong");

    inet::Packet *udpPacket = new inet::Packet("Pong");
    udpPacket->insertAtBack(pong);

    inet::Ptr<inet::UdpHeader> udpHeader = inet::makeShared<inet::UdpHeader>();
    udpHeader->setSourcePort(ROUTING_PROTOCOL_UDP_PORT);
    udpHeader->setDestinationPort(ROUTING_PROTOCOL_UDP_PORT);
    udpPacket->insertAtFront(udpHeader);

    inet::Ptr<inet::L3AddressReq> addresses = udpPacket->addTagIfAbsent<
            inet::L3AddressReq>();
    addresses->setSrcAddress(
            inet::L3Address(addressCache->getUnicastAddress(PRIMARY_ADDRESS)));
    addresses->setDestAddress(inet::L3Address(destAddress));

    inet::Ptr<inet::HopLimitReq> hopLimit = udpPacket->addTagIfAbsent<
            inet::HopLimitReq>();
    hopLimit->setHopLimit(255);

    inet::Ptr<inet::PacketProtocolTag> packetProtocol = udpPacket->addTagIfAbsent<
            inet::PacketProtocolTag>();
    packetProtocol->setProtocol(&inet::Protocol::manet);

    inet::Ptr<inet::DispatchProtocolReq> dispatchProtocol = udpPacket->addTagIfAbsent<
            inet::DispatchProtocolReq>();
    dispatchProtocol->setProtocol(&inet::Protocol::ipv6);

    sendUdpPacket(udpPacket);
}

//! Procesar mensaje PONG.
/*!
 * Si la dirección de destino es una dirección local,
 * se revisa si es un mensaje PONG pendiente, en cuyo caso,
 * si el valor de la bandera E es falso,
 * se establece la arista correspondiente como arista activa,
 * y se programa el temporizador de limpieza de aristas activas.
 * También, se transmite un mensaje HOLA_VEHIC indicando que
 * la arista está activa.
 * Si se acabó el tiempo de espera del mensaje, se ignora el mensaje.
 *
 * Si la dirección de destino no es una dirección local, se verifica
 * si el destinatario se encuentra es un vecino. En ese caso,
 * se retransmite directamente hacia este. En otro caso, se busca un
 * vecino que se encuentre más cerca del vértice de destino.
 *
 * @param pong [in] Mensaje a procesar.
 */
void RoutingProtocolCar::processPong(const inet::Ptr<Pong> &pong) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processPong");

    /*
     * Se extraen los datos del mensaje.
     */
    const inet::Ipv6Address &destAddress = pong->getDestAddress();
    bool E = pong->getE();
    Vertex srcVertex = (Vertex) pong->getSrcVertex();
    Vertex destVertex = (Vertex) pong->getDestVertex();
    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    Edge edge = boost::edge(srcVertex, destVertex, graph).first;

    EV_INFO << "Destination address: " << destAddress.str() << std::endl;
    EV_INFO << "E: " << E << std::endl;
    EV_INFO << "Edge: " << edge;
    EV_INFO << "Target vertex: " << srcVertex << std::endl;

    /*
     * Se revisa si la dirección de destino es una dirección local.
     */
    if (interfaceTable->isLocalAddress(destAddress)) {

        /*
         * Si es un mensaje PONG pendiente y el valor de la
         * bandera E es falso, se establece como arista activa.
         */
        if (pendingPongs.getMap().count(edge)) {
            if (!E) {
                pendingPongs.getMap().erase(edge);
                activeEdges.getMap()[edge].expiryTime = omnetpp::simTime()
                        + activeEdgeValidityTime;
                schedulePurgeActiveEdgesTimer();

                // TODO Transmitir mensaje de HOLA_VEHIC para indicar que la arista está activa.
            }
            // TODO Revisar si hace falta establecer la arista como inactiva si E = verdadero.
        }

        /*
         * Si no es una dirección local, se retransmite el mensaje.
         */
    } else {
        /*
         * Si el destinatario es un vecino, se selecciona su dirección
         * como siguiente salto.
         */
        if (neighbouringCars.getMap().count(destAddress))
            sendPong(pong, destAddress);

        /*
         * En otro caso, se selecciona un vehículo vecino más cercano
         * al vértice de origen.
         */
        else {
            inet::Ipv6Address nextHopAddress = getNeighbouringCarAddressOnEdgeClosestToVertex(
                    srcVertex);
            sendPong(pong, nextHopAddress);
        }
    }
}

/*
 * Directorio de vehículos vecinos.
 */

//! Obtener vehículo vecino aleatorio en la misma arista.
/*!
 * Obtiene aleatoriamente un vehículo vecino que se encuentra en la misma
 * arista, y que esté más cerca del vértice indicado.
 *
 * @param targetVertex [in] Vértice de referencia.
 * @return Dirección IPv6 del vehículo vecino seleccionado.
 */
inet::Ipv6Address RoutingProtocolCar::getRandomNeighbouringCarAddressAheadOnEdge(
        Vertex targetVertex) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::getRandomNeighbouringCarOnEdgeAddress");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);

    double distanceToTargetVertex = getDistanceToVertex(
            mobility->getLocationOnRoadNetwork(), targetVertex, graph);
    ASSERT(distanceToTargetVertex != std::numeric_limits<double>::infinity());

    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    std::vector<inet::Ipv6Address> addresses;

    // Se agregan las direcciones de los vehículos que están adelante en la misma arista
    NeighbouringCarsByEdgeConstIterator neighbouringCarIt = neighbouringCarsByEdge.lower_bound(
            edge);
    NeighbouringCarsByEdgeConstIterator neighbouringCarEndIt = neighbouringCarsByEdge.upper_bound(
            edge);
    while (neighbouringCarIt != neighbouringCarEndIt) {
        inet::Ipv6Address neighbouringCarAddress = neighbouringCarIt->second;
        // TODO Arreglar
        /*const NeighbouringCarsConstIterator neighbouringCarIt = neighbouringCars.find(
         neighbouringCarAddress);
         /*        if (neighbouringCarIt != neighbouringCars.end()) {
         LocationOnRoadNetwork &locationOnRoadNetwork = neighbouringCarIt->second.locationOnRoadNetwork;
         double neighbouringCarDistanceToTargetVertex = getDistanceToVertex(targetVertex, locationOnRoadNetwork, graph);
         ASSERT(neighbouringCarDistanceToTargetVertex != std::numeric_limits<double>::infinity());

         if (neighbouringCarDistanceToTargetVertex < distanceToTargetVertex)
         addresses.push_back(neighbouringCarAddress);
         }*/

    }

    // Se devuelve una dirección aleatoria
    if (addresses.size() > 0)
        return addresses[rand() % addresses.size()];

    else
        return inet::Ipv6Address::UNSPECIFIED_ADDRESS;
}

//! Obtener vehículo vecino más cercano a un vértice que se encuentra en
//! la misma arista.
/*!
 * Se bucan los vehículos vecinos que circulan sobre la misma arista,
 * y se obtiene el que se encuentra a la menor distancia del vértice
 * indicado.
 *
 * @param vertex [in] Vértice de referencia.
 * @return
 */
inet::Ipv6Address RoutingProtocolCar::getNeighbouringCarAddressOnEdgeClosestToVertex(
        Vertex vertex) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::getNeighbouringCarAddressOnEdgeClosestToVertex");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);
    const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertex1;
    const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertex1;

    double minDistanceToTargetVertex = std::numeric_limits<double>::infinity();
    inet::Ipv6Address address;

    NeighbouringCarsByEdgeIterator neighbouringCarIt = neighbouringCarsByEdge.lower_bound(
            edge);
    NeighbouringCarsByEdgeIterator neighbouringCarEndIt = neighbouringCarsByEdge.upper_bound(
            edge);
    while (neighbouringCarIt != neighbouringCarEndIt) {
        const inet::Ipv6Address neighbouringCarAddress = neighbouringCarIt->second;
        const NeighbouringCar &neighbouringCar = neighbouringCars.getMap()[neighbouringCarAddress];
        const LocationOnRoadNetwork &locationOnRoadNetwork = neighbouringCar.value.locationOnRoadNetwork;
        double neighbouringCarDistanceToTargetVertex = getDistanceToVertex(
                locationOnRoadNetwork, vertex, graph);
        ASSERT(
                neighbouringCarDistanceToTargetVertex
                        != std::numeric_limits<double>::infinity());

        if (neighbouringCarDistanceToTargetVertex < minDistanceToTargetVertex)
            minDistanceToTargetVertex = neighbouringCarDistanceToTargetVertex;
        address = neighbouringCarAddress;
    }

    return address;
}

//! Obtener la cantidad de vehículos vecinos que se encuentran
//! en la misma arista.
/*!
 * @return Cantidad de vehículos vecinos que se encuentran en
 * la misma arista.
 */
int RoutingProtocolCar::getNeighbouringCarsOnEdgeCount() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::getNeighbouringCarsOnEdgeCount");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    int n = 0;

    showStatus();
    showNeighbouringCars();

    NeighbouringCarsConstIterator neighbouringCarsIt = neighbouringCars.getMap().begin();
    NeighbouringCarsConstIterator neighbouringCarsEndIt = neighbouringCars.getMap().end();
    while (neighbouringCarsIt != neighbouringCarsEndIt) {
        if (neighbouringCarsIt->second.value.locationOnRoadNetwork.edge == edge)
            n++;

        neighbouringCarsIt++;
    }

    return n;
}

//! Procesar el temporizador de limpieza del directorio de
//! vehículos vecinos.
void RoutingProtocolCar::processPurgeNeighbouringCarsTimer() {
    // TODO Implementar para que se eliminen las rutas de los vehículos vecinos que se eliminaron.
}

/*
 * Directorio de _hosts_ vecinos.
 */

//! Imrpimir el directorio de _hosts_ vecinos.
void RoutingProtocolCar::showNeighbouringHosts() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::showNeighbouringHosts");

}

//! Programar el temporizador de limpieza del directorio de _hosts_ vecinos.
void RoutingProtocolCar::schedulePurgeNeighbouringHostsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::schedulePurgeNeighbouringHostsTimer");

    omnetpp::simtime_t nextExpiryTime = neighbouringHosts.getNextExpiryTime();

    EV_INFO << "Next expiry time: " << nextExpiryTime << std::endl;

    if (nextExpiryTime == omnetpp::SimTime::getMaxTime()) {
        if (purgeNeighbouringHostsTimer->isScheduled())
            cancelEvent(purgeNeighbouringHostsTimer);

    } else {
        if (!purgeNeighbouringHostsTimer->isScheduled())
            scheduleAt(nextExpiryTime, purgeNeighbouringHostsTimer);

        else if (purgeNeighbouringHostsTimer->getArrivalTime()
                != nextExpiryTime) {
            cancelEvent(purgeNeighbouringHostsTimer);
            scheduleAt(nextExpiryTime, purgeNeighbouringHostsTimer);
        }
    }
}

//! Procesar el temporizador de limpieza del directorio de _hosts_ vecinos.
void RoutingProtocolCar::processPurgeNeighbouringHostsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processPurgeNeighbouringHostsTimer");

    neighbouringHosts.removeOldValues(omnetpp::simTime());
    schedulePurgeNeighbouringHostsTimer();
}

/*
 * Aristas activas.
 */

//! Imprimir las aristas activas.
void RoutingProtocolCar::showActiveEdges() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::showActiveEdges");

}

//! Programar el temporizador de limpieza de aristas activas.
void RoutingProtocolCar::schedulePurgeActiveEdgesTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::schedulePurgeActiveEdgesTimer");

    omnetpp::simtime_t nextExpiryTime = activeEdges.getNextExpiryTime();

    EV_INFO << "Next expiry time: " << nextExpiryTime << std::endl;

    if (nextExpiryTime == omnetpp::SimTime::getMaxTime()) {
        if (purgeActiveEdgesTimer->isScheduled())
            cancelEvent(purgeActiveEdgesTimer);

    } else {
        if (!purgeActiveEdgesTimer->isScheduled())
            scheduleAt(nextExpiryTime, purgeActiveEdgesTimer);

        else if (purgeActiveEdgesTimer->getArrivalTime() != nextExpiryTime) {
            cancelEvent(purgeActiveEdgesTimer);
            scheduleAt(nextExpiryTime, purgeActiveEdgesTimer);
        }
    }
}

//! Procesar el temporizador de limpieza de aristas activas.
void RoutingProtocolCar::processPurgeActiveEdgesTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processPurgeActiveEdgesTimer");

    activeEdges.removeOldValues(omnetpp::simTime());
    schedulePurgeActiveEdgesTimer();
}

/*
 * Datagramas demorados.
 */

//! Imprimir los datagramas demorados.
void RoutingProtocolCar::showDelayedDatagrams() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::showDelayedDatagrams");

}

//! Programar el temporizador de limpieza de datagramas demorados.
void RoutingProtocolCar::schedulePurgeDelayedDatagramsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::schedulePurgeDelayedDatagramsTimer");

    omnetpp::simtime_t nextExpiryTime = delayedDatagrams.getNextExpiryTime();

    EV_INFO << "Next expiry time: " << nextExpiryTime << std::endl;

    if (nextExpiryTime == omnetpp::SimTime::getMaxTime()) {
        if (purgeDelayedDatagramsTimer->isScheduled())
            cancelEvent(purgeDelayedDatagramsTimer);

    } else {
        if (!purgeDelayedDatagramsTimer->isScheduled())
            scheduleAt(nextExpiryTime, purgeDelayedDatagramsTimer);

        else if (purgeDelayedDatagramsTimer->getArrivalTime()
                != nextExpiryTime) {
            cancelEvent(purgeDelayedDatagramsTimer);
            scheduleAt(nextExpiryTime, purgeDelayedDatagramsTimer);
        }
    }
}

//! Procesar el temporizador de limpieza de datagramas demorados.
void RoutingProtocolCar::processPurgeDelayedDatagramsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processPurgeDelayedDatagramsTimer");

    delayedDatagrams.removeOldValues(omnetpp::simTime());
    schedulePurgeDelayedDatagramsTimer();
}

/*
 * Mensajes PONG pendientes.
 */

//! Imprimir los mensajes PONG pendientes.
void RoutingProtocolCar::showPendingPongs() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::showPendingPongs");

}

//! Programar el temporizador de limpieza de mensajes PONG pendientes.
void RoutingProtocolCar::schedulePurgePendingPongsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::schedulePurgePendingPongsTimer");

    omnetpp::simtime_t nextExpiryTime = pendingPongs.getNextExpiryTime();

    EV_INFO << "Next expiry time: " << nextExpiryTime << std::endl;

    if (nextExpiryTime == omnetpp::SimTime::getMaxTime()) {
        if (purgePendingPongsTimer->isScheduled())
            cancelEvent(purgePendingPongsTimer);

    } else {
        if (!purgePendingPongsTimer->isScheduled())
            scheduleAt(nextExpiryTime, purgePendingPongsTimer);

        else if (purgePendingPongsTimer->getArrivalTime() != nextExpiryTime) {
            cancelEvent(purgePendingPongsTimer);
            scheduleAt(nextExpiryTime, purgePendingPongsTimer);
        }
    }
}

//! Procesar el temporizador de limpieza de mensajes PONG pendientes.
void RoutingProtocolCar::processPurgePendingPongsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processPurgePendingPongsTimer");

    pendingPongs.removeOldValues(omnetpp::simTime());
    schedulePurgePendingPongsTimer();
}

/*
 * Enrutamiento.
 */

//! Enrutar datagrama.
/*!
 * Revisa si existe en la tabla de enrutamiento una ruta hacia la
 * dirección de destino. Si no existe, se intenta descubrir y crear una
 * ruta. Si no se encuentra la ruta, se descarta el datagrama.
 *
 * @param datagram [in] Datagrama a enrutar.
 * @param destAddress [in] Dirección IPv6 de destino.
 * @return Resultado del enrutamiento.
 */
inet::INetfilter::IHook::Result RoutingProtocolCar::routeDatagram(
        inet::Packet *datagram, const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::routeDatagram");

    /*********************************************************************************
     * Validaci��n de la cabecera de opciones de salto por salto
     *********************************************************************************/
    if (!validateHopByHopOptionsHeader(datagram))
        return inet::INetfilter::IHook::Result::DROP;

    /*********************************************************************************
     * Verificar si ya existe una ruta
     *********************************************************************************/
    const inet::Ipv6Route *route = routingTable->doLongestPrefixMatch(
            destAddress);
    if (route != nullptr) {
        if (omnetpp::simTime() < route->getExpiryTime())
            return inet::INetfilter::IHook::ACCEPT;

        else
            routingTable->deleteRoute(const_cast<inet::Ipv6Route*>(route));
    }

    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const Graph &graph = roadNetwork->getGraph();

// Se obtiene la ubicación vial del vehículo
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;

    /*********************************************************************************
     * Obtener vértices visitados
     *********************************************************************************/
    const TlvVisitedVerticesOption *visitedVerticesOption = findTlvOption<
            TlvVisitedVerticesOption>(datagram);
    ASSERT(visitedVerticesOption != nullptr);
    int numVisitedVertices = visitedVerticesOption->getVisitedVerticesArraySize();
    VertexSet visitedVertices;
    for (int i = 0; i < numVisitedVertices; i++)
        visitedVertices.insert(visitedVerticesOption->getVisitedVertices(i));

    /*********************************************************************************
     * Obtener aristas inactivas
     *********************************************************************************/
    int numActiveEdges = this->activeEdges.getMap().size();
    ActiveEdgesConstIterator activeEdgeIt = this->activeEdges.getMap().begin();
    EdgeSet activeEdges;
    for (int i = 0;
            i < numActiveEdges
                    && activeEdgeIt != this->activeEdges.getMap().end();
            i++, activeEdgeIt++)
        activeEdges.insert(activeEdgeIt->first);

    /*********************************************************************************
     * Calcular ruta más corta
     *********************************************************************************/
    ShortestPath shortestPath;
    shortestPath.computeShortestPath(edge, graph, visitedVertices, activeEdges);

    /*********************************************************************************
     * Obtener vértice destino local
     *********************************************************************************/
    Vertex destVertex = getLocalDestVertex(datagram, shortestPath);

// Se obtiene la ruta m��s corta al vértice de destino
    VertexVector shortestPathToDestVertex = shortestPath.getShortestPathToVertex(
            destVertex, graph);

// Se busca el siguiente salto
    inet::Ipv6Address nextHopAddress = findNextHop(shortestPathToDestVertex,
            shortestPath);

    if (!nextHopAddress.isUnspecified()) {
        addRoute(destAddress, 128, nextHopAddress, 1,
                omnetpp::simTime() + routeValidityTime);
        return inet::INetfilter::IHook::Result::ACCEPT;
    }

    return inet::INetfilter::IHook::Result::DROP;
}

//! Verificar la cabecera de opciones de salto por salto.
/*!
 * Verifica si la cabecera tiene la opción de ubicación del destino.
 *
 * Si el destino se encuentra en la misma subred, se verifica si
 * la cabecera contiene la opción de ubicación vial del destino. Si no
 * la tiene, la agrega.
 *
 * Si la cabecera no contiene la opción de vértices visitados, la agrega.
 */
bool RoutingProtocolCar::validateHopByHopOptionsHeader(
        inet::Packet *datagram) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::validateHopByHopOptionsHeader");

// Se obtiene la ubicación de destino del datagrama
    const TlvDestGeohashLocationOption *destGeohashLocationOption = findTlvOption<
            TlvDestGeohashLocationOption>(datagram);

    if (destGeohashLocationOption == nullptr)
        return false;

    GeohashLocation destGeohashLocation(destGeohashLocationOption->getGeohash(),
            12);

// Si el destino está en la misma región Geohash
    const GeohashLocation &geohashRegion = mobility->getRoadNetwork()->getGeohashRegion();
    if (geohashRegion.contains(destGeohashLocation)) {
        // Si el datagrama no incluye la opción de ubicación vial del destino, se agrega
        const TlvDestLocationOnRoadNetworkOption *destLocationOnRoadNetworkOption = findTlvOption<
                TlvDestLocationOnRoadNetworkOption>(datagram);
        if (destLocationOnRoadNetworkOption == nullptr) {
            setTlvDestLocationOnRoadNetworkOption(datagram,
                    destGeohashLocation);
        }
    }

// Si el datagrama no incluye la opción de vértices visitados, se agrega
    const TlvVisitedVerticesOption *visitedVerticesOption = findTlvOption<
            TlvVisitedVerticesOption>(datagram);
    if (visitedVerticesOption == nullptr) {
        TlvVisitedVerticesOption *visitedVerticesOption = createTlvVisitedVerticesOption();
        setTlvOption(datagram, visitedVerticesOption);
    }

    return true;
}

//! Obtener el vértice de destino local.
/*!
 * @param datagram [in] Datagrama a enrutar.
 * @param shortestPath [in] Rutas más cortas.
 * @return Vértice de destino local.
 */
Vertex RoutingProtocolCar::getLocalDestVertex(inet::Packet *datagram,
        const ShortestPath &shortestPath) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::getLocalDestVertex");

    const TlvDestGeohashLocationOption *destGeohashLocationOption = findTlvOption<
            TlvDestGeohashLocationOption>(datagram);
    ASSERT(destGeohashLocationOption != nullptr);

    const TlvVisitedVerticesOption *visitedVerticesOption = findTlvOption<
            TlvVisitedVerticesOption>(datagram);
    ASSERT(visitedVerticesOption != nullptr);

    GeohashLocation destGeohashLocation(destGeohashLocationOption->getGeohash(),
            12);
    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const GeohashLocation &geohashRegion = roadNetwork->getGeohashRegion();

    Vertex destVertex;

// Si el destino se encuentra en la misma subred
    if (geohashRegion.contains(destGeohashLocation)) {
        const TlvDestLocationOnRoadNetworkOption *locationOnRoadNetworkOption = findTlvOption<
                TlvDestLocationOnRoadNetworkOption>(datagram);
        ASSERT(locationOnRoadNetworkOption != nullptr);

        Vertex vertexA = (Vertex) locationOnRoadNetworkOption->getVertexA();
        Vertex vertexB = (Vertex) locationOnRoadNetworkOption->getVertexB();
        double routeDistanceA = shortestPath.getRouteDistance(vertexA);
        double routeDistanceB = shortestPath.getRouteDistance(vertexB);

        // Se selecciona como vértice destino local el vértice con menor distancia de ruta
        if (routeDistanceA < routeDistanceB)
            destVertex = vertexA;

        else
            destVertex = vertexB;

        // Si el destino se encuentra en una subred distinta
    } else {
        const GeographicLib::GeoCoords &destLocation = destGeohashLocation.getLocation();
        const Bounds &localRegionBounds = geohashRegion.getBounds();

        VertexVector gatewayVertices;

        // Si el vértice se encuentra al norte de la regi��n local
        if (destLocation.Latitude() > localRegionBounds.getNorth()) {
            // Se obtienen los vértices gateway del norte
            const VertexVector &northGatewayVertices = roadNetwork->getGatewayVertices(
                    GeohashLocation::Direction::NORTH);
            gatewayVertices.insert(gatewayVertices.end(),
                    northGatewayVertices.begin(), northGatewayVertices.end());

            // Si el vértice se encuentra al sur de la regi��n local
        } else if (destLocation.Longitude() < localRegionBounds.getSouth()) {
            // Se obtienen los vértices gateway del sur
            const VertexVector &southGatewayVertices = roadNetwork->getGatewayVertices(
                    GeohashLocation::Direction::SOUTH);
            gatewayVertices.insert(gatewayVertices.end(),
                    southGatewayVertices.begin(), southGatewayVertices.end());
        }

        // Si el vértice se encuentra al este de la regi��n local
        if (destLocation.Longitude() > localRegionBounds.getEast()) {
            // Se obtienen los vértices gateway del este
            const VertexVector &eastGatewayVertices = roadNetwork->getGatewayVertices(
                    GeohashLocation::Direction::EAST);
            gatewayVertices.insert(gatewayVertices.end(),
                    eastGatewayVertices.begin(), eastGatewayVertices.end());

            // Si el vértice se encuentra al oeste de la regi��n local
        } else if (destLocation.Latitude() < localRegionBounds.getWest()) {
            // Se obtienen los vértices gateway del oeste
            const VertexVector &westGatewayVertices = roadNetwork->getGatewayVertices(
                    GeohashLocation::Direction::WEST);
            gatewayVertices.insert(gatewayVertices.end(),
                    westGatewayVertices.begin(), westGatewayVertices.end());
        }

        // Se busca el vértice gateway cuya distancia sea m��nima
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

//! Se obtiene el conjunto de vértices visitados.
/*!
 * @param visitedVerticesOption [in] Opción de vértices visitados.
 * @return Conjunto de vértices visitados.
 */
VertexSet RoutingProtocolCar::getVisitedVertices(
        TlvVisitedVerticesOption *visitedVerticesOption) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::getVisitedVertices");

    VertexSet visitedVertices;
    Vertex visitedVertex;
    int numVisitedVertices = visitedVerticesOption->getVisitedVerticesArraySize();
    for (int i = 0; i < numVisitedVertices; i++) {
        visitedVertex = visitedVerticesOption->getVisitedVertices(i);
        visitedVertices.insert(visitedVertex);
    }

    return visitedVertices;
}

//! Obtener el vértice de destino.
/*!
 * @param destGeohashLocation [in] Ubicación Geohash del destino.
 * @param destEdge [in] Arista de la ubicación del destno.
 * @param shortestPath [in] Rutas más cortas.
 * @return Vértice de destino.
 */
Vertex RoutingProtocolCar::getDestVertex(
        const GeohashLocation &destGeohashLocation, Edge destEdge,
        const ShortestPath &shortestPath) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::destLocationOnRoadNetwork");

    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const Graph &graph = roadNetwork->getGraph();
    Vertex destVertexA = boost::source(destEdge, graph);
    Vertex destVertexB = boost::target(destEdge, graph);
    Vertex destVertex;

// Si el destino está en otra subred
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

        // Se obtienen los vértices gateway posibles
        VertexVector gatewayVertices;
        if (directionLat != GeohashLocation::Direction::NONE) {
            const VertexVector &gatewayVerticesLat = roadNetwork->getGatewayVertices(
                    directionLat);
            gatewayVertices.insert(gatewayVertices.end(),
                    gatewayVerticesLat.begin(), gatewayVerticesLat.end());
        }
        if (directionLat != GeohashLocation::Direction::NONE) {
            const VertexVector &gatewayVerticesLon = roadNetwork->getGatewayVertices(
                    directionLon);
            gatewayVertices.insert(gatewayVertices.end(),
                    gatewayVerticesLon.begin(), gatewayVerticesLon.end());
        }

        // Buscar el vértice gateway con distancia mínima
        double minDistance = std::numeric_limits<double>::infinity();
        for (const Vertex &vertex : gatewayVertices) {
            double distance = shortestPath.getRouteDistance(vertex);
            if (distance < minDistance) {
                minDistance = distance;
                destVertex = vertex;
            }
        }

        // Si el destino está en la misma subred
    } else {
        // Se obtiene la ubicación vial del destino
        Vertex destVertexA = boost::source(destEdge, graph);
        Vertex destVertexB = boost::target(destEdge, graph);

        double distanceToDestVertexA = shortestPath.getRouteDistance(
                destVertexA);
        double distanceToDestVertexB = shortestPath.getRouteDistance(
                destVertexB);

        destVertex =
                distanceToDestVertexA < distanceToDestVertexB ?
                        destVertexA :
                        destVertexB;
    }

    return destVertex;
}

//! Obtener aristas en la ruta más corta que forman un tramo recto.
/*!
 * Se obtienen las aristas en la ruta que forman el tramo largo más recto,
 * y en las que haya vehículos vecinos circulando.
 *
 * @param shortestPathToDestVertex [in] Ruta más corta al vértice
 * de destino.
 * @param shortestPath [in] Rutas más cortas.
 * @return Aristas que forman un
 */
EdgeVector RoutingProtocolCar::getReachableEdges(
        const VertexVector &shortestPathToDestVertex,
        const ShortestPath &shortestPath) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::getReachableEdges");

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

            // Si la distancia hacia el siguiente vértice es aceptable
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

//! Encontrar siguiente salto.
/*!
 * Se obtiene el siguiente salto en la ruta.
 *
 * @param shortestPathToDestVertex [in] Ruta más corta al vértice
 * de destino.
 * @param shortestPath [in] Rutas más cortas.
 * @return Dirección IPv6 del siguiente salto.
 */
inet::Ipv6Address RoutingProtocolCar::findNextHop(
        const VertexVector &shortestPathToDestVertex,
        const ShortestPath &shortestPath) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::findNextHop");

    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const Graph &graph = roadNetwork->getGraph();

    EdgeVector reachableEdges = getReachableEdges(shortestPathToDestVertex,
            shortestPath);
    ASSERT(reachableEdges.size() > 0);
    EdgeVectorIterator edgeIt = reachableEdges.end() - 1;
    Edge edge;
    Vertex vertexB;
    inet::Ipv6Address nextHopAddress;

    // Si hay únicamente una arista alcanzable
    if (reachableEdges.size() == 1) {
        edge = reachableEdges[0];
        vertexB = boost::target(edge, graph);
        GeohashLocation::Direction gatewayType = graph[vertexB].gatewayType;

        // Si el vértice de destino de la arista es un gateway
        if (gatewayType != GeohashLocation::Direction::NONE) {
            const GeohashLocation &geohashRegion = roadNetwork->getGeohashRegion();
            GeohashLocation neighbourGeohashRegion;
            geohashRegion.getNeighbour(gatewayType, neighbourGeohashRegion);
            nextHopAddress = findNeighbourInNeighbourinRegion(
                    neighbourGeohashRegion);

            if (!nextHopAddress.isUnspecified())
                return nextHopAddress;
        }
    }

// Mientras no se encuentre un vehículo vecino en la arista
    while (neighbouringCarsByEdge.find(*edgeIt) == neighbouringCarsByEdge.end())
        // Se obtiene la siguiente arista
        edgeIt--;

    edge = *edgeIt;    // Primera arista por donde circulan vehículos vecinos
    vertexB = boost::target(edge, graph);    // Vértice de destino de la arista
    double minDistanceToVertexB = std::numeric_limits<double>::infinity();    // Distancia mínima delos vecinos al vértice de destino de la arista
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    double distanceToVertexB = getDistanceToVertex(locationOnRoadNetwork,
            vertexB, graph);    // Distancia del vehículo al vértice B (puede ser infinito si el vértice está en otra arista)
    nextHopAddress = inet::Ipv6Address::UNSPECIFIED_ADDRESS;    // Dirección del vecino más cercano al vértice de destino de la arista
    NeighbouringCarsByEdgeConstIterator neighbouringCarIt = neighbouringCarsByEdge.lower_bound(
            edge);
    NeighbouringCarsByEdgeConstIterator neighbouringCarEndIt = neighbouringCarsByEdge.upper_bound(
            edge);

    while (neighbouringCarIt != neighbouringCarEndIt) {
        const inet::Ipv6Address &neighbouringCarAddress = neighbouringCarIt->second;
        const NeighbouringCar &neighbouringCar = neighbouringCars.getMap().at(
                neighbouringCarAddress);
        const LocationOnRoadNetwork &locationOnRoadNetwork = neighbouringCar.value.locationOnRoadNetwork;
        double neighbouringCarDistanceToVertexB = getDistanceToVertex(
                locationOnRoadNetwork, vertexB, graph);

        // Si se encontró un vecino que está a una distancia menor del vértice de destino
        if (neighbouringCarDistanceToVertexB < minDistanceToVertexB) {
            // Si se encuentra en la misma arista del vehículo
            if (edge == locationOnRoadNetwork.edge) {
                // Si se encuentra a una distancia menor del vértice de destino, se considera como siguiente salto
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

//! Obtener vehículo vecino en la región Geohash adyacente.
/*!
 * @param neighbouringGeohashRegion [in] Región Geohash adyacente.
 * @return Dirección IPv6 del vehículo vecino en la región Geohash indicada.
 */
inet::Ipv6Address RoutingProtocolCar::findNeighbourInNeighbourinRegion(
        const GeohashLocation &neighbouringGeohashRegion) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::findNeighbourInNeighbourinRegion");

    NeighbouringCarsConstIterator neighbouringCarsIt = neighbouringCars.getMap().begin();
    NeighbouringCarsConstIterator neighbouringCarsEndIt = neighbouringCars.getMap().end();

    while (neighbouringCarsIt != neighbouringCarsEndIt) {
        const GeohashLocation &neighbouringCarGeohashLocation = neighbouringCarsIt->second.value.geohashLocation;
        if (neighbouringGeohashRegion.contains(neighbouringCarGeohashLocation))
            return neighbouringCarsIt->first;
    }

    return inet::Ipv6Address::UNSPECIFIED_ADDRESS;
}

/*
 * Estatus del vehículo.
 */

//! Mostrar la dirección IPv6 del vehículo y su ubicación vial.
void RoutingProtocolCar::showStatus() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::show");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork = mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertex1;
    const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertex2;

    EV_INFO << "Address: " << addressCache->getUnicastAddress(PRIMARY_ADDRESS)
            << std::endl;
    EV_INFO << "Edge: " << edge << std::endl;
    EV_INFO << "Distance to vertex A: " << distanceToVertexA << std::endl;
    EV_INFO << "Distance to vertex B: " << distanceToVertexB << std::endl;
}

inet::INetfilter::IHook::Result RoutingProtocolCar::datagramPreRoutingHook(
        inet::Packet *datagram) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::datagramPreRoutingHook");
    EV_INFO << "Datagram: " << datagram->str() << std::endl;

    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader = inet::getNetworkProtocolHeader(
            datagram);
    inet::Ipv6Address srcAddress = networkHeader->getSourceAddress().toIpv6();
    inet::Ipv6Address destAddress = networkHeader->getDestinationAddress().toIpv6();

    EV_INFO << "Source address: " << srcAddress.str() << std::endl;
    EV_INFO << "Destination address: " << destAddress.str() << std::endl;

    if (interfaceTable->isLocalAddress(inet::L3Address(destAddress))
            || !destAddress.isSiteLocal() || destAddress.isMulticast())
        return inet::INetfilter::IHook::ACCEPT;

//return routeDatagram(datagram, destAddress);
    return inet::INetfilter::IHook::ACCEPT;
}

inet::INetfilter::IHook::Result RoutingProtocolCar::datagramLocalOutHook(
        inet::Packet *datagram) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::datagramLocalOutHook");
    EV_INFO << "Datagram: " << datagram->str() << std::endl;

    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader = inet::getNetworkProtocolHeader(
            datagram);
    inet::Ipv6Address srcAddress = networkHeader->getSourceAddress().toIpv6();
    inet::Ipv6Address destAddress = networkHeader->getDestinationAddress().toIpv6();

    EV_INFO << "Source address: " << srcAddress.str() << std::endl;
    EV_INFO << "Destination address: " << destAddress.str() << std::endl;

    if (interfaceTable->isLocalAddress(inet::L3Address(destAddress))
            || !destAddress.isSiteLocal() || destAddress.isMulticast())
        return inet::INetfilter::IHook::ACCEPT;

    return inet::INetfilter::IHook::ACCEPT;
}

void RoutingProtocolCar::handleStartOperation(
        inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::handleStartOperation");

    RoutingProtocolBase::handleStartOperation(operation);
    scheduleHelloCarTimer();
}

void RoutingProtocolCar::handleStopOperation(
        inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::handleStopOperation");

    RoutingProtocolBase::handleStopOperation(operation);
    cancelAndDelete(helloCarTimer);
    cancelAndDelete(purgeNeighbouringHostsTimer);
    cancelAndDelete(purgeActiveEdgesTimer);
    cancelAndDelete(purgeDelayedDatagramsTimer);
    cancelAndDelete(purgePendingPongsTimer);
    neighbouringHosts.getMap().clear();
    activeEdges.getMap().clear();
}

void RoutingProtocolCar::handleCrashOperation(
        inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::handleCrashOperation");

    RoutingProtocolBase::handleCrashOperation(operation);
    cancelAndDelete(helloCarTimer);
    cancelAndDelete(purgeNeighbouringHostsTimer);
    cancelAndDelete(purgeActiveEdgesTimer);
    cancelAndDelete(purgeDelayedDatagramsTimer);
    cancelAndDelete(purgePendingPongsTimer);
    neighbouringHosts.getMap().clear();
    activeEdges.getMap().clear();
}

void RoutingProtocolCar::receiveSignal(omnetpp::cComponent *source,
        omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
        cObject *details) {

}
