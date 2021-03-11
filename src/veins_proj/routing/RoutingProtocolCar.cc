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

#include "veins_proj/geohash/GeohashLocation.h"
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

/*!
 * @brief Inicialización.
 *
 * @param [in] Etapa de inicialización.
 */
void RoutingProtocolCar::initialize(int stage) {
    RoutingProtocolBase::initialize(stage);

    /*
     * Etapa de Inicialización local.
     */
    if (stage == inet::INITSTAGE_LOCAL) {
        /*
         * Contexto.
         */
        mobility = omnetpp::check_and_cast<CarMobility*>(
                host->getSubmodule("mobility"));
        if (!mobility)
            throw omnetpp::cRuntimeError("No mobility module found");
        configurator = omnetpp::check_and_cast<CarConfigurator*>(
                getModuleByPath(par("configuratorModule")));
        if (!configurator)
            throw omnetpp::cRuntimeError("No configurator module found");

        /*
         * Mensajes propios.
         */
        helloCarTimer = new omnetpp::cMessage("helloCarTimer");
        purgeNeighbouringHostsTimer = new omnetpp::cMessage(
                "purgeNeighbouringHostsTimer");
        purgeEdgesStatusTimer = new omnetpp::cMessage("purgeEdgesStatusTimer");
        purgePendingPongsTimer = new omnetpp::cMessage(
                "purgePendingPongsTimer");
    }
}

/*
 * Manejo de mensajes.
 */

/*!
 * @brief Manejo de mensajes propios.
 *
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

    else if (message == purgeEdgesStatusTimer)
        processPurgeEdgesStatusTimer();

    else if (message == purgePendingPongsTimer)
        processPurgePendingPongsTimer();

    else
        RoutingProtocolBase::processSelfMessage(message);
}

/*
 * Mensajes HOLA_VEHIC.
 */

/*!
 * @brief Programar el temporizador de transmisión de mensajes HOLA_VEHIC.
 */
void RoutingProtocolCar::scheduleHelloCarTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::scheduleHelloCarTimer");

    scheduleAt(omnetpp::simTime() + helloCarInterval + uniform(0, 1),
            helloCarTimer);
}

/*!
 * @brief Procesar el temporizador de transmisión de mensajes HOLA_VEIC.
 */
void RoutingProtocolCar::processHelloCarTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processHelloCarTimer");

    const inet::Ipv6Address &primaryUnicastAddress =
            configurator->getUnicastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
    const inet::Ipv6Address &primaryMulticastAddress =
            configurator->getMulticastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
    const inet::Ipv6Address &secondaryUnicastAddress =
            configurator->getUnicastAddress(
                    ConfiguratorBase::NetworkType::SECONDARY);
    const inet::Ipv6Address &secondaryMulticastAddress =
            configurator->getMulticastAddress(
                    ConfiguratorBase::NetworkType::SECONDARY);

    const inet::Ipv6InterfaceData *ipv6Data =
            networkInterface->findProtocolData<inet::Ipv6InterfaceData>();

    if (ipv6Data->hasAddress(primaryUnicastAddress)) {
        const inet::Ptr<HelloCar> helloCar = createHelloCar(
                primaryUnicastAddress);
        sendRoutingMessage(helloCar, "ANC_VEHIC", primaryUnicastAddress,
                primaryMulticastAddress);
    }

    if (ipv6Data->hasAddress(secondaryUnicastAddress)) {
        const inet::Ptr<HelloCar> helloCar = createHelloCar(
                secondaryUnicastAddress);
        sendRoutingMessage(helloCar, "ANC_VEHIC", secondaryUnicastAddress,
                secondaryMulticastAddress);
    }

    scheduleHelloCarTimer();
}

/*
 * Mensajes HOLA_VEHIC.
 */

/*!
 * @brief Crear mensaje HOLA_VEHIC.
 *
 * @param srcAddress [in] Dirección del vehículo remitente.
 *
 * @return Mensaje HOLA_VEHIC.
 */
const inet::Ptr<HelloCar> RoutingProtocolCar::createHelloCar(
        const inet::Ipv6Address &srcAddress) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::createHelloCar");

    /*
     * Se obtienen los datos para el mensaje.
     */
    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork =
            mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);
    const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertexA;
    const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertexB;
    const GeohashLocation &geohashLocation = mobility->getGeohashLocation();
    double speed = mobility->getSpeed();
    double direction = mobility->getDirection();

    // @formatter:off
    EV_DEBUG << "Address: " << srcAddress.str() << std::endl
             << "Geohash location: " << geohashLocation.getGeohashString() << std::endl
             << "Speed: " << speed << std::endl
             << "Adjacency: " << direction << std::endl
             << "Vertex A: " << vertexA << std::endl
             << "Vertex B: " << vertexB << std::endl
             << "Distance to vertex A: " << distanceToVertexA << std::endl
             << "Distance to vertex B: " << distanceToVertexB << std::endl;
                                            // @formatter:on

    /*
     * Se crea el mensaje y se le agregan los datos.
     */
    const inet::Ptr<HelloCar> &helloCar = inet::makeShared<HelloCar>();
    helloCar->setSrcAddress(srcAddress);
    helloCar->setGeohash(geohashLocation.getBits());
    helloCar->setSpeed(speed);
    helloCar->setDirection(direction);
    helloCar->setVertexA(vertexA);
    helloCar->setVertexB(vertexB);
    helloCar->setDistanceToVertexA(distanceToVertexA);
    return helloCar;
}

/*!
 * @brief Procesar mensaje HOLA_VEHIC.
 *
 * @param helloCar [in] Mensaje a procesar.
 */
void RoutingProtocolCar::processHelloCar(const inet::Ptr<HelloCar> &helloCar) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processHelloHost");

    /*
     * Se obtienen los datos del mensaje.
     */
    const inet::Ipv6Address &srcAddress = helloCar->getSrcAddress();
    GeohashLocation geohashLocation(helloCar->getGeohash(), 12);
    double speed = helloCar->getSpeed();
    double direction = helloCar->getDirection();
    Vertex vertexA = (Vertex) helloCar->getVertexA();
    Vertex vertexB = (Vertex) helloCar->getVertexB();
    double distanceToVertexA = helloCar->getDistanceToVertexA();
    const Graph &graph =
            roadNetworkDatabase->getRoadNetwork(geohashLocation)->getGraph();
    Edge edge = boost::edge(vertexA, vertexB, graph).first;
    double distanceToVertexB = graph[edge].length - distanceToVertexA;
    LocationOnRoadNetwork locationOnRoadNetwork = { edge, 0, distanceToVertexA,
            distanceToVertexB };

    /*
     * Se guarda el registro en el directorio de vehículos vecinos,
     * y se revisa si ya existe una ruta para este, en cuyo caso,
     * se actualiza la hora de expiración.
     * Si no existe, se crea una.
     */
    neighbouringCars.getMap()[srcAddress].expiryTime = omnetpp::simTime()
            + neighbouringCarValidityTime;
    neighbouringCars.getMap()[srcAddress].value = { geohashLocation, speed,
            direction, locationOnRoadNetwork };
    neighbouringCarsByEdge.insert(NeighbouringCarByEdge(edge, srcAddress));    // TODO Revisar si es necesario.
    inet::Ipv6Route *route =
            const_cast<inet::Ipv6Route*>(routingTable->doLongestPrefixMatch(
                    srcAddress));
    if (route != nullptr)
        route->setExpiryTime(omnetpp::simTime() + routeValidityTime);
    else {
        route = new inet::Ipv6Route(srcAddress, 128,
                inet::IRoute::SourceType::MANET);
        route->setNextHop(srcAddress);
        route->setInterface(networkInterface);
        route->setMetric(1);
        route->setExpiryTime(omnetpp::simTime() + routeValidityTime);
        routingTable->addRoute(route);
    }

    // @formatter:off
    EV_INFO << "Address: " << srcAddress.str() << std::endl
            << "Geohash location: " << geohashLocation.getGeohashString() << std::endl
            << "Speed: " << speed << std::endl
            << "Adjacency: " << direction << std::endl
            << "Vertex A: " << vertexA << std::endl
            << "Vertex B: " << vertexB << std::endl
            << "Edge: " << edge << std::endl
            << "Distance to vertex A: " << distanceToVertexA << std::endl
            << "Distance to vertex B: " << distanceToVertexB << std::endl;

    EV_DEBUG << "Number of car neighbours: " << neighbouringCars.getMap().size()
             << std::endl;
                                                    // @formatter:on

    showRoutes();
    schedulePurgeNeighbouringCarsTimer();    // TODO Revisar si es necesario.
}

/*
 * Mensajes HOLA_HOST.
 */

/*!
 * @brief Procesar mensaje HOLA_HOST.
 *
 * @param helloHost [in] Mensaje a procesar.
 */
void RoutingProtocolCar::processHelloHost(
        const inet::Ptr<HelloHost> &helloHost) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processHelloHost");

    /*
     * Se obtienen los datos del mensaje.
     */
    inet::Ipv6Address srcAddress = helloHost->getAddress();
    GeohashLocation geohashLocation(helloHost->getGeohash(), 12);

    EV_INFO << "Address: " << srcAddress.str() << std::endl;
    EV_INFO << "Geohash location: " << helloHost->getGeohash() << std::endl;
    EV_INFO << "                : "
            << geohashLocation.getGeohashString()
            << std::endl;

    /*
     * Se guarda el registro en el directorio de _hosts_ vecinos,
     * y se revisa si ya existe una ruta para este, en cuyo caso,
     * se actualiza la hora de expiración.
     * Si no existe, se crea una.
     */
    neighbouringHosts.getMap()[srcAddress] = { omnetpp::simTime()
            + neighbouringHostValidityTime, geohashLocation };
    removeOldRoutes(omnetpp::simTime());
    inet::Ipv6Route *route =
            const_cast<inet::Ipv6Route*>(routingTable->doLongestPrefixMatch(
                    srcAddress));
    if (route != nullptr)
        route->setExpiryTime(omnetpp::simTime() + routeValidityTime);
    else {
        route = new inet::Ipv6Route(srcAddress, 128,
                inet::IRoute::SourceType::MANET);
        route->setNextHop(srcAddress);
        route->setInterface(networkInterface);
        route->setMetric(1);
        route->setExpiryTime(omnetpp::simTime() + routeValidityTime);
        routingTable->addRoute(route);
    }

    EV_INFO << "Number of host neighbours: "
            << neighbouringHosts.getMap().size()
            << std::endl;

    showRoutes();

    schedulePurgeNeighbouringHostsTimer();
}

/*
 * Mensajes PING.
 */

/*!
 * @brief Crear mensaje PING.
 *
 * @param srcAddress [in] Dirección del vehículo remitente.
 * @param pingVertex [in] Vértice de origen.
 * @param pongVertex [in] Vértice de destino.
 *
 * @return Mensaje PING.
 */
const inet::Ptr<Ping> RoutingProtocolCar::createPing(
        const inet::Ipv6Address &srcAddress, Vertex pingVertex,
        Vertex pongVertex) const {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolCar::createPing");

    const inet::Ptr<Ping> &ping = inet::makeShared<Ping>();

    // @formatter:off
    EV_INFO << "Source address: " << srcAddress.str() << std::endl
            << "Target vertex: " << pongVertex << std::endl
            << "Source vertex: " << pingVertex << std::endl;
                            // @formatter:on

    inet::Ipv6Address nextHopAddress =
            getRandomNeighbouringCarAddressAheadOnEdge(pongVertex);

    EV_INFO << "Next hop: " << nextHopAddress.str() << std::endl;

    if (nextHopAddress.isUnspecified())
        return nullptr;

    ping->setAddress(srcAddress);
    ping->setPingVertex(pingVertex);
    ping->setPongVertex(pongVertex);

    return ping;
}

/*!
 * @brief Procesar mensaje PING.
 *
 * Si el vehículo se encuentra en el vértice de destino,
 * se crea un mensaje PONG de respuesta y se transmite al vecino
 * que se encuentre más cerca del vértice de origen;
 * después, transmite un mensaje HOLA_VEHIC
 * indicando que la arista está activa.
 *
 * Si no se encuentra en el vértice de destino,
 * se busca el vecino que se encuentre más cerca del
 * vértice de destino y se retransmite el mensaje hacia este.
 *
 * Si no se encuentra un vecino cerca del vértice de destino,
 * se crea un mensaje PONG de respuesta con la bandera de error activa,
 * y se busca el vecino que se encuentre más cerca del vertice
 * de origen para transmitir el mensaje hacia este.
 *
 * Si no se encuentra un vecino cerca del vértice de origen,
 * se descarta el mensaje.
 *
 * @param ping [in] Mensaje a procesar.
 */
void RoutingProtocolCar::processPing(const inet::Ptr<Ping> &ping) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processPing");

    /*
     * Se extraen los datos del mensaje.
     */
    const inet::Ipv6Address &pingAddress = ping->getAddress();
    Vertex pingVertex = (Vertex) ping->getPingVertex();
    Vertex pongVertex = (Vertex) ping->getPongVertex();
    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    Edge edge = boost::edge(pingVertex, pongVertex, graph).first;

    EV_INFO << "Address: " << pingAddress.str() << std::endl;
    EV_INFO << "Edge: " << edge << std::endl;
    EV_INFO << "Target vertex: " << pongVertex << std::endl;

    /*
     * Si se encuentra en el vértice de destino se guarda el estatus
     * de la arista y se responde con un mensaje PONG.
     */
    const LocationOnRoadNetwork &locationOnRoadNetwork =
            mobility->getLocationOnRoadNetwork();
    const inet::Ipv6Address &primaryUnicastAddress =
            configurator->getUnicastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
    const inet::Ipv6Address &primaryMulticastAddress =
            configurator->getMulticastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
    if (inVertex(locationOnRoadNetwork, pongVertex, graph)) {
        edgesStatus.getMap()[edge].expiryTime = omnetpp::simTime()
                + edgeStatusValidityTime;
        edgesStatus.getMap()[edge].value = true;

        /*
         * Se busca el vecino más cercano al vértice de origen y se le envía
         * el mensaje PONG de respuesta.
         */
        const inet::Ipv6Address &pongNextHopAddress =
                findNeighbouringCarClosestToVertex(pingVertex);
        if (!pongNextHopAddress.isUnspecified()) {
            const inet::Ptr<Pong> pong = createPong(pingAddress, false,
                    pingVertex, pongVertex);
            sendRoutingMessage(pong, "PONG", primaryMulticastAddress,
                    pongNextHopAddress);
        }

        /*
         * Se crea el mensaje HOLA_VEHIC y se transmite a la dirección
         * _multicast_ para indicar a los vecinos que la arista está activa.
         */
        const inet::Ptr<HelloCar> helloCar = createHelloCar(
                primaryUnicastAddress);
        helloCar->setPingPong(true);
        helloCar->setPingPongError(false);
        helloCar->setPingVertex(pingVertex);
        helloCar->setPongVertex(pongVertex);
        sendRoutingMessage(helloCar, "HOLA_VEHIC", primaryUnicastAddress,
                primaryMulticastAddress);

        /*
         * Si no se encuentra en el vértice de destino, se busca el vecino
         * más cercano a este y se usa como siguiente salto.
         */
    } else {
        const inet::Ipv6Address &pingNextHopAddress =
                findNeighbouringCarClosestToVertex(pongVertex);
        if (!pingNextHopAddress.isUnspecified())
            sendRoutingMessage(ping, "PING", primaryMulticastAddress,
                    pingNextHopAddress);

        /*
         * Si no se encuentra un vehículo vecino más cercano al
         * vértice de destino, se busca el vecino más cercano
         * al vértice de origen y se le envía un mensaje PONG de error.
         */
        else {
            const inet::Ipv6Address &pongNextHopAddress =
                    findNeighbouringCarClosestToVertex(pingVertex);
            if (!pongNextHopAddress.isUnspecified()) {
                const inet::Ptr<Pong> pong = createPong(pingAddress, true,
                        pingVertex, pongVertex);
                sendRoutingMessage(pong, "PONG", primaryMulticastAddress,
                        pongNextHopAddress);
            }
        }
    }
}

/*
 * Mensajes PONG.
 */

/*!
 * @brief Crear mensaje PONG.
 * 
 * @param pingAddress [in] Dirección del vehículo que originó
 * el mensaje PING.
 * @param error [in] Bandera de error.
 * @param pingVertex [in] Vértice de origen.
 * @param pongVertex [in] Vértice de destino.
 *
 * @return Mensaje PONG.
 */
const inet::Ptr<Pong> RoutingProtocolCar::createPong(
        const inet::Ipv6Address &pingAddress, bool error, Vertex pingVertex,
        Vertex pongVertex) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::createPong");

    const inet::Ptr<Pong> &pong = inet::makeShared<Pong>();

    EV_INFO << "Destination address: " << pingAddress.str() << std::endl;
    EV_INFO << "Error: " << error << std::endl;
    EV_INFO << "Target vertex: " << pingVertex << std::endl;
    EV_INFO << "Source vertex: " << pongVertex << std::endl;

    pong->setError(error);
    pong->setDestAddress(pingAddress);
    pong->setPingVertex(pingVertex);
    pong->setPongVertex(pongVertex);

    return pong;
}

/*!
 * @brief Procesar mensaje PONG.
 *
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
    bool error = pong->getError();
    Vertex pingVertex = (Vertex) pong->getPingVertex();
    Vertex pongVertex = (Vertex) pong->getPongVertex();
    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    Edge edge = boost::edge(pingVertex, pongVertex, graph).first;

    EV_INFO << "Destination address: " << destAddress.str() << std::endl;
    EV_INFO << "Error: " << error << std::endl;
    EV_INFO << "Edge: " << edge;
    EV_INFO << "Target vertex: " << pingVertex << std::endl;

    /*
     * Se revisa si la dirección de destino es una dirección local,
     * y si es un mensaje PONG pendiente, se guarda el estatus de la arista,
     * y se transmite un mensaje HOLA_VEHIC para anunciarlo.
     */
    const inet::Ipv6Address &primaryUnicastAddress =
            configurator->getUnicastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
    const inet::Ipv6Address &primaryMulticastAddress =
            configurator->getMulticastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
    if (interfaceTable->isLocalAddress(destAddress)) {
        if (pendingPongs.getMap().count(edge)) {
            pendingPongs.getMap().erase(edge);
            edgesStatus.getMap()[edge].expiryTime = omnetpp::simTime()
                    + edgeStatusValidityTime;
            edgesStatus.getMap()[edge].value = !error;
            const inet::Ptr<HelloCar> helloCar = createHelloCar(
                    primaryUnicastAddress);
            helloCar->setPingPong(true);
            helloCar->setPingPongError(error);
            helloCar->setPingVertex(pingVertex);
            helloCar->setPongVertex(pongVertex);
            sendRoutingMessage(helloCar, "ANC_VEHIC", primaryUnicastAddress,
                    primaryMulticastAddress);

            schedulePurgeEdgesStatusTimer();
        }

        /*
         * Si no es una dirección local, y el destinatario es un vecino,
         * se retransmite el mensaje directamente hacia este.
         */
    } else {
        if (neighbouringCars.getMap().count(destAddress)) {
            sendRoutingMessage(pong, "PONG", primaryUnicastAddress,
                    destAddress);

            /*
             * En otro caso, se selecciona como siguiente salto
             * un vehículo vecino más cercano al vértice de origen.
             */
        } else {
            inet::Ipv6Address nextHopAddress =
                    findNeighbouringCarClosestToVertex(pingVertex);
            sendRoutingMessage(pong, "PONG", primaryUnicastAddress,
                    nextHopAddress);
        }
    }
}

/*
 * Directorio de vehículos vecinos.
 */

/*!
 * @brief Obtener vehículo vecino aleatorio en la misma arista.
 *
 * Obtiene aleatoriamente un vehículo vecino que se encuentra en la misma
 * arista, y que esté más cerca del vértice indicado.
 *
 * @param targetVertex [in] Vértice de referencia.
 *
 * @return Dirección IPv6 del vehículo vecino seleccionado.
 */
inet::Ipv6Address RoutingProtocolCar::getRandomNeighbouringCarAddressAheadOnEdge(
        Vertex targetVertex) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::getRandomNeighbouringCarOnEdgeAddress");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork =
            mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);

    double distanceToTargetVertex = getDistanceToVertex(
            mobility->getLocationOnRoadNetwork(), targetVertex, graph);
    ASSERT(distanceToTargetVertex != std::numeric_limits<double>::infinity());

    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    std::vector<inet::Ipv6Address> addresses;

    // Se agregan las direcciones de los vehículos que están adelante en la misma arista
    NeighbouringCarsByEdgeConstIterator neighbouringCarIt =
            neighbouringCarsByEdge.lower_bound(edge);
    NeighbouringCarsByEdgeConstIterator neighbouringCarEndIt =
            neighbouringCarsByEdge.upper_bound(edge);
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

/*!
 * @brief Buscar vehículo vecino más cercano a un vértice que se encuentra en
 * la misma arista.
 *
 * Se buscan los vehículos vecinos que circulan sobre la misma arista,
 * y se obtiene el que se encuentra a la menor distancia del vértice
 * indicado.
 *
 * @param vertex [in] Vértice de referencia.
 *
 * @return Dirección IPv6 del vecino encontrado, o `::/128`
 * si no se encuentra ninguno.
 *
 * TODO Arreglar.
 */
const inet::Ipv6Address& RoutingProtocolCar::findNeighbouringCarClosestToVertex(
        Vertex vertex) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::findNeighbouringCarClosestToVertex");

    /*
     * Se obtiene la distancia al vértice para comparar con los
     * vehículos vecinos.
     */
    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork =
            mobility->getLocationOnRoadNetwork();
    double minDistance = getDistanceToVertex(locationOnRoadNetwork, vertex,
            graph);

    /*
     * Se recorre el directorio de vehículos vecinos en busca de uno
     * que esté a una distancia menor del vértice.
     */
    double distance;
    NeighbouringCarsConstIterator it = neighbouringCars.getMap().begin();
    NeighbouringCarsConstIterator endIt = neighbouringCars.getMap().end();
    NeighbouringCarsConstIterator chosenIt = neighbouringCars.getMap().end();
    while (it != endIt) {
        distance = getDistanceToVertex(it->second.value.locationOnRoadNetwork,
                vertex, graph);
        if (minDistance > distance) {
            minDistance = distance;
            chosenIt = it;
        }
        it++;
    }

    if (chosenIt == endIt)
        return inet::Ipv6Address::UNSPECIFIED_ADDRESS;

    return chosenIt->first;
}

/*!
 * @brief Obtener la cantidad de vehículos vecinos que se encuentran
 * en la misma arista.
 *
 * @return Cantidad de vehículos vecinos que se encuentran en
 * la misma arista.
 */
int RoutingProtocolCar::getNeighbouringCarsOnEdgeCount() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::getNeighbouringCarsOnEdgeCount");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork =
            mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    int n = 0;

    showStatus();
    showNeighbouringCars();

    NeighbouringCarsConstIterator neighbouringCarsIt =
            neighbouringCars.getMap().begin();
    NeighbouringCarsConstIterator neighbouringCarsEndIt =
            neighbouringCars.getMap().end();
    while (neighbouringCarsIt != neighbouringCarsEndIt) {
        if (neighbouringCarsIt->second.value.locationOnRoadNetwork.edge == edge)
            n++;

        neighbouringCarsIt++;
    }

    return n;
}

/*
 * Directorio de _hosts_ vecinos.
 */

/*!
 * @brief Imrpimir el directorio de _hosts_ vecinos.
 *
 * TODO Implementar.
 */
void RoutingProtocolCar::showNeighbouringHosts() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::showNeighbouringHosts");

}

/*!
 * @brief Programar el temporizador de limpieza del directorio
 * de _hosts_ vecinos.
 */
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

/*!
 * @brief Procesar el temporizador de limpieza del directorio
 * de _hosts_ vecinos.
 */
void RoutingProtocolCar::processPurgeNeighbouringHostsTimer() {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processPurgeNeighbouringHostsTimer");

    neighbouringHosts.removeOldValues(omnetpp::simTime());
    removeOldRoutes(omnetpp::simTime());
    schedulePurgeNeighbouringHostsTimer();
}

/*
 * Estatus de las aristas.
 */

/*!
 * @brief Imprimir las aristas activas.
 */
void RoutingProtocolCar::showEdgesStatus() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::showEdgesStatus");

}

/*!
 * @brief Programar el temporizador de limpieza de aristas activas.
 */
void RoutingProtocolCar::schedulePurgeEdgesStatusTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::schedulePurgeEdgesStatusTimer");

    omnetpp::simtime_t nextExpiryTime = edgesStatus.getNextExpiryTime();

    EV_INFO << "Next expiry time: " << nextExpiryTime << std::endl;

    if (nextExpiryTime == omnetpp::SimTime::getMaxTime()) {
        if (purgeEdgesStatusTimer->isScheduled())
            cancelEvent(purgeEdgesStatusTimer);

    } else {
        if (!purgeEdgesStatusTimer->isScheduled())
            scheduleAt(nextExpiryTime, purgeEdgesStatusTimer);

        else if (purgeEdgesStatusTimer->getArrivalTime() != nextExpiryTime) {
            cancelEvent(purgeEdgesStatusTimer);
            scheduleAt(nextExpiryTime, purgeEdgesStatusTimer);
        }
    }
}

/*!
 * @brief Procesar el temporizador de limpieza de aristas activas.
 */
void RoutingProtocolCar::processPurgeEdgesStatusTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processPurgeEdgesStatusTimer");

    edgesStatus.removeOldValues(omnetpp::simTime());
    schedulePurgeEdgesStatusTimer();
}

/*
 * Datagramas demorados.
 */

/*!
 * @brief Imprimir los datagramas demorados.
 */
void RoutingProtocolCar::showDelayedDatagrams() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::showDelayedDatagrams");

}

/*!
 * @brief Programar el temporizador de limpieza de datagramas demorados.
 */
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

/*!
 * @brief Procesar el temporizador de limpieza de datagramas demorados.
 */
void RoutingProtocolCar::processPurgeDelayedDatagramsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::processPurgeDelayedDatagramsTimer");

    delayedDatagrams.removeOldValues(omnetpp::simTime());
    schedulePurgeDelayedDatagramsTimer();
}

/*
 * Operación ping-pong
 *
 */

/*!
 * @brief Iniciar una operación ping-pong.
 *
 * Se crea un mensaje PING y se transmite hacia el vehículo vecino
 * más cercano al vértice de destino.
 *
 * @param pingVertex [in] Vértice de origen.
 * @param pongVertex [in] Vértice de destino.
 */
void RoutingProtocolCar::startPingPong(const Vertex pingVertex,
        const Vertex pongVertex) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolCar::startPingPong");

    /*
     * Se crea el mensaje PING y se busca el vehículo vecino más cercano
     * al vértice de destino.
     * Después, se envía el mensaje a este.
     */
    const inet::Ipv6Address &primaryUnicastAddress =
            configurator->getUnicastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
    const inet::Ptr<Ping> ping = createPing(primaryUnicastAddress, pingVertex,
            pongVertex);
    const inet::Ipv6Address &nextHopAddress =
            findNeighbouringCarClosestToVertex(pongVertex);
    sendRoutingMessage(ping, "PING", primaryUnicastAddress, nextHopAddress);
}

/*
 * Mensajes PONG pendientes.
 */

/*!
 * @brief Imprimir los mensajes PONG pendientes.
 */
void RoutingProtocolCar::showPendingPongs() const {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolCar::showPendingPongs");

    PendingPongsConstIterator it = pendingPongs.getMap().begin();
    PendingPongsConstIterator endIt = pendingPongs.getMap().end();
    while (it != endIt) {
        // @formatter:off
        EV_INFO << "Ping vertex: " << it->second.value.pingVertex << std::endl
                << "Pong vertex: " << it->second.value.pongVertex << std::endl
                << "Expiry time: " << it->second.expiryTime << std::endl;
                                                                                        // @formatter:on
        it++;
    }
}

/*!
 * @brief Programar el temporizador de limpieza de mensajes
 * PONG pendientes.
 */
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

/*!
 * @brief Procesar el temporizador de limpieza de mensajes
 * PONG pendientes.
 */
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

/*!
 * @brief Enrutar datagrama.
 *
 * TODO Corregir la documentación.
 *
 * Se lleva a cabo el siguiente procedimiento:
 *
 * - Validar la cabecera de opciones de salto por salto
 * - Eliminar las rutas expiradas
 * - Verificar si existe una ruta para la dirección de destino del datagrama
 * - Si no existe una ruta
 *     - Calcular el vértice de destino local
 *     - Si el vértice de destino local es un vértice _gateway_, y se encuentra en este vértice
 *       - Seleccionar como siguiente salto un vehículo vecino que se encuentre en la subred adyacente y crear la ruta
 *     - Si no existen aristas no inactivas
 *       - Seleccionar como siguiente salto el vecino más cercano al destino.
 *     - Calcular una ruta vial hacia el vértice de destino local
 *     - Si se encuentra en el primer vértice de la ruta vial y la primera arista de esta no es una arista activa
 *         - Iniciar una operación ping-pong y demorar el datagrama
 *     - Calcular el tramo recto de aristas más largo
 *     - Seleccionar como siguiente salto el vehículo vecino más lejano en el tramo recto y crear la ruta
 *     - Si no se encontró el siguiente salto, elegir el vecino más cercano al vértice de destino local
 *     - Si no existe un vecino más cercano al vértice de destino local, descartar el datagrama
 *
 * Primero, se valida la cabecera de opciones de salto por salto.
 *
 * Revisa si existe en la tabla de enrutamiento una ruta hacia la
 * dirección de destino. Si no existe, se intenta descubrir y crear una
 * ruta. Si no se encuentra la ruta, se descarta el datagrama.
 *
 * @param datagram [in] Datagrama a enrutar.
 *
 * @return Resultado del enrutamiento.
 */
inet::INetfilter::IHook::Result RoutingProtocolCar::routeDatagram(
        inet::Packet *datagram) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::routeDatagram");

    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();

    /*********************************************************************************
     * Validación de la cabecera de opciones de salto por salto
     *********************************************************************************/
    if (!validateHopByHopOptionsHeader(datagram))
        return inet::INetfilter::IHook::Result::DROP;

    /*********************************************************************************
     * Verificar si ya existe una ruta
     *********************************************************************************/
    removeOldRoutes(omnetpp::simTime());
    inet::Ipv6Route *route =
            const_cast<inet::Ipv6Route*>(routingTable->doLongestPrefixMatch(
                    destAddress));
    if (route != nullptr) {
        if (omnetpp::simTime() < route->getExpiryTime())
            return inet::INetfilter::IHook::ACCEPT;

        else
            routingTable->deleteRoute(const_cast<inet::Ipv6Route*>(route));
    }

    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const Graph &graph = roadNetwork->getGraph();

// Se obtiene la ubicación vial del vehículo
    const LocationOnRoadNetwork &locationOnRoadNetwork =
            mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;

    /*********************************************************************************
     * Obtener vértices visitados
     *********************************************************************************/
    const TlvVisitedVerticesOption *visitedVerticesOption = findTlvOption<
            TlvVisitedVerticesOption>(datagram);
    ASSERT(visitedVerticesOption != nullptr);
    int numVisitedVertices =
            visitedVerticesOption->getVisitedVerticesArraySize();
    VertexSet visitedVertices;
    for (int i = 0; i < numVisitedVertices; i++)
        visitedVertices.insert(visitedVerticesOption->getVisitedVertices(i));

    /*********************************************************************************
     * Obtener aristas inactivas
     *********************************************************************************/
    int numEdgesStatus = this->edgesStatus.getMap().size();
    EdgesStatusConstIterator activeEdgeIt = this->edgesStatus.getMap().begin();
    EdgeSet edgesStatus;
    for (int i = 0;
            i < numEdgesStatus
                    && activeEdgeIt != this->edgesStatus.getMap().end();
            i++, activeEdgeIt++)
        edgesStatus.insert(activeEdgeIt->first);

    /*********************************************************************************
     * Calcular ruta más corta
     *********************************************************************************/
    ShortestPath shortestPath;
    shortestPath.computeShortestPath(edge, graph, visitedVertices, edgesStatus);

    /*********************************************************************************
     * Obtener vértice destino local
     *********************************************************************************/
    Vertex destVertex = getLocalDestVertex(datagram, shortestPath);

// Se obtiene la ruta más corta al vértice de destino
    VertexVector shortestPathToDestVertex =
            shortestPath.getShortestPathToVertex(destVertex, graph);

// Se busca el siguiente salto
    inet::Ipv6Address nextHopAddress = findNextHop(shortestPathToDestVertex,
            shortestPath);

    if (!nextHopAddress.isUnspecified()) {
        route = new inet::Ipv6Route(destAddress, 128,
                inet::IRoute::SourceType::MANET);
        route->setNextHop(nextHopAddress);
        route->setInterface(networkInterface);
        route->setMetric(1);
        route->setExpiryTime(omnetpp::simTime() + routeValidityTime);
        routingTable->addRoute(route);

        return inet::INetfilter::IHook::Result::ACCEPT;
    }

    return inet::INetfilter::IHook::Result::DROP;
}

/*!
 * @brief Verificar la cabecera de opciones de salto por salto.
 *
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
    const TlvDestGeohashLocationOption *destGeohashLocationOption =
            findTlvOption<TlvDestGeohashLocationOption>(datagram);

    if (destGeohashLocationOption == nullptr)
        return false;

    GeohashLocation destGeohashLocation(destGeohashLocationOption->getGeohash(),
            12);

// Si el destino está en la misma región Geohash
    const GeohashLocation &geohashRegion =
            mobility->getRoadNetwork()->getGeohashRegion();
    if (geohashRegion.contains(destGeohashLocation)) {
        // Si el datagrama no incluye la opción de ubicación vial del destino, se agrega
        const TlvDestLocationOnRoadNetworkOption *destLocationOnRoadNetworkOption =
                findTlvOption<TlvDestLocationOnRoadNetworkOption>(datagram);
        if (destLocationOnRoadNetworkOption == nullptr) {
            setTlvDestLocationOnRoadNetworkOption(datagram,
                    destGeohashLocation);
        }
    }

// Si el datagrama no incluye la opción de vértices visitados, se agrega
    const TlvVisitedVerticesOption *visitedVerticesOption = findTlvOption<
            TlvVisitedVerticesOption>(datagram);
    if (visitedVerticesOption == nullptr) {
        TlvVisitedVerticesOption *visitedVerticesOption =
                createTlvVisitedVerticesOption();
        setTlvOption(datagram, visitedVerticesOption);
    }

    return true;
}

/*!
 * @brief Obtener el vértice de destino local.
 *
 * @param datagram [in] Datagrama a enrutar.
 * @param shortestPath [in] Rutas más cortas.
 *
 * @return Vértice de destino local.
 */
Vertex RoutingProtocolCar::getLocalDestVertex(inet::Packet *datagram,
        const ShortestPath &shortestPath) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::getLocalDestVertex");

    const TlvDestGeohashLocationOption *destGeohashLocationOption =
            findTlvOption<TlvDestGeohashLocationOption>(datagram);
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
        const TlvDestLocationOnRoadNetworkOption *locationOnRoadNetworkOption =
                findTlvOption<TlvDestLocationOnRoadNetworkOption>(datagram);
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
        const GeographicLib::GeoCoords &destLocation =
                destGeohashLocation.getLocation();
        const Bounds &localRegionBounds = geohashRegion.getBounds();

        VertexVector gatewayVertices;

        // Si el vértice se encuentra al norte de la regi��n local
        if (destLocation.Latitude() > localRegionBounds.getNorth()) {
            // Se obtienen los vértices gateway del norte
            const VertexVector &northGatewayVertices =
                    roadNetwork->getGatewayVertices(
                            GeohashLocation::Adjacency::NORTH);
            gatewayVertices.insert(gatewayVertices.end(),
                    northGatewayVertices.begin(), northGatewayVertices.end());

            // Si el vértice se encuentra al sur de la regi��n local
        } else if (destLocation.Longitude() < localRegionBounds.getSouth()) {
            // Se obtienen los vértices gateway del sur
            const VertexVector &southGatewayVertices =
                    roadNetwork->getGatewayVertices(
                            GeohashLocation::Adjacency::SOUTH);
            gatewayVertices.insert(gatewayVertices.end(),
                    southGatewayVertices.begin(), southGatewayVertices.end());
        }

        // Si el vértice se encuentra al este de la regi��n local
        if (destLocation.Longitude() > localRegionBounds.getEast()) {
            // Se obtienen los vértices gateway del este
            const VertexVector &eastGatewayVertices =
                    roadNetwork->getGatewayVertices(
                            GeohashLocation::Adjacency::EAST);
            gatewayVertices.insert(gatewayVertices.end(),
                    eastGatewayVertices.begin(), eastGatewayVertices.end());

            // Si el vértice se encuentra al oeste de la regi��n local
        } else if (destLocation.Latitude() < localRegionBounds.getWest()) {
            // Se obtienen los vértices gateway del oeste
            const VertexVector &westGatewayVertices =
                    roadNetwork->getGatewayVertices(
                            GeohashLocation::Adjacency::WEST);
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

/*!
 * @brief Se obtiene el conjunto de vértices visitados.
 *
 * @param visitedVerticesOption [in] Opción de vértices visitados.
 *
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
    int numVisitedVertices =
            visitedVerticesOption->getVisitedVerticesArraySize();
    for (int i = 0; i < numVisitedVertices; i++) {
        visitedVertex = visitedVerticesOption->getVisitedVertices(i);
        visitedVertices.insert(visitedVertex);
    }

    return visitedVertices;
}

/*!
 * @brief Obtener el vértice de destino.
 *
 * @param destGeohashLocation [in] Ubicación Geohash del destino.
 * @param destEdge [in] Arista de la ubicación del destno.
 * @param shortestPath [in] Rutas más cortas.
 *
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
        const GeohashLocation &geohashLocation =
                roadNetwork->getGeohashRegion();
        const GeographicLib::GeoCoords &A = mobility->getLocation();
        const GeographicLib::GeoCoords &B = destGeohashLocation.getLocation();

        // Se determina en qu������ direcci������n en latitud se encuentra la regi������n de destino
        GeohashLocation::Adjacency directionLat =
                GeohashLocation::Adjacency::NONE;
        if (B.Latitude() < A.Latitude())
            directionLat = GeohashLocation::Adjacency::SOUTH;
        else if (B.Latitude() > A.Longitude())
            directionLat = GeohashLocation::Adjacency::NORTH;

        // Se determina en qu������ direcci������n en longitud se encuentra la regi������n de destino
        GeohashLocation::Adjacency directionLon =
                GeohashLocation::Adjacency::NONE;
        if (B.Longitude() < A.Longitude())
            directionLon = GeohashLocation::Adjacency::WEST;
        else if (B.Longitude() > A.Longitude())
            directionLon = GeohashLocation::Adjacency::EAST;

        // Se obtienen los vértices gateway posibles
        VertexVector gatewayVertices;
        if (directionLat != GeohashLocation::Adjacency::NONE) {
            const VertexVector &gatewayVerticesLat =
                    roadNetwork->getGatewayVertices(directionLat);
            gatewayVertices.insert(gatewayVertices.end(),
                    gatewayVerticesLat.begin(), gatewayVerticesLat.end());
        }
        if (directionLat != GeohashLocation::Adjacency::NONE) {
            const VertexVector &gatewayVerticesLon =
                    roadNetwork->getGatewayVertices(directionLon);
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

/*!
 * @brief Obtener aristas en la ruta más corta que forman un tramo recto.
 *
 * Se obtienen las aristas en la ruta que forman el tramo largo más recto,
 * y en las que haya vehículos vecinos circulando.
 *
 * @param shortestPathToDestVertex [in] Ruta más corta al vértice
 * de destino.
 * @param shortestPath [in] Rutas más cortas.
 *
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

/*!
 * @brief Encontrar siguiente salto.
 *
 * Se obtiene el siguiente salto en la ruta.
 *
 * @param shortestPathToDestVertex [in] Ruta más corta al vértice
 * de destino.
 * @param shortestPath [in] Rutas más cortas.
 *
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
        GeohashLocation::Adjacency adjacency = graph[vertexB].adjacency;

        // Si el vértice de destino de la arista es un gateway
        if (adjacency != GeohashLocation::Adjacency::NONE) {
            const GeohashLocation &geohashRegion =
                    roadNetwork->getGeohashRegion();
            GeohashLocation adjacentGeohashRegion;
            geohashRegion.getAdjacentGeohashLocation(adjacency,
                    adjacentGeohashRegion);
            nextHopAddress = findNeighbourCarInAdjacentdRegion(
                    adjacentGeohashRegion);

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
    const LocationOnRoadNetwork &locationOnRoadNetwork =
            mobility->getLocationOnRoadNetwork();
    double distanceToVertexB = getDistanceToVertex(locationOnRoadNetwork,
            vertexB, graph);    // Distancia del vehículo al vértice B (puede ser infinito si el vértice está en otra arista)
    nextHopAddress = inet::Ipv6Address::UNSPECIFIED_ADDRESS;    // Dirección del vecino más cercano al vértice de destino de la arista
    NeighbouringCarsByEdgeConstIterator neighbouringCarIt =
            neighbouringCarsByEdge.lower_bound(edge);
    NeighbouringCarsByEdgeConstIterator neighbouringCarEndIt =
            neighbouringCarsByEdge.upper_bound(edge);

    while (neighbouringCarIt != neighbouringCarEndIt) {
        const inet::Ipv6Address &neighbouringCarAddress =
                neighbouringCarIt->second;
        const NeighbouringCar &neighbouringCar = neighbouringCars.getMap().at(
                neighbouringCarAddress);
        const LocationOnRoadNetwork &locationOnRoadNetwork =
                neighbouringCar.value.locationOnRoadNetwork;
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

/*!
 * @brief Obtener vehículo vecino en la región Geohash adyacente.
 *
 * @param neighbouringGeohashRegion [in] Región Geohash adyacente.
 *
 * @return Dirección IPv6 del vehículo vecino en la región Geohash indicada.
 */
inet::Ipv6Address RoutingProtocolCar::findNeighbourCarInAdjacentdRegion(
        const GeohashLocation &neighbouringGeohashRegion) const {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolCar::findNeighbourCarInAdjacentdRegion");

    NeighbouringCarsConstIterator it = neighbouringCars.getMap().begin();
    NeighbouringCarsConstIterator endIt = neighbouringCars.getMap().end();

    while (it != endIt) {
        const GeohashLocation &neighbouringCarGeohashLocation =
                it->second.value.geohashLocation;
        if (neighbouringGeohashRegion.contains(neighbouringCarGeohashLocation))
            return it->first;
    }

    return inet::Ipv6Address::UNSPECIFIED_ADDRESS;
}

/*
 * Estatus del vehículo.
 */

/*!
 * @brief Mostrar la dirección IPv6 del vehículo y su ubicación vial.
 */
void RoutingProtocolCar::showStatus() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::show");

    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork =
            mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertexA;
    const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertexB;

    // @formatter:off
    EV_INFO << "Address: " << configurator->getUnicastAddress(ConfiguratorBase::NetworkType::PRIMARY) << std::endl
            << "Edge: " << edge << std::endl
            << "Distance to vertex A: " << distanceToVertexA << std::endl
            << "Distance to vertex B: " << distanceToVertexB << std::endl;
                                                // @formatter:on
}

/*
 * Netfilter.
 */

/*!
 * @brief Procesar datagrama recibido de la capa inferior
 * antes de enrutarlo.
 *
 * @param datagram [in] Datagrama a procesar.
 *
 * @return Resultado del procesamiento.
 */
inet::INetfilter::IHook::Result RoutingProtocolCar::datagramPreRoutingHook(
        inet::Packet *datagram) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::datagramPreRoutingHook");
    EV_INFO << "Datagram: " << datagram->str() << std::endl;

    /*
     * Se obtiene la dirección de destino del datagrama para saber hacia
     * dónde enrutarlo.
     */
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address srcAddress = networkHeader->getSourceAddress().toIpv6();
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();

    // @formatter:off
    EV_INFO << "Source address: " << srcAddress.str() << std::endl
            << "Destination address: " << destAddress.str() << std::endl;
                                                // @formatter:on

    /*
     * Si la dirección de destino es una dirección local o si es _multicast_,
     * se acepta el datagrama..
     */
    if (interfaceTable->isLocalAddress(inet::L3Address(destAddress))
            || destAddress.isMulticast())
        return inet::INetfilter::IHook::ACCEPT;

    /*
     * Se enruta el datagrama.
     *
     * TODO Enrutar el datagrama.
     */
//return routeDatagram(datagram, destAddress);
    return inet::INetfilter::IHook::ACCEPT;
}

/*!
 * @brief Procesar datagrama recibido de la capa superior
 * antes de enrutarlo.
 *
 * @param datagram [in] Datagrama a procesar.
 *
 * @return Resultado del procesamiento.
 */
inet::INetfilter::IHook::Result RoutingProtocolCar::datagramLocalOutHook(
        inet::Packet *datagram) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolCar::datagramLocalOutHook");
    EV_INFO << "Datagram: " << datagram->str() << std::endl;

    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address srcAddress = networkHeader->getSourceAddress().toIpv6();
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();

    // @formatter:off
    EV_INFO << "Source address: " << srcAddress.str() << std::endl
            << "Destination address: " << destAddress.str() << std::endl;
                                                // @formatter:on

    return inet::INetfilter::IHook::ACCEPT;
}

/*
 * Lifecycle.
 */

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
    cancelAndDelete(purgeEdgesStatusTimer);
    cancelAndDelete(purgeDelayedDatagramsTimer);
    cancelAndDelete(purgePendingPongsTimer);
    neighbouringHosts.getMap().clear();
    edgesStatus.getMap().clear();
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
    cancelAndDelete(purgeEdgesStatusTimer);
    cancelAndDelete(purgeDelayedDatagramsTimer);
    cancelAndDelete(purgePendingPongsTimer);
    neighbouringHosts.getMap().clear();
    edgesStatus.getMap().clear();
}

/*
 * Notification.
 */

void RoutingProtocolCar::receiveSignal(omnetpp::cComponent *source,
        omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
        cObject *details) {

}
