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
#include <iterator>
#include <boost/stacktrace.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/tuple/tuple.hpp>

using namespace veins_proj;

Define_Module(RoutingProtocolCar);

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
 * @param srcAddress [in] Dirección del vehículo que trnasmite el mensaje.
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

    EV_DEBUG << "Address: "
             << srcAddress.str()
             << std::endl
             << "Geohash location: "
             << geohashLocation.getGeohash()
             << std::endl
             << "Speed: "
             << speed
             << std::endl
             << "Adjacency: "
             << direction
             << std::endl
             << "Vertex A: "
             << vertexA
             << std::endl
             << "Vertex B: "
             << vertexB
             << std::endl
             << "Distance to vertex A: "
             << distanceToVertexA
             << std::endl
             << "Distance to vertex B: "
             << distanceToVertexB
             << std::endl;

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

    EV_INFO << "Address: "
            << srcAddress.str()
            << std::endl
            << "Geohash location: "
            << geohashLocation.getGeohash()
            << std::endl
            << "Speed: "
            << speed
            << std::endl
            << "Adjacency: "
            << direction
            << std::endl
            << "Vertex A: "
            << vertexA
            << std::endl
            << "Vertex B: "
            << vertexB
            << std::endl
            << "Edge: "
            << edge
            << std::endl
            << "Distance to vertex A: "
            << distanceToVertexA
            << std::endl
            << "Distance to vertex B: "
            << distanceToVertexB
            << std::endl;

    EV_DEBUG << "Number of car neighbours: "
             << neighbouringCars.getMap().size()
             << std::endl;

    showRoutes();
    schedulePurgeNeighbouringCarsTimer();
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
            << geohashLocation.getGeohash()
            << std::endl;

    /*
     * Se guarda el registro en el directorio de *hosts* vecinos,
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

    EV_INFO << "Source address: "
            << srcAddress.str()
            << std::endl
            << "Target vertex: "
            << pongVertex
            << std::endl
            << "Source vertex: "
            << pingVertex
            << std::endl;

    inet::Ipv6Address nextHopAddress = findNextHopClosestToVertex(pongVertex);

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
    const inet::Ipv6Address &primaryUnicastAddress =
            configurator->getUnicastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
    const inet::Ipv6Address &primaryMulticastAddress =
            configurator->getMulticastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
    if (mobility->isAtVertex(pongVertex)) {
        edgesStatus.getMap()[edge].expiryTime = omnetpp::simTime()
                + edgeStatusValidityTime;
        edgesStatus.getMap()[edge].value = true;

        /*
         * Se busca el vecino más cercano al vértice de origen y se le envía
         * el mensaje PONG de respuesta.
         */
        const inet::Ipv6Address &pongNextHopAddress =
                findNextHopClosestToVertex(pingVertex);
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
                findNextHopClosestToVertex(pongVertex);
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
                    findNextHopClosestToVertex(pingVertex);
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
            inet::Ipv6Address nextHopAddress = findNextHopClosestToVertex(
                    pingVertex);
            sendRoutingMessage(pong, "PONG", primaryUnicastAddress,
                    nextHopAddress);
        }
    }
}

/*
 * Directorio de *hosts* vecinos.
 */

/*!
 * @brief Imrpimir el directorio de *hosts* vecinos.
 */
void RoutingProtocolCar::showNeighbouringHosts() const {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolCar::showNeighbouringHosts");

    EV_INFO << "Neighbouring hosts:" << std::endl;

    NeighbouringHostsConstIterator it = neighbouringHosts.getMap().begin();
    NeighbouringHostsConstIterator endIt = neighbouringHosts.getMap().end();
    while (it != endIt) {
        EV_INFO << "Address: "
                << it->first.str()
                << std::endl
                << "Expiry time: "
                << it->second.expiryTime
                << "s"
                << std::endl;
    }
}

/*!
 * @brief Programar el temporizador de limpieza del directorio
 * de *hosts* vecinos.
 */
void RoutingProtocolCar::schedulePurgeNeighbouringHostsTimer() {
    EV_DEBUG << "******************************************************************************************************************************************************************"
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
 * de *hosts* vecinos.
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
void RoutingProtocolCar::showDelayedDatagrams() const {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl
             << "RoutingProtocolCar::showDelayedDatagrams"
             << std::endl;
    Enter_Method
    ("RoutingProtocolCar::showDelayedDatagrams");

    DelayedDatagramsConstIterator it = delayedDatagrams.begin();
    DelayedDatagramsConstIterator endIt = delayedDatagrams.end();
    while (it != endIt) {
        const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
                inet::getNetworkProtocolHeader(it->second);
        inet::Ipv6Address destAddress =
                networkHeader->getDestinationAddress().toIpv6();

        EV_INFO << "Destination address: "
                << destAddress.str()
                << std::endl
                << "Datagram: "
                << it->second->str()
                << std::endl
                << "Piong-pong edge: "
                << it->first
                << std::endl;
        it++;
    }
}

/*!
 * @brief Eliminar los datagramas demorados cuyo mensaje PONG no llegó,
 * o no existe una ruta para enviarlos.
 */
void RoutingProtocolCar::removeStuckDelayedDatagrams() {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl
             << "RoutingProtocolCar::removeStuckDelayedDatagrams"
             << std::endl;
    Enter_Method
    ("RoutingProtocolCar::removeStuckDelayedDatagrams");

    DelayedDatagramsIterator it = delayedDatagrams.begin();
    DelayedDatagramsIterator endIt = delayedDatagrams.end();
    while (it != endIt) {
        if (pendingPongs.getMap().count(it->first))
            it = delayedDatagrams.erase(it);
        else
            it++;
    }
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
 * @return `true` si se pudo iniciar la operación ping-pong.
 */
bool RoutingProtocolCar::startPingPong(const Vertex pingVertex,
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
    const inet::Ipv6Address &nextHopAddress = findNextHopClosestToVertex(
            pongVertex);
    if (nextHopAddress.isUnspecified())
        return false;

    sendRoutingMessage(ping, "PING", primaryUnicastAddress, nextHopAddress);
    return true;
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
        EV_INFO << "Ping vertex: "
                << it->second.value.pingVertex
                << std::endl
                << "Pong vertex: "
                << it->second.value.pongVertex
                << std::endl
                << "Expiry time: "
                << it->second.expiryTime
                << std::endl;

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
    removeStuckDelayedDatagrams();
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
 *     - Calcular las rutas más cortas para obtener el vértice de destino local
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
 * @return Resultado del enrutamiento.
 */
inet::INetfilter::IHook::Result RoutingProtocolCar::routeDatagram(
        inet::Packet *datagram) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::routeDatagram");

    /*
     * Se valida la cabecera de opciones de salto por salto para
     * garantizar que el datagrama incluye toda la información
     * necesaria para realizar el enrutamiento.
     */
    if (!validateHopByHopOptionsHeader(datagram)) {
        if (hasGUI())
            inet::getContainingNode(host)->bubble(
                    "Hop by hop options header not correct, dropping packet");
        return inet::INetfilter::IHook::Result::DROP;
    }
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    /*
     * Se eliminan las rutas viejas y se obtiene la dirección de destino
     * para verificar si existe una ruta para esta.
     */
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();
    removeOldRoutes(omnetpp::simTime());
    // TODO Agregar nuevos vértices visidatos del siguiente salto al datagrama.
    if (routingTable->doLongestPrefixMatch(destAddress))
        return inet::INetfilter::IHook::ACCEPT;
    /*
     * Se obtienen los datos de la cabecera del datagrama y el conjunto
     * de aristas activas para poder calcular las rutas más cortas.
     */
    VertexSet visitedVertices = getVisitedVertices(datagram);
    EdgeSet activeEdges;
    edgesStatus.removeOldValues(omnetpp::simTime());
    EdgesStatusConstIterator it = edgesStatus.getMap().begin();
    EdgesStatusConstIterator endIt = edgesStatus.getMap().end();
    while (it != endIt) {
        if (it->second.value)
            activeEdges.insert(it->first);
        it++;
    }
    /*
     * Se calcula la ruta vial más corta al resto de los vértices
     * para poder obtener el vértice de destino local,
     * y después se obtiene la ruta vial más corta hacia este.
     * Si no se encontró una ruta vial, se descarta el datagrama.
     */
    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    const Edge &edge = mobility->getLocationOnRoadNetwork().edge;
    ShortestPaths shortestPaths;
    shortestPaths.computeShortestPath(edge, graph, visitedVertices,
            activeEdges);
    Vertex localDestVertex;
    bool localDestVertexFound;
    boost::tie(localDestVertex, localDestVertexFound) = getLocalDestVertex(
            datagram, shortestPaths);
    if (!localDestVertexFound) {
        // TODO Seleccionar vehículo más cercano a la ubicación del destino.
        if (hasGUI())
            inet::getContainingNode(host)->bubble(
                    "No local destination vertex found, dropping packet");
        return inet::INetfilter::IHook::DROP;
    }
    /*
     * Si la ruta más corta únicamente contiene dos vértices,
     * el segundo es el vértice de destino local.
     * En este caso, si el vértice de destino local es *gateway*,
     * se selecciona como siguiente salto un vehículo vecino
     * en la subred adyacente y se crea la ruta hacia este.
     * También se elimina la opción de vértices visitados,
     * ya que estos no son válidos en la siguiente subred.
     * Si no se encuentra el siguiente salto, se descarta el datagrama.
     */
    VertexVector shortestPath = shortestPaths.getShortestPathToVertex(
            localDestVertex, graph);
    if (shortestPath.size() == 2) {
        ASSERT(shortestPath[1] == localDestVertex);
        if (graph[localDestVertex].adjacency
                != GeohashLocation::Adjacency::NONE) {
            const inet::Ipv6Address &nextHopAddress =
                    findNextHopInAdjacentNetwork(
                            graph[localDestVertex].adjacency);
            if (!nextHopAddress.isUnspecified()) {
                inet::Ipv6Route *route = new inet::Ipv6Route(destAddress, 128,
                        inet::IRoute::SourceType::MANET);
                route->setNextHop(nextHopAddress);
                route->setInterface(networkInterface);
                route->setMetric(1);
                route->setExpiryTime(omnetpp::simTime() + routeValidityTime);
                routingTable->addRoute(route);
                // TODO Eliminar opción de vértices visitados.
                return inet::INetfilter::IHook::ACCEPT;
            } else {
                // TODO Seleccionar vehículo más cercano a la ubicación del destino.
                if (hasGUI())
                    inet::getContainingNode(host)->bubble(
                            "No next hop found, dropping packet");
                return inet::INetfilter::IHook::DROP;
            }
        }
    }
    /*
     * Si no se conoce el estatus de la segunda arista en la ruta vial,
     * se elige como siguiente salto el vecino más cercano
     * al segundo vértice de la ruta vial
     * Si no se encontró el siguiente salto, o el vehículo se encuentra
     * en el segudo vértice de la ruta vial, se demora el datagrama
     * y se inicia una operación ping-pong si no hay una en curso.
     * Si no se inició con éxito la operación ping-pong,
     * se elimina el datagrama de los datagramas demorados.
     * cuyo destino es el tercer vértice de la ruta vial.
     * De otro modo, se crea una ruta hacia el siguiente salto
     * que se encontró previamente.
     */
    const Edge secondEdge =
            boost::edge(shortestPath[1], shortestPath[2], graph).first;
    if (!edgesStatus.getMap().count(secondEdge)) {
        const inet::Ipv6Address &nextHopAddress = findNextHopClosestToVertex(
                shortestPath[1]);
        if (nextHopAddress.isUnspecified()
                || mobility->isAtVertex(shortestPath[1])) {
            delayedDatagrams.insert(
                    std::pair<Edge, inet::Packet*>(secondEdge, datagram));
            if (!pendingPongs.getMap().count(secondEdge)) {
                if (startPingPong(shortestPath[1], shortestPath[2])) {
                    return inet::INetfilter::IHook::QUEUE;
                } else {
                    delayedDatagrams.erase(secondEdge);
                    // TODO Seleccionar vehículo más cercano a la ubicación del destino.
                    if (hasGUI())
                        inet::getContainingNode(host)->bubble(
                                "No next hop found, dropping packet");
                    return inet::INetfilter::IHook::DROP;
                }
            }
        } else {
            inet::Ipv6Route *route = new inet::Ipv6Route(destAddress, 128,
                    inet::IRoute::SourceType::MANET);
            route->setNextHop(nextHopAddress);
            route->setInterface(networkInterface);
            route->setMetric(1);
            route->setExpiryTime(omnetpp::simTime() + routeValidityTime);
            routingTable->addRoute(route);
            // TODO Agregar nuevos vértices visitados a la ruta y al datagrama.
            return inet::INetfilter::IHook::ACCEPT;
        }
    }
    /*
     * Se busca el tramo recto de aristas más largo posible
     * en el que existan vehículos vecinos,
     * y se selecciona como siguiente salto el vecino más lejano
     * para lograr que el datagrama haga el mayor avance posible,
     * y se crea la ruta.
     * Si no se encuentra el siguiente salto, se descarta el datagrama.
     */
    const inet::Ipv6Address &nextHopAddress = findNextHopFurthestInStraightPath(
            shortestPath, shortestPaths);
    if (!nextHopAddress.isUnspecified()) {
        inet::Ipv6Route *route = new inet::Ipv6Route(destAddress, 128,
                inet::IRoute::SourceType::MANET);
        route->setNextHop(nextHopAddress);
        route->setInterface(networkInterface);
        route->setMetric(1);
        route->setExpiryTime(omnetpp::simTime() + routeValidityTime);
        routingTable->addRoute(route);
        // TODO Agregar nuevos vértices visitados a la ruta y al datagrama.
        return inet::INetfilter::IHook::ACCEPT;
    } else {
        // TODO Seleccionar vehículo más cercano a la ubicación del destino.
        if (hasGUI())
            inet::getContainingNode(host)->bubble(
                    "No next hop found, dropping packet");
        return inet::INetfilter::IHook::DROP;
    }

    return inet::INetfilter::IHook::Result::ACCEPT;
}

/*!
 * @brief Verificar la cabecera de opciones de salto por salto.
 *
 * Verifica si la cabecera tiene la opción de ubicación del destino.
 *
 * Si el destino se encuentra en la misma subred, se verifica si
 * la cabecera contiene la opción de ubicación vial del destino. Si no
 * la tiene, se agrega.
 *
 * Si la cabecera no contiene la opción de vértices visitados, se agrega.
 */
bool RoutingProtocolCar::validateHopByHopOptionsHeader(
        inet::Packet *datagram) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::validateHopByHopOptionsHeader");

    /*
     * Se verifica la opción de ubicación del destino.
     */
    const TlvDestGeohashLocationOption *destGeohashLocationOption =
            findTlvOption<TlvDestGeohashLocationOption>(datagram);
    if (destGeohashLocationOption == nullptr)
        return false;

    /*
     * Si el destino está en la misma región Geohash,
     * se calcula su ubicación vial y se agrega la opción de
     * ubicación vial del destino si hace falta.
     */
    GeohashLocation destGeohashLocation(destGeohashLocationOption->getGeohash(),
            12);
    const GeohashLocation &geohashRegion =
            mobility->getRoadNetwork()->getGeohashRegion();
    if (geohashRegion.contains(destGeohashLocation)) {
        const TlvDestLocationOnRoadNetworkOption *destLocationOnRoadNetworkOption =
                findTlvOption<TlvDestLocationOnRoadNetworkOption>(datagram);
        if (destLocationOnRoadNetworkOption == nullptr)
            setTlvDestLocationOnRoadNetworkOption(datagram,
                    destGeohashLocation);
    }

    /*
     * Si la cabecera no tiene la opción de vértices visitados,
     * se agrega.
     */
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
 * @brief Se obtiene el conjunto de vértices visitados.
 *
 * @param visitedVerticesOption [in] Opción de vértices visitados.
 * @return Conjunto de vértices visitados.
 */
VertexSet RoutingProtocolCar::getVisitedVertices(inet::Packet *datagram) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::getVisitedVertices");

    const TlvVisitedVerticesOption *visitedVerticesOption = findTlvOption<
            TlvVisitedVerticesOption>(datagram);
    ASSERT(visitedVerticesOption != nullptr);
    int numVisitedVertices =
            visitedVerticesOption->getVisitedVerticesArraySize();
    VertexSet visitedVertices;
    for (int i = 0; i < numVisitedVertices; i++)
        visitedVertices.insert(visitedVerticesOption->getVisitedVertices(i));

    return visitedVertices;
}

/*!
 * @brief Obtener el vértice de destino local.
 *
 * @param datagram [in] Datagrama a enrutar.
 * @param shortestPaths [in] Rutas más cortas.
 * @return Vértice de destino local.
 */
std::pair<Vertex, bool> RoutingProtocolCar::getLocalDestVertex(
        inet::Packet *datagram, const ShortestPaths &shortestPaths) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::getLocalDestVertex");

    /*
     * Se obtiene la ubicación del destino para saber si se encuentra
     * en la misma subred o en otra.
     */
    const TlvDestGeohashLocationOption *destGeohashLocationOption =
            findTlvOption<TlvDestGeohashLocationOption>(datagram);
    ASSERT(destGeohashLocationOption != nullptr);
    GeohashLocation destGeohashLocation(destGeohashLocationOption->getGeohash(),
            12);
    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const GeohashLocation &geohashRegion = roadNetwork->getGeohashRegion();
    Vertex localDestVertex;
    bool localDestVertexFound = false;
    /*
     * Si la ubicación del destino se encuentra en la misma región,
     * se obtiene su ubicación vial de la cabecera del datagrama
     * para buscar determinar cuál de los dos vértices de la arista
     * en la que se encuentra será el vertice de destino local.
     * Si no se encuentra una ruta hacia ninguno de los dos vértices,
     * `destVertexFound` vale `false`.
     */
    if (geohashRegion.contains(destGeohashLocation)) {
        const TlvDestLocationOnRoadNetworkOption *destLocationOnRoadNetworkOption =
                findTlvOption<TlvDestLocationOnRoadNetworkOption>(datagram);
        ASSERT(destLocationOnRoadNetworkOption != nullptr);
        Vertex vertexA = (Vertex) destLocationOnRoadNetworkOption->getVertexA();
        Vertex vertexB = (Vertex) destLocationOnRoadNetworkOption->getVertexB();
        double routeDistanceA = shortestPaths.getRouteDistance(vertexA);
        double routeDistanceB = shortestPaths.getRouteDistance(vertexB);
        localDestVertex = vertexA;
        localDestVertexFound =
                shortestPaths.routeToVertexFound(vertexA) ? true : false;
        if (routeDistanceB < routeDistanceA) {
            localDestVertex = vertexB;
            localDestVertexFound =
                    shortestPaths.routeToVertexFound(vertexB) ? true : false;
        }
        /*
         * Si la ubicación del destino se encuentra en otra región Geohash,
         * se obtiene su ubicación y se determina hacia qué dirección se
         * encuentra la región en la que se encuentra para obtener
         * el vértice *gateway* que será el vértice de destino local.
         */
    } else {
        const GeographicLib::GeoCoords &destLocation =
                destGeohashLocation.getLocation();
        const Bounds &localRegionBounds = geohashRegion.getBounds();
        VertexVector tentativeGatewayVertices;
        /*
         * Si el destino se encuentra al norte de la región local,
         * se agregan como vértices *gateway* tentativos los vértices
         * con adyacencia al norte.
         */
        if (destLocation.Latitude() > localRegionBounds.getNorth()) {
            const VertexVector &northGatewayVertices =
                    roadNetwork->getGatewayVertices(
                            GeohashLocation::Adjacency::NORTH);
            tentativeGatewayVertices.insert(tentativeGatewayVertices.end(),
                    northGatewayVertices.begin(), northGatewayVertices.end());
            /*
             * Si el destino se encuentra al norte de la región local,
             * se agregan como vértices *gateway* tentativos los vértices
             * con adyacencia al sur.
             */
        } else if (destLocation.Longitude() < localRegionBounds.getSouth()) {
            const VertexVector &southGatewayVertices =
                    roadNetwork->getGatewayVertices(
                            GeohashLocation::Adjacency::SOUTH);
            tentativeGatewayVertices.insert(tentativeGatewayVertices.end(),
                    southGatewayVertices.begin(), southGatewayVertices.end());
        }
        /*
         * Si el destino se encuentra al norte de la región local,
         * se agregan como vértices *gateway* tentativos los vértices
         * con adyacencia al este.
         */
        if (destLocation.Longitude() > localRegionBounds.getEast()) {
            const VertexVector &eastGatewayVertices =
                    roadNetwork->getGatewayVertices(
                            GeohashLocation::Adjacency::EAST);
            tentativeGatewayVertices.insert(tentativeGatewayVertices.end(),
                    eastGatewayVertices.begin(), eastGatewayVertices.end());
            /*
             * Si el destino se encuentra al norte de la región local,
             * se agregan como vértices *gateway* tentativos los vértices
             * con adyacencia al oeste.
             */
        } else if (destLocation.Latitude() < localRegionBounds.getWest()) {
            const VertexVector &westGatewayVertices =
                    roadNetwork->getGatewayVertices(
                            GeohashLocation::Adjacency::WEST);
            tentativeGatewayVertices.insert(tentativeGatewayVertices.end(),
                    westGatewayVertices.begin(), westGatewayVertices.end());
        }
        /*
         * Se busca dentro de los vértices *gateway* tentativos el que
         * tenga la menor distancia de ruta y se selecciona como
         * vértice de destino local.
         */
        double minRouteDistance = std::numeric_limits<double>::infinity();
        VertexVectorIterator it = tentativeGatewayVertices.begin();
        VertexVectorIterator endIt = tentativeGatewayVertices.end();
        while (it != endIt) {
            double routeDistance = shortestPaths.getRouteDistance(*it);
            if (minRouteDistance > routeDistance) {
                localDestVertex = *it;
                localDestVertexFound = true;
            }

            it++;
        }
    }

    return std::pair<Vertex, bool>(localDestVertex, localDestVertexFound);
}

/*!
 * @brief Obtener aristas en la ruta más corta que forman un tramo recto.
 *
 * Se obtienen las aristas en la ruta que forman el tramo largo más recto,
 * y en las que haya vehículos vecinos circulando.
 *
 * @param shortestPathToDestVertex [in] Ruta más corta al vértice
 * de destino.
 * @param shortestPaths [in] Rutas más cortas.
 * @return Aristas que forman un
 */
EdgeVector RoutingProtocolCar::getReachableEdges(
        const VertexVector &shortestPathToDestVertex,
        const ShortestPaths &shortestPaths) const {
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

            double distance = shortestPaths.getRouteDistance(vertexB);

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
 * @brief Obtener vehículo vecino en la región Geohash adyacente.
 *
 * @param neighbouringGeohashRegion [in] Región Geohash adyacente.
 * @return Dirección IPv6 del siguiente salto, o `::/128`.
 * si no se encuentra ninguno.
 */
const inet::Ipv6Address& RoutingProtocolCar::findNextHopInAdjacentNetwork(
        const GeohashLocation::Adjacency adjacencyDirection) const {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl
             << "RoutingProtocolCar::findNextHopInAdjacentNetwork"
             << std::endl;
    Enter_Method
    ("RoutingProtocolCar::findNextHopInAdjacentNetwork");

    GeohashLocation adjacentGeohashRegion =
            mobility->getRoadNetwork()->getGeohashRegion().getAdjacentGeohashRegion(
                    adjacencyDirection);
    NeighbouringCarsConstIterator it = neighbouringCars.getMap().begin();
    NeighbouringCarsConstIterator endIt = neighbouringCars.getMap().end();
    while (it != endIt) {
        const GeohashLocation &neighbouringCarGeohashLocation =
                it->second.value.geohashLocation;
        if (adjacentGeohashRegion.contains(neighbouringCarGeohashLocation))
            return it->first;
    }

    return inet::Ipv6Address::UNSPECIFIED_ADDRESS;
}

/*!
 * @brief Encontrar siguiente salto más cercano a una ubicación.
 *
 * @return Dirección IPv6 del siguiente salto, o `::/128`.
 * si no se encuentra ninguno.
 */
const inet::Ipv6Address& RoutingProtocolCar::findNextHopClosestToLocation(
        const GeohashLocation &geohashLocation) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl
            << "RoutingProtocolCar::findNextHopClosestToLocation"
            << std::endl;
    Enter_Method
    ("RoutingProtocolCar::findNextHopClosestToLocation");

    /*
     * Se recorre el directorio de vehículos vecinos en busca de uno
     * que esté a una distancia menor de la ubiación.
     */
    double minDistance = std::numeric_limits<double>::infinity();
    double distance;
    NeighbouringCarsConstIterator it = neighbouringCars.getMap().begin();
    NeighbouringCarsConstIterator endIt = neighbouringCars.getMap().end();
    NeighbouringCarsConstIterator chosenIt = neighbouringCars.getMap().end();
    while (it != endIt) {
        distance = it->second.value.geohashLocation.getDistance(
                geohashLocation);
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
 * @brief Encontrar siguiente salto más lejano en el tramo más recto
 * de la ruta vial.
 *
 * @param shortestPath  [in] Ruta vial a un vértice.
 * @param shortestPsths [in] Rutas viales más cortas
 * @return Dirección IPv6 del siguiente salto, o `::/128`.
 * si no se encuentra ninguno.
 */
const inet::Ipv6Address& RoutingProtocolCar::findNextHopFurthestInStraightPath(
        const VertexVector &shortestPath,
        const ShortestPaths &shortestPaths) const {
    return inet::Ipv6Address::UNSPECIFIED_ADDRESS;

    /*
     * Se agrupan los vehículos vecinos según la arista
     * en la que se encuentran.
     */
    const Graph &graph = mobility->getRoadNetwork()->getGraph();
    typedef std::multimap<Edge, inet::Ipv6Address> NeighbouringCarsByEdge;
    NeighbouringCarsByEdge neighbouringCarsByEdge;
    NeighbouringCarsConstIterator it = neighbouringCars.getMap().begin();
    NeighbouringCarsConstIterator endIt = neighbouringCars.getMap().begin();
    while (it != endIt)
        neighbouringCarsByEdge.insert(
                std::pair<Edge, inet::Ipv6Address>(
                        it->second.value.locationOnRoadNetwork.edge,
                        it->first));
    /*
     * Se busca la última arista en la ruta vial que forme un tramo recto,
     * y en la que haya vehículos vecinos.
     */
    Vertex vertexA;
    Vertex vertexB;
    Edge edge;
    int lastVertex = 1;
    int i = 1;
    int n = shortestPath.size();
    while (i < n) {
        vertexA = shortestPath[i - 1];
        vertexB = shortestPath[i];
        if (shortestPaths.getRouteDistance(vertexB) > 15)
            break;
        edge = boost::edge(vertexA, vertexB, graph).first;
        if (neighbouringCarsByEdge.count(edge))
            lastVertex = i;
    }
    /*
     * Se busca el vehículo vecino en la última arista que se encuentre
     * más cerca del segundo vértice de esta.
     */
    vertexA = shortestPath[lastVertex - 1];
    vertexA = shortestPath[lastVertex];
    edge = boost::edge(vertexA, vertexB, graph).first;
    double minDistance = std::numeric_limits<double>::infinity();
    NeighbouringCarsByEdge::const_iterator it2 =
            neighbouringCarsByEdge.lower_bound(edge);
    NeighbouringCarsByEdge::const_iterator endIt2 =
            neighbouringCarsByEdge.upper_bound(edge);
    NeighbouringCarsByEdge::const_iterator chosenIt =
            neighbouringCarsByEdge.upper_bound(edge);
    while (it2 != endIt2) {
        const inet::Ipv6Address &address = it2->second;
        const LocationOnRoadNetwork &locationOnRoadNetwork =
                neighbouringCars.getMap().find(address)->second.value.locationOnRoadNetwork;
        double distance = getDistanceToVertex(locationOnRoadNetwork, vertexB,
                graph);
        if (minDistance > distance) {
            minDistance = distance;
            chosenIt = it2;
        }
        it2++;
    }

    if (chosenIt == endIt2)
        return inet::Ipv6Address::UNSPECIFIED_ADDRESS;

    return chosenIt->second;
}

/*!
 * @brief Buscar vehículo vecino más cercano a un vértice que
 * se encuentra en la misma arista.
 *
 * Se buscan los vehículos vecinos que circulan sobre la misma arista,
 * y se obtiene el que se encuentra a la menor distancia del vértice
 * indicado.
 *
 * @param vertex [in] Vértice de referencia.
 * @return Dirección IPv6 del siguiente salto, o `::/128`.
 * si no se encuentra ninguno.
 */
const inet::Ipv6Address& RoutingProtocolCar::findNextHopClosestToVertex(
        Vertex vertex) const {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl
             << "RoutingProtocolCar::findNextHopClosestToVertex"
             << std::endl;
    Enter_Method
    ("RoutingProtocolCar::findNextHopClosestToVertex");

    /*
     * Se obtiene la distancia del vehículo al vértice
     * para comparar con la distancia de cada vehículo vecino a este.
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

    const LocationOnRoadNetwork &locationOnRoadNetwork =
            mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertexA;
    const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertexB;

    EV_INFO << "Address: "
            << configurator->getUnicastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY)
            << std::endl
            << "Edge: "
            << edge
            << std::endl
            << "Distance to vertex A: "
            << distanceToVertexA
            << std::endl
            << "Distance to vertex B: "
            << distanceToVertexB
            << std::endl;
}

/*
 * Netfilter.
 */

/*!
 * @brief Procesar datagrama recibido de la capa inferior
 * antes de enrutarlo.
 *
 * @param datagram [in] Datagrama a procesar.
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

    EV_INFO << "Source address: "
            << srcAddress.str()
            << std::endl
            << "Destination address: "
            << destAddress.str()
            << std::endl;

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
    //return routeDatagram(datagram);
    return inet::INetfilter::IHook::ACCEPT;
}

/*!
 * @brief Procesar datagrama recibido de la capa superior
 * antes de enrutarlo.
 *
 * @param datagram [in] Datagrama a procesar.
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

    EV_INFO << "Source address: "
            << srcAddress.str()
            << std::endl
            << "Destination address: "
            << destAddress.str()
            << std::endl;

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
