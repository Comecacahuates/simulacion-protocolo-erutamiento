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
 * @file RoutingProtocolBase.CC
 * @author Adrián Juárez Monroy
 */

#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/routing/RoutingProtocolBase.h"
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
#include "veins_proj/routing/RouteData.h"
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include <cmath>
#include <vector>
#include <limits>
#include <boost/stacktrace.hpp>

using namespace veins_proj;

Register_Abstract_Class(RoutingProtocolBase);

/*
 * Interfaz del módulo.
 */

/*!
 * @brief Inicialización.
 *
 * @param stage [in] Etapa de inicialización.
 */
void RoutingProtocolBase::initialize(int stage) {
    inet::RoutingProtocolBase::initialize(stage);

    /*
     * Etapa de inicialización local.
     */
    if (stage == inet::INITSTAGE_LOCAL) {
        /*
         * Parámetros de enrutamiento.
         */
        startTime = par("startTime");
        helloCarInterval = par("helloCarInterval");
        neighbouringCarValidityTime = par("neighbouringCarValidityTime");
        helloHostInterval = par("helloHostInterval");
        neighbouringHostValidityTime = par("neighbouringHostValidityTime");
        edgeStatusValidityTime = par("edgeStatusValidityTime");
        routeValidityTime = par("routeValidityTime");
        udpPacketDelayTime = par("udpPacketDelayTime");
        vertexProximityRadius = par("vertexProximityRadius");

        /*
         * Contexto.
         */
        host = inet::getContainingNode(this);
        interfaceTable = inet::L3AddressResolver().interfaceTableOf(host);
        if (!interfaceTable)
            throw omnetpp::cRuntimeError("No interface table found");
        routingTable = inet::L3AddressResolver().findIpv6RoutingTableOf(host);
        if (!routingTable)
            throw omnetpp::cRuntimeError("No routing table found");
        networkProtocol = inet::getModuleFromPar<inet::INetfilter>(
                par("networkProtocolModule"), this);
        if (!networkProtocol)
            throw omnetpp::cRuntimeError("No network protocol module found");
        roadNetworkDatabase = omnetpp::check_and_cast<RoadNetworkDatabase*>(
                getModuleByPath(par("roadNetworkDatabaseModule")));
        if (!roadNetworkDatabase)
            throw omnetpp::cRuntimeError("No roadway database module found");

        /*
         * Mensajes propios
         */
        purgeNeighbouringCarsTimer = new omnetpp::cMessage(
                "purgeNeighbouringCarsTimer");

        /*
         * Etapa de inicialización de interfaces de red.
         */
    } else if (stage == inet::INITSTAGE_NETWORK_INTERFACE_CONFIGURATION) {
        networkInterface = interfaceTable->findInterfaceByName(
                par("outputInterface"));
        if (!networkInterface)
            throw omnetpp::cRuntimeError("Output interface not found");

        /*
         * Etapa de inicialización de protocolos de enrutamiento.
         */
    } else if (stage == inet::INITSTAGE_ROUTING_PROTOCOLS) {
        inet::registerProtocol(inet::Protocol::manet, gate("ipOut"),
                gate("ipIn"));
        host->subscribe(inet::linkBrokenSignal, this);
        networkProtocol->registerHook(0, this);
    }
}

/*!
 * @brief Manejo de mensajes.
 *
 * @param message [in] Mensaje a procesar.
 */
void RoutingProtocolBase::handleMessageWhenUp(omnetpp::cMessage *message) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolBase::handleMessageWhenUp");

    if (message->isSelfMessage())
        processSelfMessage(message);
    else
        processMessage(message);
}

/*
 * Manejo de mensajes
 */

/*!
 * @brief Manejo de mensajes.
 *
 * @param message [in] Mensaje a procesar.
 */
void RoutingProtocolBase::processMessage(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::processMessage");

    inet::Packet *packet = omnetpp::check_and_cast<inet::Packet*>(message);

    if (packet != nullptr)
        processUdpPacket(packet);

    else
        throw omnetpp::cRuntimeError("Unknown message");

    delete packet;
}

/*!
 * @brief Manejo de mensajes propios.
 *
 * @param message [in] Mensaje a procesar.
 */
void RoutingProtocolBase::processSelfMessage(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::processSelfMessage");

    if (message == purgeNeighbouringCarsTimer)
        processPurgeNeighbouringCarsTimer();

    else
        throw omnetpp::cRuntimeError("Unknown self message");
}

/*
 * Paquetes UDP.
 */

/*!
 * @brief Enviar paquete UDP.
 *
 * Envía un paquete UDP hacia la compuerta que se conecta con el
 * protocolo IP.
 *
 * @param packet  [in] Paque a enviar.
 * @param delayed [in] Indica si la transmisión del paquete se va a demorar.
 * Se utiliza si se tienen que enviar dos paquetes consecutivamente.
 */
void RoutingProtocolBase::sendUdpPacket(inet::Packet *packet, bool delayed) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::sendUdpPacket");

    if (delayed)
        sendDelayed(packet, udpPacketDelayTime, "ipOut");
    else
        send(packet, "ipOut");
}

/*!
 * @brief Procesar paquete UDP.
 *
 * Identifica el tipo de paquete y lo envía a la función de procesamiento
 * correspondiente.
 *
 * @param packet [in] Paquete a procesar.
 */
void RoutingProtocolBase::processUdpPacket(inet::Packet *udpPacket) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::processUdpPacket");
    EV_INFO << "UDP packet: " << udpPacket->getName() << std::endl;

    udpPacket->popAtFront<inet::UdpHeader>();

    const inet::Ptr<const RoutingPacket> &routingPacket = udpPacket->popAtFront<
            RoutingPacket>();

    PacketType packetType = routingPacket->getPacketType();

    switch (packetType) {

        case PacketType::HELLO_CAR: {
            inet::Ptr<HelloCar> helloCar = inet::dynamicPtrCast<HelloCar>(
                    routingPacket->dupShared());
            processHelloCar(helloCar);

            break;
        }

        case PacketType::HELLO_HOST: {
            inet::Ptr<HelloHost> helloHost = inet::dynamicPtrCast<HelloHost>(
                    routingPacket->dupShared());
            processHelloHost(helloHost);

            break;
        }

        default:
            throw omnetpp::cRuntimeError(
                    "Routing packet arrived with undefined packet type: %d",
                    packetType);
    }
}

/*
 * Mensajes de enrutamiento.
 */

/*!
 * @brief Se encapsula el mensaje de enrutamiento en un datagrama UDP
 * y se envía a la dirección indicada.
 *
 * @param routingMessage [in] Mensaje a enviar.
 * @param name           [in] Nombre del mensaje.
 * @param srcAddress     [in] Dirección de origen del mensaje.
 * @param destAddress    [in] Dirección de destino del mensaje.
 * @param delayed        [in] Indica si la transmisión del paquete
 * se va a demorar. Se utiliza si se tienen que enviar
 * dos mensajes consecutivamente.
 */
void RoutingProtocolBase::sendRoutingMessage(
        const inet::Ptr<RoutingPacket> routingMessage, const char *name,
        const inet::Ipv6Address &srcAddress,
        const inet::Ipv6Address &destAddress, bool delayed) {

    inet::Packet *udpPacket = new inet::Packet(name);
    udpPacket->insertAtBack(routingMessage);

    inet::Ptr<inet::UdpHeader> udpHeader = inet::makeShared<inet::UdpHeader>();
    udpHeader->setSourcePort(ROUTING_PROTOCOL_UDP_PORT);
    udpHeader->setDestinationPort(ROUTING_PROTOCOL_UDP_PORT);
    udpPacket->insertAtFront(udpHeader);

    inet::Ptr<inet::L3AddressReq> addresses = udpPacket->addTagIfAbsent<
            inet::L3AddressReq>();
    addresses->setSrcAddress(inet::L3Address(srcAddress));
    addresses->setDestAddress(inet::L3Address(destAddress));

    inet::Ptr<inet::HopLimitReq> hopLimit = udpPacket->addTagIfAbsent<
            inet::HopLimitReq>();
    hopLimit->setHopLimit(255);

    inet::Ptr<inet::PacketProtocolTag> packetProtocol =
            udpPacket->addTagIfAbsent<inet::PacketProtocolTag>();
    packetProtocol->setProtocol(&inet::Protocol::manet);

    inet::Ptr<inet::DispatchProtocolReq> dispatchProtocol =
            udpPacket->addTagIfAbsent<inet::DispatchProtocolReq>();
    dispatchProtocol->setProtocol(&inet::Protocol::ipv6);

    sendUdpPacket(udpPacket, delayed);
}

/*
 * Directorio de vehículos vecinos.
 */

/*!
 * @brief Imprimir el directorio de vehículos vecinos.
 */
void RoutingProtocolBase::showNeighbouringCars() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::showNeighbouringCars");

    NeighbouringCarsConstIt neighbouringCarsIt =
            neighbouringCars.getMap().begin();
    NeighbouringCarsConstIt neighbouringCarsEndIt =
            neighbouringCars.getMap().end();
    while (neighbouringCarsIt != neighbouringCarsEndIt) {
        EV_INFO << "Address: " << neighbouringCarsIt->first << std::endl;
        EV_INFO << "Edge: "
                << neighbouringCarsIt->second.value.locationOnRoadNetwork.edge
                << std::endl;
        EV_INFO << "Distance to vertex A: "
                << neighbouringCarsIt->second.value.locationOnRoadNetwork.distanceToVertexA
                << std::endl;
        EV_INFO << "Distance to vertex B: "
                << neighbouringCarsIt->second.value.locationOnRoadNetwork.distanceToVertexB
                << std::endl;
    }
}

/*!
 * @brief Programar el temporizador de limpieza del directorio de
 * vehículos vecinos.
 */
void RoutingProtocolBase::schedulePurgeNeighbouringCarsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::schedulePurgeNeighbouringCarsTimer");

    omnetpp::simtime_t nextExpiryTime = neighbouringCars.getNextExpiryTime();

    EV_INFO << "Next expiry time: " << nextExpiryTime << std::endl;

    if (nextExpiryTime == omnetpp::SimTime::getMaxTime()) {
        if (purgeNeighbouringCarsTimer->isScheduled())
            cancelEvent(purgeNeighbouringCarsTimer);

    } else {
        if (!purgeNeighbouringCarsTimer->isScheduled())
            scheduleAt(nextExpiryTime, purgeNeighbouringCarsTimer);    // TODO Arreglar.

        else if (purgeNeighbouringCarsTimer->getArrivalTime()
                != nextExpiryTime) {
            cancelEvent(purgeNeighbouringCarsTimer);
            scheduleAt(nextExpiryTime, purgeNeighbouringCarsTimer);
        }
    }
}

/*!
 * @brief Procesar el temporizador de limpieza del directorio de
 * vehículos vecinos.
 */
void RoutingProtocolBase::processPurgeNeighbouringCarsTimer() {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolBase::processPurgeNeighbouringCarsTimer");

    neighbouringCars.removeOldValues(omnetpp::simTime());
    schedulePurgeNeighbouringCarsTimer();
}

/*!
 * @brief Obtener el vehículo vecino más cercano.
 *
 * Encuentra el vehículo vecino cuya ubicación es la más cercana
 * a la ubicación indicada.
 *
 * @param geohashLocation [in] Ubicación Geohash de referencia.
 *
 * @return Dirección IPv6 del vehículo vecino más cercano.
 */
inet::Ipv6Address RoutingProtocolBase::findClosestNeighbouringCar(
        const GeohashLocation &geohashLocation) const {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolBase::getfindClosestNeighbouringCar");

    /*
     * Se recorre el directorio de vehículos vecinos
     * y se busca el más cercano.
     */
    double minDistance = std::numeric_limits<double>::infinity();
    inet::Ipv6Address address = inet::Ipv6Address::UNSPECIFIED_ADDRESS;
    NeighbouringCarsConstIt it = neighbouringCars.getMap().begin();
    NeighbouringCarsConstIt endIt = neighbouringCars.getMap().end();
    while (it != endIt) {
        double distance = geohashLocation.getDistance(
                it->second.value.geohashLocation);

        if (minDistance > distance) {
            minDistance = distance;
            address = it->first;
        }

        it++;
    }

    return address;
}

/*
 * Rutas.
 */

/*!
 * @brief Mostrar rutas en la tabla de enrutamiento.
 */
void RoutingProtocolBase::showRoutes() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::showRoutes");

    for (int i = 0; i < routingTable->getNumRoutes(); i++)
        EV_INFO << "Route: " << routingTable->getRoute(i) << std::endl;
}

/*!
 * @brief Eliminar rutas expiradas de la tabla de enrutamiento.
 *
 * @param expiryTime [in] Hora de expiración máxima para eliminar las rutas.
 */
void RoutingProtocolBase::removeExpiredRoutes(omnetpp::simtime_t expiryTime) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::removeExpiredRoutes");

    inet::Ipv6Route *route;

    for (int i = routingTable->getNumRoutes() - 1; i >= 0; i--) {
        route = routingTable->getRoute(i);

        if (route != nullptr && route->getSourceType() == inet::IRoute::MANET) {
            RouteData *routeData = check_and_cast<RouteData*>(
                    route->getProtocolData());
            if (routeData->getExpiryTime() <= expiryTime)
                routingTable->deleteRoute(route);
        }
    }
    routingTable->purgeDestCache();
}

/*
 * Opciones TLV.
 */

/*!
 * @brief Crear opción TLV de ubicación del destino.
 *
 * @param geohashLocationBits [in] Ubicación Geohash del destino.
 *
 * @return Opción TLV de ubicación del destino.
 */
TlvDestGeohashLocationOption* RoutingProtocolBase::createTlvDestGeohashLocationOption(
        uint64_t geohashLocationBits) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::createTlvDestGeohashLocationOption");

    TlvDestGeohashLocationOption *tlvOption =
            new TlvDestGeohashLocationOption();
    tlvOption->setGeohash(geohashLocationBits);
    tlvOption->setLength(computeTlvOptionLength(tlvOption));
    tlvOption->setType(IPV6TLVOPTION_TLV_DEST_GEOHASH_LOCATION);
    return tlvOption;
}

/*!
 * @brief Calcular la longitud en octetos de una opción TLV de ubicación
 * del destino.
 *
 * @param tlvOption [in] Opción TLV cuya longitud se calcula.
 *
 * @return Longitud de la opción TLV.
 */
int RoutingProtocolBase::computeTlvOptionLength(
        TlvDestGeohashLocationOption *tlvOption) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::computeTlvOptionLength");

    int geohashBytes = 8;
    return geohashBytes;
}

/*!
 * @brief Crear opción TLV de ubicación vial del destino.
 *
 * @param vertexA [in] Primer vértice de la arista de la ubicación vial.
 * @param vertexB [in] Segundo vértice de la arista de la ubicación vial.
 * @param distanceToVertexA [in] Distancia al primer vértice.
 *
 * @return Opción TLV de ubicación vial del destino.
 */
TlvDestLocationOnRoadNetworkOption* RoutingProtocolBase::createTlvDestLocationOnRoadNetworkOption(
        const GeohashLocation &destGeohashLocation) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::createTlvDestLocationOnRoadNetworkOption");

    const RoadNetwork *roadNetwork = roadNetworkDatabase->getRoadNetwork(
            destGeohashLocation);
    LocationOnRoadNetwork locationOnRoadNetwork;
    roadNetwork->getLocationOnRoadNetwork(destGeohashLocation.getLocation(), 0,
            0, locationOnRoadNetwork);
    const Graph &graph = roadNetwork->getGraph();
    Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);
    double &distanceToVertexA = locationOnRoadNetwork.distanceToVertexA;

    TlvDestLocationOnRoadNetworkOption *tlvOption =
            new TlvDestLocationOnRoadNetworkOption();
    tlvOption->setVertexA(vertexA);
    tlvOption->setVertexB(vertexB);
    tlvOption->setDistanceToVertexA(distanceToVertexA);
    tlvOption->setLength(computeTlvOptionLength(tlvOption));
    tlvOption->setType(IPV6TLVOPTION_TLV_DEST_ON_ROAD_NETWORK_LOCATION);
    return tlvOption;
}

/*!
 * @brief Calcular la longitud en octetos de una opción TLV de
 * ubicación vial del destino.
 *
 * @param tlvOption [in] Opción TLV cuya longitud se calcula.
 * @return Longitud de la opción TLV.
 */
int RoutingProtocolBase::computeTlvOptionLength(
        TlvDestLocationOnRoadNetworkOption *tlvOption) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::computeTlvOptionLength");

    int vertexABytes = 2;
    int vertexBBytes = 2;
    int distanceToVertexABytes = 2;
    return vertexABytes + vertexBBytes + distanceToVertexABytes;
}

/*!
 * @brief Crear opción TLV de vértices visitados vacía.
 *
 * @return Opción TLV de vértices visitados.
 */
TlvVisitedVerticesOption* RoutingProtocolBase::createTlvVisitedVerticesOption(
        const VertexSet &visitedVertices) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::createTlvVisitedVerticesOption");

    TlvVisitedVerticesOption *tlvOption = new TlvVisitedVerticesOption();
    tlvOption->setVisitedVerticesArraySize(visitedVertices.size());
    VertexSetConstIt it = visitedVertices.begin();
    VertexSetConstIt endIt = visitedVertices.end();
    tlvOption->setVisitedVerticesArraySize(visitedVertices.size());
    size_t i = 0;
    while (it != endIt) {
        tlvOption->setVisitedVertices(i, *it);
        it++;
        i++;
    }
    tlvOption->setLength(computeTlvOptionLength(tlvOption));
    return tlvOption;
}

/*!
 * @brief Calcular la longitud en octetos de una opción TLV de
 * vértices visitados.
 *
 * @param tlvOption [in] Opción TLV cuya longitud se calcula.
 * @return Longitud de la opción TLV.
 */
int RoutingProtocolBase::computeTlvOptionLength(
        TlvVisitedVerticesOption *tlvOption) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::computeTlvOptionLength");

    int visitedVerticesBytes = 2 * tlvOption->getVisitedVerticesArraySize();
    return visitedVerticesBytes;
}

/*
 * Lifecycle
 */

void RoutingProtocolBase::handleStopOperation(
        inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::handleStopOperation");

    cancelAndDelete(purgeNeighbouringCarsTimer);
    neighbouringCars.getMap().clear();
}

void RoutingProtocolBase::handleCrashOperation(
        inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::handleCrashOperation");

    cancelAndDelete(purgeNeighbouringCarsTimer);
    neighbouringCars.getMap().clear();
}
