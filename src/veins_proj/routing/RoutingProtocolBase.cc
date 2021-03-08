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

/*
 * Interfaz del módulo.
 */

//! Inicialización.
void RoutingProtocolBase::initialize(int stage) {
    inet::RoutingProtocolBase::initialize(stage);

    /*
     * Inicialización local.
     */
    if (stage == inet::INITSTAGE_LOCAL) {
        /*
         * Parámetros de enrutamiento.
         */
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
        addressCache = omnetpp::check_and_cast<AddressCache*>(
                host->getSubmodule("addressCache"));
        if (!addressCache)
            throw omnetpp::cRuntimeError("No host configurator module found");
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
         * Inicialización de interfaces de red.
         */
    } else if (stage == inet::INITSTAGE_NETWORK_INTERFACE_CONFIGURATION) {
        networkInterface = interfaceTable->findInterfaceByName(
                par("outputInterface"));
        if (!networkInterface)
            throw omnetpp::cRuntimeError("Output interface not found");

        /*
         * Inicialización de protocolos de enrutamiento.
         */
    } else if (stage == inet::INITSTAGE_ROUTING_PROTOCOLS) {
        inet::registerProtocol(inet::Protocol::manet, gate("ipOut"),
                gate("ipIn"));
        host->subscribe(inet::linkBrokenSignal, this);
        networkProtocol->registerHook(0, this);
    }
}

//! Manejo de mensajes.
void RoutingProtocolBase::handleMessageWhenUp(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::handleMessageWhenUp");
    EV_INFO << message->getName() << std::endl;
    //EV_INFO << boost::stacktrace::stacktrace() << std::endl;

    if (message->isSelfMessage())
        processSelfMessage(message);

    else
        processMessage(message);
}

/*
 * Manejo de mensajes
 */

//! Manejo de mensajes.
/*!
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

//! Manejo de mensajes propios.
/*!
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

//! Enviar paquete UDP.
/*!
 * Envía un paquete UDP hacia la compuerta que se conecta con el
 * protocolo IP.
 *
 * @param packet [in] Paque a enviar.
 */
void RoutingProtocolBase::sendUdpPacket(inet::Packet *packet) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::sendUdpPacket");
    send(packet, "ipOut");
}

//! Procesar paquete UDP.
/*!
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

        case PacketType::ACK: {
            inet::Ptr<Ack> ack = inet::dynamicPtrCast<Ack>(
                    routingPacket->dupShared());
            processAck(ack);

            break;
        }

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

        case PacketType::PING: {
            inet::Ptr<Ping> ping = inet::dynamicPtrCast<Ping>(
                    routingPacket->dupShared());
            processPing(ping);

            break;
        }

        default:
            throw omnetpp::cRuntimeError(
                    "Routing packet arrived with undefined packet type: %d",
                    packetType);
    }
}

/*
 * Mensajes ACK.
 */

//! Crear mensaje ACK.
/*!
 * @param address [in] Dirección del remitente del mensaje.
 * @return Mensaje ACK.
 */
const inet::Ptr<Ack> RoutingProtocolBase::createAck(
        const inet::Ipv6Address &address) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::createAck");

    const inet::Ptr<Ack> &ack = inet::makeShared<Ack>();

    EV_INFO << "Address: " << address.str() << std::endl;

    ack->setAddress(address);

    return ack;
}

//! Enviar mensaje ACK.
/*!
 * Encapsula un mensaje ACK en un datagrama UDP y lo envía a la dirección
 * indicada.
 *
 * @param ack [in] Mensaje a enviar.
 * @param destAddress [in] Dirección de destino del mensaje.
 */
void RoutingProtocolBase::sendAck(const inet::Ptr<Ack> &ack,
        const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::sendAck");
    EV_INFO << "Destination address: " << destAddress.str() << std::endl;

    inet::Packet *udpPacket = new inet::Packet("Ack");
    udpPacket->insertAtBack(ack);

    inet::Ptr<inet::UdpHeader> udpHeader = inet::makeShared<inet::UdpHeader>();
    udpHeader->setSourcePort(ROUTING_PROTOCOL_UDP_PORT);
    udpHeader->setDestinationPort(ROUTING_PROTOCOL_UDP_PORT);
    udpPacket->insertAtFront(udpHeader);

    inet::Ptr<inet::L3AddressReq> addresses = udpPacket->addTagIfAbsent<
            inet::L3AddressReq>();
    addresses->setSrcAddress(ack->getAddress());
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

//! Procesar mensaje ACK.
/*!
 * @param ack [in] Mensaje a procesar.
 */
void RoutingProtocolBase::processAck(const inet::Ptr<Ack> &ack) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::processAck");
    EV_INFO << "Address: " << ack->getAddress().str() << std::endl;
}

/*
 * Directorio de vehículos vecinos.
 */

//! Imprime el directorio de vehículos vecinos.
void RoutingProtocolBase::showNeighbouringCars() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::showNeighbouringCars");

    NeighbouringCarsConstIterator neighbouringCarsIt = neighbouringCars.getMap().begin();
    NeighbouringCarsConstIterator neighbouringCarsEndIt = neighbouringCars.getMap().end();
    while (neighbouringCarsIt != neighbouringCarsEndIt) {
        EV_INFO << "Address: " << neighbouringCarsIt->first << std::endl;
        EV_INFO << "Edge: "
                << neighbouringCarsIt->second.value.locationOnRoadNetwork.edge
                << std::endl;
        EV_INFO << "Distance to vertex A: "
                << neighbouringCarsIt->second.value.locationOnRoadNetwork.distanceToVertex1
                << std::endl;
        EV_INFO << "Distance to vertex B: "
                << neighbouringCarsIt->second.value.locationOnRoadNetwork.distanceToVertex2
                << std::endl;
    }
}

//! Programar el temporizador de limpieza del directorio de
//! vehículos vecinos.
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
            scheduleAt(nextExpiryTime, purgeNeighbouringCarsTimer);

        else if (purgeNeighbouringCarsTimer->getArrivalTime()
                != nextExpiryTime) {
            cancelEvent(purgeNeighbouringCarsTimer);
            scheduleAt(nextExpiryTime, purgeNeighbouringCarsTimer);
        }
    }
}

//! Procesar el temporizador de limpieza del directorio de
//! vehículos vecinos.
void RoutingProtocolBase::processPurgeNeighbouringCarsTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::processPurgeNeighbouringCarsTimer");

    neighbouringCars.removeOldValues(omnetpp::simTime());
    schedulePurgeNeighbouringCarsTimer();
}

//! Obtener el vehículo vecino más cercano.
/*!
 * Encuentra el vehículo vecino cuya ubicación es la más cercana
 * a la ubicación indicada.
 *
 * @param geohashLocation [in] Ubicación Geohash de la que se quiere
 * conocer el vehículo vecino más cercano.
 * @return Dirección IPv6 del vehículo vecino más cercano.
 */
inet::Ipv6Address RoutingProtocolBase::getClosestNeighbouringCarAddress(
        const GeohashLocation &geohashLocation) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::getClosestNeighbouringCarAddress");

    double minDistance = std::numeric_limits<double>::infinity();
    inet::Ipv6Address closestNeighbourAddress = inet::Ipv6Address::UNSPECIFIED_ADDRESS;

    NeighbouringCarsConstIterator neighbouringCarsIt = neighbouringCars.getMap().begin();
    NeighbouringCarsConstIterator neighbouringCarsEndIt = neighbouringCars.getMap().end();
    while (neighbouringCarsIt != neighbouringCarsEndIt) {
        double distance = geohashLocation.getDistance(
                neighbouringCarsIt->second.value.geohashLocation);

        if (distance < minDistance) {
            minDistance = distance;
            closestNeighbourAddress = neighbouringCarsIt->first;
        }

        neighbouringCarsIt++;
    }

    return closestNeighbourAddress;
}

/*
 * Rutas.
 */

//! Mostrar rutas en la tabla de enrutamiento.
void RoutingProtocolBase::showRoutes() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::showRoutes");

    for (int i = 0; i < routingTable->getNumRoutes(); i++)
        EV_INFO << "Route: " << routingTable->getRoute(i) << std::endl;
}

//! Agregar una ruta a la tabla de enrutamiento.
/*!
 * Antes de agregar la ruta, se verifica si esta ya existe en la tabla
 * de enrutamiento, en cuyo caso no se agrega.
 *
 * @param destPrefix [in] Prefijo de la dirección IPv6 de destino.
 * @param prefixLength [in] Longitud del prefijo.
 * @param nextHop [in] Dirección IPv6 del siguiente salto.
 * @param metric [in] Métrica de la ruta.
 * @param expiryTime [in] Hora de expiración de la ruta.
 */
void RoutingProtocolBase::addRoute(const inet::Ipv6Address &destPrefix,
        const short prefixLength, const inet::Ipv6Address &nextHop, int metric,
        omnetpp::simtime_t expiryTime) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::addRoute");

    // Se revisa si ya existe una ruta
    bool routeExists = false;
    inet::Ipv6Route *route;
    for (int i = 0; i < routingTable->getNumRoutes(); i++) {
        route = routingTable->getRoute(i);
        if (route->getDestPrefix().matches(destPrefix, prefixLength)
                && route->getNextHop() == nextHop) {
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
        route = new inet::Ipv6Route(destPrefix, prefixLength,
                inet::IRoute::MANET);
        route->setNextHop(nextHop);
        route->setMetric(metric);
        route->setSource(this);
        route->setInterface(networkInterface);
        route->setExpiryTime(expiryTime);
        routingTable->addRoute(route);

        EV_INFO << "Route: " << route->str() << std::endl;
    }
}

//! Eliminar rutas por dirección IPv6 de siguiente salto.
/*!
 * Elimina las rutas de la tabla de enrutamiento cuya dirección IPv6
 * de siguiente salto sea igual a la dirección indicada.
 *
 * @param nextHopAddress [in] Dirección IPv6 de siguiente salto de las
 * rutas a eliminar.
 */
void RoutingProtocolBase::purgeNextHopRoutes(
        const inet::Ipv6Address &nextHopAddress) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::purgeNextHopRoutes");

    inet::Ipv6Route *route;

    for (int i = routingTable->getNumRoutes() - 1; i >= 0; i--) {
        route = routingTable->getRoute(i);

        if (route != nullptr && route->getSourceType() == inet::IRoute::MANET)
            if (nextHopAddress == route->getNextHop())
                routingTable->deleteRoute(route);
    }
}

//! Eliminar rutas viejas.
/*!
 * Elimina las rutas de la tabla de enrutamiento coya hora de expiración
 * sea anterior a la hora indicada.
 *
 * @param time [in] Hora de expiración máxima para eliminar las rutas.
 */
void RoutingProtocolBase::removeOldRoutes(omnetpp::simtime_t time) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::purgeNextHopRoutes");

    inet::Ipv6Route *route;

    for (int i = routingTable->getNumRoutes() - 1; i >= 0; i--) {
        route = routingTable->getRoute(i);

        if (route != nullptr && route->getSourceType() == inet::IRoute::MANET)
            if (route->getExpiryTime() <= time)
                routingTable->deleteRoute(route);
    }
}

/*
 * Opciones TLV.
 */

//! Agregar opción TLV a un datagrama.
/*!
 * @param datagram [in] Datagrama al que se le agregará la opción TLV.
 * @param tlvOption [in] Opción TLV a agregar al datagrama.
 */
void RoutingProtocolBase::setTlvOption(inet::Packet *datagram,
        inet::TlvOptionBase *tlvOption) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::setTlvOption");

    datagram->trimFront();

    inet::Ptr<inet::Ipv6Header> ipv6Header = inet::removeNetworkProtocolHeader<
            inet::Ipv6Header>(datagram);
    inet::B oldHeaderLength = ipv6Header->calculateHeaderByteLength();
    inet::Ipv6ExtensionHeader *extensionHeader = ipv6Header->findExtensionHeaderByTypeForUpdate(
            inet::IpProtocolId::IP_PROT_IPv6EXT_HOP);
    inet::Ipv6HopByHopOptionsHeader *optionsHeader = omnetpp::check_and_cast_nullable<
            inet::Ipv6HopByHopOptionsHeader*>(extensionHeader);

    if (!optionsHeader) {
        optionsHeader = new inet::Ipv6HopByHopOptionsHeader();
        optionsHeader->setByteLength(inet::B(8));
        ipv6Header->addExtensionHeader(optionsHeader);
    }

    optionsHeader->getTlvOptionsForUpdate().insertTlvOption(tlvOption);
    optionsHeader->setByteLength(
            inet::B(
                    inet::utils::roundUp(
                            2
                                    + inet::B(
                                            optionsHeader->getTlvOptions().getLength()).get(),
                            8)));
    inet::B newHeaderLength = ipv6Header->calculateHeaderByteLength();
    ipv6Header->addChunkLength(newHeaderLength - oldHeaderLength);
    inet::insertNetworkProtocolHeader(datagram, inet::Protocol::ipv6,
            ipv6Header);
}

//! Crear opción TLV de ubicación del destino.
/*!
 * @param geohashLocationBits [in] Ubicación Geohash del destino.
 * @return Opción TLV.
 */
TlvDestGeohashLocationOption* RoutingProtocolBase::createTlvDestGeohashLocationOption(
        uint64_t geohashLocationBits) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::createTlvDestGeohashLocationOption");

    TlvDestGeohashLocationOption *tlvOption = new TlvDestGeohashLocationOption();
    tlvOption->setGeohash(geohashLocationBits);
    tlvOption->setLength(computeTlvOptionLength(tlvOption));
    tlvOption->setType(IPV6TLVOPTION_TLV_DEST_GEOHASH_LOCATION);
    return tlvOption;
}

//! Calcular la longitud en octetos de una opción TLV de ubicación
//! del destino.
/*!
 * @param tlvOption [in] Opción TLV cuya longitud se calcula.
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

//! Crear opción TLV de ubicación vial del destino.
/*!
 * @param vertexA [in] Primer vértice de la arista de la ubicación vial.
 * @param vertexB [in] Segundo vértice de la arista de la ubicación vial.
 * @param distanceToVertexA [in] Distancia al primer vértice.
 * @return Opción TLV.
 */
TlvDestLocationOnRoadNetworkOption* RoutingProtocolBase::createTlvDestLocationOnRoadNetworkOption(
        Vertex vertexA, Vertex vertexB, double distanceToVertexA) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::createTlvDestLocationOnRoadNetworkOption");

    TlvDestLocationOnRoadNetworkOption *tlvOption = new TlvDestLocationOnRoadNetworkOption();
    tlvOption->setVertexA(vertexA);
    tlvOption->setVertexB(vertexB);
    tlvOption->setDistanceToVertexA(distanceToVertexA);
    tlvOption->setLength(computeTlvOptionLength(tlvOption));
    tlvOption->setType(IPV6TLVOPTION_TLV_DEST_ON_ROAD_NETWORK_LOCATION);
    return tlvOption;
}

//! Agregar opción TLV de ubicación vial del destino a un datagrama.
/*!
 * @param datagram [inout] Datagrama al que se le agregará la opción TLV.
 * destGeohashLocation tlvOption [in] Ubicación Geohash del destino.
 */
void RoutingProtocolBase::setTlvDestLocationOnRoadNetworkOption(
        inet::Packet *datagram,
        const GeohashLocation &destGeohashLocation) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::setTlvDestLocationOnRoadNetworkOption");

    RoadNetwork *roadNetwork = roadNetworkDatabase->getRoadNetwork(
            destGeohashLocation);
    LocationOnRoadNetwork locationOnRoadNetwork;
    roadNetwork->getLocationOnRoadNetwork(destGeohashLocation.getLocation(), 0,
            0, locationOnRoadNetwork);
    const Graph &graph = roadNetwork->getGraph();
    Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);
    double &distanceToVertexA = locationOnRoadNetwork.distanceToVertex1;

    TlvDestLocationOnRoadNetworkOption *destLocationOnRoadNetworkOption = createTlvDestLocationOnRoadNetworkOption(
            vertexA, vertexB, distanceToVertexA);
    setTlvOption(datagram, destLocationOnRoadNetworkOption);
}

//! Calcular la longitud en octetos de una opción TLV de ubicación vial
//! del destino.
/*!
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

//! Crear opción TLV de vértices visitados vacía.
TlvVisitedVerticesOption* RoutingProtocolBase::createTlvVisitedVerticesOption() const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::createTlvVisitedVerticesOption");

    TlvVisitedVerticesOption *tlvOption = new TlvVisitedVerticesOption();
    tlvOption->setVisitedVerticesArraySize(0);
    return tlvOption;
}

//! Calcular la longitud en octetos de una opción TLV de vértices visitados.
/*!
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

void RoutingProtocolBase::handleStartOperation(
        inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolBase::handleStartOperation");
}

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
