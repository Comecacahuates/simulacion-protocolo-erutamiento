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

#include "veins_proj/routing/RoutingProtocolStaticHost.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/packet/chunk/Chunk.h"
#include "inet/transportlayer/udp/UdpHeader_m.h"
#include "inet/networklayer/common/HopLimitTag_m.h"
#include "inet/networklayer/common/L3AddressTag_m.h"
#include "inet/networklayer/common/L3Tools.h"
#include "inet/networklayer/common/NextHopAddressTag_m.h"
#include "inet/linklayer/common/InterfaceTag_m.h"
#include "inet/networklayer/ipv6/Ipv6InterfaceData.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/roadnetwork/RoadNetwork.h"

using namespace veins_proj;

Define_Module(RoutingProtocolStaticHost);

/*
 * Interfaz del módulo.
 */

/*!
 * @brief Inicialización.
 *
 * @param stage [in] Etapa de inicialización.
 */
void RoutingProtocolStaticHost::initialize(int stage) {
    RoutingProtocolBase::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        // Context
        mobility = omnetpp::check_and_cast<StaticHostMobility*>(
                host->getSubmodule("mobility"));

        if (!mobility)
            throw omnetpp::cRuntimeError("No mobility module found");

        hostsLocationTable = omnetpp::check_and_cast<HostsLocationTable*>(
                getModuleByPath(par("hostsLocationTableModule")));

        if (!hostsLocationTable)
            throw omnetpp::cRuntimeError(
                    "No hosts location table module found");

        // Self messages
        helloHostTimer = new omnetpp::cMessage("helloHostTimer");
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
void RoutingProtocolStaticHost::processSelfMessage(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::processSelfMessage" << std::endl;

    if (message == helloHostTimer)
        processHelloHostTimer();

    else
        RoutingProtocolBase::processSelfMessage(message);
}

/*
 * Mensajes HOLA_VEHIC.
 */

/*!
 * @brief Procesar mensaje HOLA_VEIC.
 *
 * @param helloCar [in] Mensaje a procesar.
 */
void RoutingProtocolStaticHost::processHelloCar(
        const inet::Ptr<HelloCar> &helloCar) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::processHelloCar" << std::endl;

    // Se obtienen los datos del mensaje
    const inet::Ipv6Address &srcAddress = helloCar->getSrcAddress();
    GeohashLocation geohashLocation(helloCar->getGeohash(), 12);
    double speed = helloCar->getSpeed();
    double direction = helloCar->getDirection();
    Vertex vertexA = (Vertex) helloCar->getVertexA();
    Vertex vertexB = (Vertex) helloCar->getVertexB();
    double distanceToVertexA = helloCar->getDistanceToVertexA();

    RoadNetwork *roadNetwork = roadNetworkDatabase->getRoadNetwork(
            mobility->getGeohashLocation());
    const Graph &graph = roadNetwork->getGraph();

    if (roadNetwork->getGeohashRegion().contains(geohashLocation)) {
    }

    Edge edge = boost::edge(vertexA, vertexB, graph).first;
    double distanceToVertexB = graph[edge].length - distanceToVertexA;
    LocationOnRoadNetwork locationOnRoadNetwork = { edge, 0, distanceToVertexA,
            distanceToVertexB };

    EV_INFO << "Address: " << srcAddress.str() << std::endl;
    EV_INFO << "Geohash location: " << geohashLocation.getGeohashString()
            << std::endl;
    EV_INFO << "Speed: " << speed << std::endl;
    EV_INFO << "Direction: " << direction << std::endl;
    EV_INFO << "Vertex A: " << vertexA << std::endl;
    EV_INFO << "Vertex B: " << vertexB << std::endl;
    EV_INFO << "Edge: " << edge << std::endl;
    EV_INFO << "Distance to vertex A: " << distanceToVertexA << std::endl;
    EV_INFO << "Distance to vertex B: " << distanceToVertexB << std::endl;

    neighbouringCars.getMap()[srcAddress].expiryTime = omnetpp::simTime();
    neighbouringCars.getMap()[srcAddress].value = { geohashLocation, speed,
            direction, locationOnRoadNetwork };

    int distance = (int) geohashLocation.getDistance(
            mobility->getGeohashLocation());

    addRoute(srcAddress, 64, srcAddress, distance,
            omnetpp::simTime() + neighbouringCarValidityTime);

    EV_INFO << "Number of car neighbours: " << neighbouringCars.getMap().size()
            << std::endl;

    showRoutes();
    schedulePurgeNeighbouringCarsTimer();
}

/*
 * Mensajes HOLA_HOST
 */

/*!
 * @brief Programar el temporizador de transmisión de mensajes HOLA_HOST.
 */
void RoutingProtocolStaticHost::scheduleHelloHostTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolBase::scheduleHelloHostTimer" << std::endl;

    scheduleAt(omnetpp::simTime() + helloHostInterval + uniform(0, 1),
            helloHostTimer);
}

/*!
 * @brief Procesar el temporizador de transmisión de mensajes HOLA_HOST.
 */
void RoutingProtocolStaticHost::processHelloHostTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::processHelloHostTimer" << std::endl;

    const inet::Ipv6Address &primaryUnicastAddress = addressCache->getUnicastAddress(
            PRIMARY_ADDRESS);
    const inet::Ipv6Address &primaryMulticastAddress = addressCache->getMulticastAddress(
            PRIMARY_ADDRESS);

    sendHelloHost(createHelloHost(primaryUnicastAddress),
            primaryMulticastAddress);
    scheduleHelloHostTimer();
}

/*!
 * @brief Crear mensaje HOLA_HOST.
 *
 * @param hostAddress [in] Dirección del _host_ que transmite el mnesaje.
 *
 * @return Mensaje HOLA_HOST.
 */
const inet::Ptr<HelloHost> RoutingProtocolStaticHost::createHelloHost(
        const inet::Ipv6Address &hostAddress) const {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::createHelloHost" << std::endl;

    const inet::Ptr<HelloHost> &helloHost = inet::makeShared<HelloHost>();

    const GeohashLocation &geohashLocation = mobility->getGeohashLocation();

    EV_INFO << "Address: " << hostAddress.str() << std::endl;
    EV_INFO << "Geohash location: " << geohashLocation.getGeohashString()
            << std::endl;
    EV_INFO << "                : " << geohashLocation.getBits() << std::endl;

    helloHost->setAddress(hostAddress);
    helloHost->setGeohash(geohashLocation.getBits());

    return helloHost;
}

/*!
 * @brief Enviar mensaje HOLA_HOST.
 *
 * Encapsula un mensaje HOLA_HOST en un datagrama UDP y lo envía
 * a la dirección indicada.
 *
 * @param helloCar [in] Mensaje a enviar.
 * @param destAddress [in] Dirección de destino del mensaje.
 */
void RoutingProtocolStaticHost::sendHelloHost(
        const inet::Ptr<HelloHost> &helloHost,
        const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::sendHelloHost" << std::endl;

    inet::Packet *udpPacket = new inet::Packet("ANC_HOST");
    udpPacket->insertAtBack(helloHost);

    inet::Ptr<inet::UdpHeader> udpHeader = inet::makeShared<inet::UdpHeader>();
    udpHeader->setSourcePort(ROUTING_PROTOCOL_UDP_PORT);
    udpHeader->setDestinationPort(ROUTING_PROTOCOL_UDP_PORT);
    udpPacket->insertAtFront(udpHeader);

    inet::Ptr<inet::L3AddressReq> addresses = udpPacket->addTagIfAbsent<
            inet::L3AddressReq>();
    addresses->setSrcAddress(helloHost->getAddress());
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

/*
 * Rutas.
 */

/*!
 * @brief Agregar ruta hacia un destino a la tabla de enrutamiento.
 *
 * Se busca el vehículo vecino más cercano y se selecciona como
 * siguiente salto para la ruta. Si se encuentra el siguiente salto,
 * se crea la ruta y se agrega a la tabla de enrutamiento
 * si esta no existe todavía, o se actualiza si ya existía.
 *
 * @param destAddress [in] Dirección de destino.
 *
 * @return `true` si se creo la ruta o si ya existía.
 */
bool RoutingProtocolStaticHost::addRouteToDest(
        const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::addRouteIfNotExists" << std::endl;

    if (routingTable->doLongestPrefixMatch(destAddress) == nullptr) {
        const GeohashLocation &geohashLocation = mobility->getGeohashLocation();
        inet::Ipv6Address nextHopAddress = getClosestNeighbouringCar(
                geohashLocation);

        if (nextHopAddress.isUnspecified())
            return false;

        const GeohashLocation &neighbouringCarGeohashLocation = neighbouringCars.getMap()[nextHopAddress].value.geohashLocation;
        int distance = (int) geohashLocation.getDistance(
                neighbouringCarGeohashLocation);
        addRoute(destAddress, 64, nextHopAddress, distance,
                neighbouringCars.getMap()[nextHopAddress].expiryTime);
    }

    return true;
}

/*
 * Enrutamiento.
 */

/*!
 * @brief Enrutar datagrama.
 *
 * Revisa si existe en la tabla de enrutamiento una ruta hacia la
 * dirección de destino. Si no existe, se intenta descubrir y crear una
 * ruta. Si no se encuentra la ruta, se descarta el datagrama.
 *
 * @param datagram [in] Datagrama a enrutar.
 * @param destAddress [in] Dirección IPv6 de destino.
 *
 * @return Resultado del enrutamiento.
 */
inet::INetfilter::IHook::Result RoutingProtocolStaticHost::routeDatagram(
        inet::Packet *datagram, const inet::Ipv6Address &destAddress) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::routeDatagram" << std::endl;

    // Se crea una ruta si no existe
    bool routeExists = addRouteToDest(destAddress);

    // Si no existe la ruta y no se pudo crear
    if (!routeExists) {
        if (hasGUI())
            inet::getContainingNode(host)->bubble(
                    "No next hop found, dropping packet");
        return inet::INetfilter::IHook::DROP;
    }

    GeohashLocation destGeohashLocation = hostsLocationTable->getHostLocation(
            destAddress).geohashLocation;
    TlvDestGeohashLocationOption *destGeohashLocationOption = createTlvDestGeohashLocationOption(
            destGeohashLocation.getBits());
    setTlvOption(datagram, destGeohashLocationOption);

    return inet::INetfilter::IHook::ACCEPT;
}

/*
 * Netfilter.
 */

inet::INetfilter::IHook::Result RoutingProtocolStaticHost::datagramPreRoutingHook(
        inet::Packet *datagram) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::datagramPreRoutingHook" << std::endl;
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

inet::INetfilter::IHook::Result RoutingProtocolStaticHost::datagramLocalOutHook(
        inet::Packet *datagram) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::datagramLocalOutHook" << std::endl;
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

    return routeDatagram(datagram, destAddress);
}

/*
 * Lifecycle.
 */

void RoutingProtocolStaticHost::handleStartOperation(
        inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::handleStartOperation" << std::endl;

    RoutingProtocolBase::handleStartOperation(operation);
    scheduleHelloHostTimer();
}

void RoutingProtocolStaticHost::handleStopOperation(
        inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::handleStopOperation" << std::endl;

    RoutingProtocolBase::handleStopOperation(operation);
    cancelAndDelete(helloHostTimer);
}

void RoutingProtocolStaticHost::handleCrashOperation(
        inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::handleCrashOperation" << std::endl;

    RoutingProtocolBase::handleCrashOperation(operation);
    cancelAndDelete(helloHostTimer);
}

/*
 * Notification.
 */

void RoutingProtocolStaticHost::receiveSignal(omnetpp::cComponent *source,
        omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
        cObject *details) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::receiveSignal" << std::endl;

}

