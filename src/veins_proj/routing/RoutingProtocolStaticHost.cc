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

    EV_DEBUG << "Address: " << srcAddress.str() << std::endl;
    EV_DEBUG << "Geohash location: " << geohashLocation.getGeohashString()
             << std::endl;
    EV_DEBUG << "Speed: " << speed << std::endl;
    EV_DEBUG << "Direction: " << direction << std::endl;
    EV_DEBUG << "Vertex A: " << vertexA << std::endl;
    EV_DEBUG << "Vertex B: " << vertexB << std::endl;
    EV_DEBUG << "Edge: " << edge << std::endl;
    EV_DEBUG << "Distance to vertex A: " << distanceToVertexA << std::endl;
    EV_DEBUG << "Distance to vertex B: " << distanceToVertexB << std::endl;

    neighbouringCars.getMap()[srcAddress].expiryTime = omnetpp::simTime()
            + neighbouringCarValidityTime;
    neighbouringCars.getMap()[srcAddress].value = { geohashLocation, speed,
            direction, locationOnRoadNetwork };

    int distance = (int) geohashLocation.getDistance(
            mobility->getGeohashLocation());

    EV_DEBUG << "Number of car neighbours: " << neighbouringCars.getMap().size()
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

    const inet::Ptr<HelloHost> helloHost = createHelloHost(
            primaryUnicastAddress);
    sendRoutingMessage(helloHost, "HOLA_HOST", primaryUnicastAddress,
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

/*
 * Enrutamiento.
 */

/*!
 * @brief Enrutar datagrama.
 *
 * Revisa si existe en la tabla de enrutamiento una ruta hacia la
 * dirección de destino.
 * Si no existe, se busca el vehículo vecino más cercano
 * y usa como siguiente salto para la ruta hacia el destino.
 * Si no se encuentra ningún vehículo vecino,
 * se descarta el datagrama.
 *
 * @param datagram [in] Datagrama a enrutar.
 *
 * @return Resultado del enrutamiento.
 */
inet::INetfilter::IHook::Result RoutingProtocolStaticHost::routeDatagram(
        inet::Packet *datagram) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "RoutingProtocolStaticHost::routeDatagram" << std::endl;

    /*
     * Se eliminan las rutas expiradas de la tabla de enrutamiento
     * y se verifica si en la tabal de enrutamiento existe una ruta hacia esta,
     * en cuyo caso, se acepta el datagrama.
     */
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader = inet::getNetworkProtocolHeader(
            datagram);
    inet::Ipv6Address destAddress = networkHeader->getDestinationAddress().toIpv6();
    removeOldRoutes(omnetpp::simTime());
    if (routingTable->doLongestPrefixMatch(destAddress))
        return inet::INetfilter::IHook::ACCEPT;

    /*
     * Si no existe una ruta, se busca el vehículo vecino más cercano
     * y se usa como siguiente salto para crear la ruta hacia el destino.
     */
    /*
     * Si no existe una ruta, se busca el vehículo vecino más cercano
     * y se usa como siguiente salto para crear una ruta..
     * Si este no se encuentra, se descarta el datagrama.
     */
    inet::Ipv6Address nextHopAddress = findClosestNeighbouringCar(
            mobility->getGeohashLocation());
    if (nextHopAddress.isUnspecified()) {
        if (hasGUI())
            inet::getContainingNode(host)->bubble(
                    "No next hop found, dropping packet");
        return inet::INetfilter::IHook::DROP;
    }

    /*
     * Si sí se encuentra el siguiente salto, se crea una ruta hacia el destino.
     */
    inet::Ipv6Route *route = new inet::Ipv6Route(destAddress, 128,
            inet::IRoute::SourceType::MANET);
    route->setNextHop(nextHopAddress);
    route->setInterface(networkInterface);
    route->setMetric(1);
    route->setExpiryTime(omnetpp::simTime() + routeValidityTime);
    routingTable->addRoute(route);

    return inet::INetfilter::IHook::ACCEPT;
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
inet::INetfilter::IHook::Result RoutingProtocolStaticHost::datagramPreRoutingHook(
        inet::Packet *datagram) {
    EV_DEBUG
            << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolStaticHost::datagramPreRoutingHook");
    EV_DEBUG << "Datagram: " << datagram->str() << std::endl;

    /*
     * Se obtiene la dirección de destino del datagrama para saber hacia
     * dónde enrutarlo.
     */
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader = inet::getNetworkProtocolHeader(
            datagram);
    inet::Ipv6Address srcAddress = networkHeader->getSourceAddress().toIpv6();
    inet::Ipv6Address destAddress = networkHeader->getDestinationAddress().toIpv6();

    EV_DEBUG << "Source address: " << srcAddress.str() << std::endl;
    EV_DEBUG << "Destination address: " << destAddress.str() << std::endl;

    /*
     * Si la dirección de destino es una dirección local o si es _multicast_,
     * se acepta el datagrama..
     */
    if (interfaceTable->isLocalAddress(inet::L3Address(destAddress))
            || destAddress.isMulticast())
        return inet::INetfilter::IHook::ACCEPT;

    /*
     * Si el datagrama no está dirigido al _host_, se descarta.
     */
    return inet::INetfilter::IHook::DROP;
}

/*!
 * @brief Procesar datagrama recibido de la capa superior
 * antes de enrutarlo.
 *
 * Se agrega la opción TLV de ubicación del destino y se enruta el paquete.
 *
 * @param datagram [in] Datagrama a procesar.
 *
 * @return Resultado del procesamiento.
 */
inet::INetfilter::IHook::Result RoutingProtocolStaticHost::datagramLocalOutHook(
        inet::Packet *datagram) {
    EV_DEBUG
            << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("RoutingProtocolStaticHost::datagramLocalOutHook");
    EV_DEBUG << "Datagram: " << datagram->str() << std::endl;

    /*
     * Se obtiene la dirección de destino del datagrama para saber si
     * el paquete se procesa o se desecha.
     */
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader = inet::getNetworkProtocolHeader(
            datagram);
    inet::Ipv6Address srcAddress = networkHeader->getSourceAddress().toIpv6();
    inet::Ipv6Address destAddress = networkHeader->getDestinationAddress().toIpv6();

    EV_DEBUG << "Source address: " << srcAddress.str() << std::endl;
    EV_DEBUG << "Destination address: " << destAddress.str() << std::endl;

    /*
     * Si la dirección de destino es una dirección local o si es _multicast_,
     * se acepta el datagrama..
     */
    if (interfaceTable->isLocalAddress(inet::L3Address(destAddress))
            || destAddress.isMulticast())
        return inet::INetfilter::IHook::ACCEPT;

    /*
     * Si el datagrama se tiene que enrutar,
     * se agrega la opción TLV de ubicación del destino.
     */
    GeohashLocation destGeohashLocation = hostsLocationTable->getHostLocation(
            destAddress).geohashLocation;
    TlvDestGeohashLocationOption *destGeohashLocationOption = createTlvDestGeohashLocationOption(
            destGeohashLocation.getBits());
    setTlvOption(datagram, destGeohashLocationOption);

    /*
     * Se busca una ruta para el datagrama.
     */
    return routeDatagram(datagram);
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

