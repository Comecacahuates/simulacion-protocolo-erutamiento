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
 * @file RoutingProtocolStaticHost.cc
 * @author Adrián Juárez Monroy
 */

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

    /*
     * Inicialización local.
     */
    if (stage == inet::INITSTAGE_LOCAL) {
        /*
         * Contexto.
         */
        mobility = omnetpp::check_and_cast<StaticHostMobility*>(
                host->getSubmodule("mobility"));
        if (!mobility)
            throw omnetpp::cRuntimeError("No mobility module found");
        configurator = omnetpp::check_and_cast<StaticHostConfigurator*>(
                getModuleByPath(par("configuratorModule")));
        if (!configurator)
            throw omnetpp::cRuntimeError("No configurator module found");
        hostsLocationTable = omnetpp::check_and_cast<HostsLocationTable*>(
                getModuleByPath(par("hostsLocationTableModule")));
        if (!hostsLocationTable)
            throw omnetpp::cRuntimeError(
                    "No hosts location table module found");

        /*
         * Mensajes propios.
         */
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
    Enter_Method
    ("RoutingProtocolStaticHost::processSelfMessage");

    if (message == helloHostTimer)
        processHelloHostTimer();

    else
        RoutingProtocolBase::processSelfMessage(message);
}

/*
 * Mensajes HOLA_VEHIC.
 */

/*!
 * @brief Procesar mensaje HOLA_VEHIC.
 *
 * @param helloCar [in] Mensaje a procesar.
 */
void RoutingProtocolStaticHost::processHelloCar(
        const inet::Ptr<HelloCar> &helloCar) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolStaticHost::processHelloCar");

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
    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    if (!roadNetwork->getGeohashRegion().contains(geohashLocation))
        return;
    const Graph &graph = roadNetwork->getGraph();
    Edge edge = boost::edge(vertexA, vertexB, graph).first;
    double distanceToVertexB = graph[edge].length - distanceToVertexA;
    LocationOnRoadNetwork locationOnRoadNetwork = { edge, 0, distanceToVertexA,
            distanceToVertexB };

    /*
     * Se guarda el registro en el directorio de vehículos vecinos.
     */
    neighbouringCars.getMap()[srcAddress].expiryTime = omnetpp::simTime()
            + neighbouringCarValidityTime;
    neighbouringCars.getMap()[srcAddress].value = { geohashLocation, speed,
            direction, locationOnRoadNetwork };

    // @formatter:off
    EV_INFO << "Address: " << srcAddress.str() << std::endl
            << "Geohash location: " << geohashLocation.getGeohashString() << std::endl
            << "Speed: " << speed << std::endl << "Direction: " << direction << std::endl
            << "Vertex A: " << vertexA << std::endl
            << "Vertex B: " << vertexB << std::endl
            << "Edge: " << edge << std::endl
            << "Distance to vertex A: " << distanceToVertexA << std::endl
            << "Distance to vertex B: " << distanceToVertexB << std::endl;

    EV_DEBUG << "Number of car neighbours: " << neighbouringCars.getMap().size() << std::endl;
            // @formatter:on

    showRoutes();
    schedulePurgeNeighbouringCarsTimer();    // TODO Revisar si es necesario.
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
    Enter_Method
    ("RoutingProtocolBase::scheduleHelloHostTimer");

    scheduleAt(omnetpp::simTime() + helloHostInterval + uniform(0, 1),
            helloHostTimer);
}

/*!
 * @brief Procesar el temporizador de transmisión de mensajes HOLA_HOST.
 */
void RoutingProtocolStaticHost::processHelloHostTimer() {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    EV_DEBUG << "RoutingProtocolStaticHost::processHelloHostTimer" << std::endl;

    const inet::Ipv6Address &primaryUnicastAddress =
            configurator->getUnicastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
    const inet::Ipv6Address &primaryMulticastAddress =
            configurator->getMulticastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
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
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    EV_DEBUG << "RoutingProtocolStaticHost::createHelloHost" << std::endl;

    const GeohashLocation &geohashLocation = mobility->getGeohashLocation();
    const inet::Ptr<HelloHost> &helloHost = inet::makeShared<HelloHost>();
    helloHost->setAddress(hostAddress);
    helloHost->setGeohash(geohashLocation.getBits());

    EV_INFO << "Creating HELLO_HOST"
            << std::endl
            << "Address: "
            << hostAddress.str()
            << std::endl
            << "Geohash location: "
            << geohashLocation.getGeohashString()
            << std::endl;

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
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    EV_DEBUG << "RoutingProtocolStaticHost::routeDatagram" << std::endl;

    /*
     * Se eliminan las rutas expiradas de la tabla de enrutamiento
     * y se verifica si en la tabal de enrutamiento existe una ruta hacia esta,
     * en cuyo caso, se acepta el datagrama.
     */
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();
    removeOldRoutes(omnetpp::simTime());
    if (routingTable->doLongestPrefixMatch(destAddress))
        return inet::INetfilter::IHook::ACCEPT;

    /*
     * Si no existe una ruta, se busca el vehículo vecino más cercano
     * y se usa como siguiente salto para crear una ruta.
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
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolStaticHost::datagramPreRoutingHook");
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
    EV_DEBUG << "Source address: " << srcAddress.str() << std::endl
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
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolStaticHost::datagramLocalOutHook");
    EV_INFO << "Datagram: " << datagram->str() << std::endl;

    /*
     * Se obtiene la dirección de destino del datagrama para saber si
     * el paquete se procesa o se desecha.
     */
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address srcAddress = networkHeader->getSourceAddress().toIpv6();
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();

    // @formatter:off
    EV_DEBUG << "Source address: " << srcAddress.str() << std::endl
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
     * Si el datagrama se tiene que enrutar,
     * se agrega la opción TLV de ubicación del destino.
     */
    GeohashLocation destGeohashLocation = hostsLocationTable->getHostLocation(
            destAddress).geohashLocation;
    TlvDestGeohashLocationOption *destGeohashLocationOption =
            createTlvDestGeohashLocationOption(destGeohashLocation.getBits());
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
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolStaticHost::handleStartOperation");

    RoutingProtocolBase::handleStartOperation(operation);
    scheduleHelloHostTimer();
}

void RoutingProtocolStaticHost::handleStopOperation(
        inet::LifecycleOperation *operation) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolStaticHost::handleStopOperation");

    RoutingProtocolBase::handleStopOperation(operation);
    cancelAndDelete(helloHostTimer);
}

void RoutingProtocolStaticHost::handleCrashOperation(
        inet::LifecycleOperation *operation) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolStaticHost::handleCrashOperation");

    RoutingProtocolBase::handleCrashOperation(operation);
    cancelAndDelete(helloHostTimer);
}

/*
 * Notification.
 */

void RoutingProtocolStaticHost::receiveSignal(omnetpp::cComponent *source,
        omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
        cObject *details) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("RoutingProtocolStaticHost::receiveSignal");

}

