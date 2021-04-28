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
 * @file StaticHostRoutingProtocol.cc
 * @author Adrián Juárez Monroy
 */

#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "veins_proj/routing/StaticHostRoutingProtocol.h"
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
#include "veins_proj/routing/RouteData.h"

using namespace veins_proj;

Define_Module(StaticHostRoutingProtocol);

/*
 * Interfaz del módulo.
 */

/*!
 * @brief Inicialización.
 *
 * @param stage [in] Etapa de inicialización.
 */
void StaticHostRoutingProtocol::initialize(int stage) {
    RoutingProtocolBase::initialize(stage);
    /*
     * Etapa de inicialización local.
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
        locationService = omnetpp::check_and_cast<LocationService*>(
                getModuleByPath(par("locationServiceModule")));
        if (!locationService)
            throw omnetpp::cRuntimeError("No location service module found");
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
void StaticHostRoutingProtocol::processSelfMessage(omnetpp::cMessage *message) {
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
 * @param helloVehicle [in] Mensaje a procesar.
 */
void StaticHostRoutingProtocol::processHelloVehicle(
        const inet::Ptr<HelloVehicle> &helloVehicle) {
    /*
     * Se obtienen los datos del mensaje.
     */
    const inet::Ipv6Address &srcAddress = helloVehicle->getSrcAddress();
    GeohashLocation geohashLocation(helloVehicle->getGeohash(), 12);
    double speed = helloVehicle->getSpeed();
    double direction = helloVehicle->getDirection();
    Vertex u = (Vertex) helloVehicle->getU();
    Vertex v = (Vertex) helloVehicle->getV();
    double distanceToU = helloVehicle->getDistanceToU();
    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    if (!roadNetwork->getGeohashRegion().contains(geohashLocation))
        return;
    const Graph &graph = roadNetwork->getGraph();
    Edge uv = boost::edge(u, v, graph).first;
    double distanceToVertexB = graph[uv].length - distanceToU;
    LocationOnRoadNetwork locationOnRoadNetwork = { uv, 0, distanceToU,
            distanceToVertexB };
    /*
     * Se guarda el registro en el directorio de vehículos vecinos.
     */
    neighbouringVehicles.getMap()[srcAddress].expiryTime = omnetpp::simTime()
            + neighbouringVehicleValidityTime;
    neighbouringVehicles.getMap()[srcAddress].value = { geohashLocation, speed,
            direction, locationOnRoadNetwork };

    EV_INFO << "Address: "
            << srcAddress.str()
            << std::endl
            << "Geohash location: "
            << geohashLocation.getGeohash()
            << std::endl
            << "Speed: "
            << speed
            << std::endl
            << "Direction: "
            << direction
            << std::endl
            << "Vertex A: "
            << u
            << std::endl
            << "Vertex B: "
            << v
            << std::endl
            << "Edge: "
            << uv
            << std::endl
            << "Distance to vertex A: "
            << distanceToU
            << std::endl
            << "Distance to vertex B: "
            << distanceToVertexB
            << std::endl;

    EV_DEBUG << "Number of neighbouring vehicles: "
             << neighbouringVehicles.getMap().size()
             << std::endl;

    showRoutes();
    schedulePurgeNeighbouringVehiclesTimer();    // TODO Revisar si es necesario.
}

/*
 * Mensajes HOLA_HOST
 */

/*!
 * @brief Programar el temporizador de transmisión de mensajes HOLA_HOST.
 *
 * @param start [in] Indica si se va a programar el temporizador
 *                   a la hora de inicio.
 */
void StaticHostRoutingProtocol::scheduleHelloHostTimer(bool start) {
    if (start && omnetpp::simTime() < startTime)
        scheduleAt(startTime, helloHostTimer);
    else
        scheduleAt(omnetpp::simTime() + helloHostInterval, helloHostTimer);
}

/*!
 * @brief Procesar el temporizador de transmisión de mensajes HOLA_HOST.
 */
void StaticHostRoutingProtocol::processHelloHostTimer() {
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
const inet::Ptr<HelloHost> StaticHostRoutingProtocol::createHelloHost(
        const inet::Ipv6Address &hostAddress) const {
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
            << geohashLocation.getGeohash()
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
inet::INetfilter::IHook::Result StaticHostRoutingProtocol::routeDatagram(
        inet::Packet *datagram) {
    /*
     * Se eliminan las rutas expiradas de la tabla de enrutamiento
     * y se verifica si el destino es un vehículo vecino,
     * en cuyo caso, se usa como siguiente salto.
     */
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();
    removeExpiredRoutes(omnetpp::simTime());
    inet::Ipv6Address nextHopAddress;
    if (neighbouringVehicles.getMap().count(destAddress))
        nextHopAddress = destAddress;
    /*
     * Si el destino es un *host*,
     * se agrega la opción de ubicación del destino
     * a la cabecera de opciones de salto por salto.
     */
    else {
        const GeohashLocation &destGeohashLocation =
                locationService->getHostLocation(destAddress);
        TlvDestGeohashLocationOption *destGeohashLocationOption =
                createTlvDestGeohashLocationOption(
                        destGeohashLocation.getBits());
        setTlvOption<TlvDestGeohashLocationOption>(datagram,
                destGeohashLocationOption);
        /*
         * Se verifica si existe una ruta hacia el destino,
         * en cuyo caso, se acepta el datagrama.
         */
        if (routingTable->doLongestPrefixMatch(destAddress))
            return inet::INetfilter::IHook::ACCEPT;
        /*
         * Si no exite la ruta hacia el destino,
         * se usa el vehículo vecino más cercano como siguiente salto.
         */
        nextHopAddress = findClosestNeighbouringVehicle(
                mobility->getGeohashLocation());
    }
    /*
     * Si no se encuentra el siguiente salto, se descarta el datagrama.
     */
    if (nextHopAddress.isUnspecified()) {
        if (hasGUI())
            inet::getContainingNode(host)->bubble(
                    "No next hop found, dropping packet");
        return inet::INetfilter::IHook::DROP;
    }
    /*
     * Si sí se encuentra el siguiente salto, se crea una ruta hacia el destino.
     */
    inet::Ipv6Route *newRoute = new inet::Ipv6Route(destAddress, 128,
            inet::IRoute::SourceType::MANET);
    newRoute->setNextHop(nextHopAddress);
    newRoute->setInterface(networkInterface);
    newRoute->setMetric(1);
    RouteData *routeData = new RouteData(
            omnetpp::simTime() + routeValidityTime);
    newRoute->setProtocolData(routeData);
    routingTable->addRoute(newRoute);
    return inet::INetfilter::IHook::ACCEPT;
}

/*
 * Netfilter.
 */

/*!
 * @brief Procesar datagrama recibido de la capa inferior
 *        antes de enrutarlo.
 *
 * @param datagram [in] Datagrama a procesar.
 *
 * @return Resultado del procesamiento.
 */
inet::INetfilter::IHook::Result StaticHostRoutingProtocol::datagramPreRoutingHook(
        inet::Packet *datagram) {
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

    EV_DEBUG << "Source address: "
             << srcAddress.str()
             << std::endl
             << "Destination address: "
             << destAddress.str()
             << std::endl;
    /*
     * Si la dirección de destino es una dirección local o si es *multicast*,
     * se acepta el datagrama..
     */
    if (interfaceTable->isLocalAddress(inet::L3Address(destAddress))
            || destAddress.isMulticast())
        return inet::INetfilter::IHook::ACCEPT;
    /*
     * Si el datagrama no está dirigido al *host*, se descarta.
     */
    return inet::INetfilter::IHook::DROP;
}

/*!
 * @brief Procesar datagrama recibido de la capa superior
 *        antes de enrutarlo.
 *
 * Se agrega la opción TLV de ubicación del destino y se enruta el paquete.
 *
 * @param datagram [in] Datagrama a procesar.
 *
 * @return Resultado del procesamiento.
 */
inet::INetfilter::IHook::Result StaticHostRoutingProtocol::datagramLocalOutHook(
        inet::Packet *datagram) {
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

    EV_DEBUG << "Source address: "
             << srcAddress.str()
             << std::endl
             << "Destination address: "
             << destAddress.str()
             << std::endl;
    /*
     * Si la dirección de destino es una dirección local o si es *multicast*,
     * se acepta el datagrama..
     */
    if (interfaceTable->isLocalAddress(inet::L3Address(destAddress))
            || destAddress.isMulticast())
        return inet::INetfilter::IHook::ACCEPT;
    /*
     * Se busca una ruta para el datagrama.
     */
    return routeDatagram(datagram);
}

/*
 * Lifecycle.
 */

void StaticHostRoutingProtocol::handleStartOperation(
        inet::LifecycleOperation *operation) {
    scheduleHelloHostTimer(true);
}

void StaticHostRoutingProtocol::handleStopOperation(
        inet::LifecycleOperation *operation) {
    RoutingProtocolBase::handleStopOperation(operation);
    cancelAndDelete(helloHostTimer);
}

void StaticHostRoutingProtocol::handleCrashOperation(
        inet::LifecycleOperation *operation) {
    RoutingProtocolBase::handleCrashOperation(operation);
    cancelAndDelete(helloHostTimer);
}

/*
 * Notification.
 */

void StaticHostRoutingProtocol::receiveSignal(omnetpp::cComponent *source,
        omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
        cObject *details) {
}

