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
 * @file VehicleRoutingProtocol.cc
 * @author Adrián Juárez Monroy
 */

#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/routing/VehicleRoutingProtocol.h"
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
#include "veins_proj/routing/RouteData.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "veins_proj/mobility/VehicleMobility.h"
#include "veins_proj/mobility/StaticHostMobility.h"
#include <cmath>
#include <vector>
#include <limits>
#include <iterator>
#include <algorithm>
#include <boost/stacktrace.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/tuple/tuple.hpp>

using namespace veins_proj;

Define_Module(VehicleRoutingProtocol);

/*
 * Interfaz del módulo.
 */

/*!
 * @brief Inicialización.
 *
 * @param [in] Etapa de inicialización.
 */
void VehicleRoutingProtocol::initialize(int stage) {
    RoutingProtocolBase::initialize(stage);
    /*
     * Etapa de Inicialización local.
     */
    if (stage == inet::INITSTAGE_LOCAL) {
        /*
         * Contexto.
         */
        mobility = omnetpp::check_and_cast<VehicleMobility*>(
                host->getSubmodule("mobility"));
        if (!mobility)
            throw omnetpp::cRuntimeError("No mobility module found");
        configurator = omnetpp::check_and_cast<VehicleConfigurator*>(
                getModuleByPath(par("configuratorModule")));
        if (!configurator)
            throw omnetpp::cRuntimeError("No configurator module found");
        /*
         * Mensajes propios.
         */
        helloVehicleTimer = new omnetpp::cMessage("helloVehicleTimer");
        purgeNeighbouringHostsTimer = new omnetpp::cMessage(
                "purgeNeighbouringHostsTimer");
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
void VehicleRoutingProtocol::processSelfMessage(omnetpp::cMessage *message) {
    if (message == helloVehicleTimer)
        processHelloVehicleTimer();
    else if (message == purgeNeighbouringHostsTimer)
        processPurgeNeighbouringHostsTimer();
    else
        RoutingProtocolBase::processSelfMessage(message);
}

/*
 * Mensajes HOLA-VEHIC.
 */

/*!
 * @brief Programar el temporizador de transmisión de mensajes HOLA-VEHIC.
 *
 * @param start [in] Indica si se va a programar el temporizador
 *                   a la hora de inicio.
 */
void VehicleRoutingProtocol::scheduleHelloVehicleTimer(bool start) {
    if (start && omnetpp::simTime() < startTime)
        scheduleAt(startTime, helloVehicleTimer);
    else
        scheduleAt(omnetpp::simTime() + helloVehicleInterval,
                helloVehicleTimer);
}

/*!
 * @brief Procesar el temporizador de transmisión de mensajes HOLA_VEIC.
 */
void VehicleRoutingProtocol::processHelloVehicleTimer() {
    /*
     * Se envía el mensaje para la subred primaria.
     */
    const inet::Ipv6Address &primaryUnicastAddress =
            configurator->getUnicastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
    const inet::Ipv6Address &primaryMulticastAddress =
            configurator->getMulticastAddress(
                    ConfiguratorBase::NetworkType::PRIMARY);
    if (!primaryUnicastAddress.isUnspecified()) {
        const inet::Ptr<HelloVehicle> helloVehicle = createHelloVehicle(
                primaryUnicastAddress);
        sendRoutingMessage(helloVehicle, "HOLA-VEHIC", primaryUnicastAddress,
                primaryMulticastAddress);
    }
    /*
     * Se envía el mensaje para la subred secundaria.
     */
    const inet::Ipv6Address &secondaryUnicastAddress =
            configurator->getUnicastAddress(
                    ConfiguratorBase::NetworkType::SECONDARY);
    const inet::Ipv6Address &secondaryMulticastAddress =
            configurator->getMulticastAddress(
                    ConfiguratorBase::NetworkType::SECONDARY);
    if (!secondaryUnicastAddress.isUnspecified()) {
        const inet::Ptr<HelloVehicle> helloVehicle = createHelloVehicle(
                secondaryUnicastAddress);
        sendRoutingMessage(helloVehicle, "HOLA-VEHIC", secondaryUnicastAddress,
                secondaryMulticastAddress, true);
    }
    scheduleHelloVehicleTimer();
}

/*!
 * @brief Crear mensaje HOLA-VEHIC.
 *
 * @param srcAddress [in] Dirección del vehículo que trnasmite el mensaje.
 * @return Mensaje HOLA-VEHIC.
 */
const inet::Ptr<HelloVehicle> VehicleRoutingProtocol::createHelloVehicle(
        const inet::Ipv6Address &srcAddress) const {
    /*
     * Se obtienen los datos para el mensaje.
     */
    const Graph &graph = roadNetworkDatabase->getRoadNetwork(
            mobility->getGeohashLocation())->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork =
            mobility->getLocationOnRoadNetwork();
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex u = boost::source(edge, graph);
    Vertex v = boost::target(edge, graph);
    const double &distanceToU = locationOnRoadNetwork.distanceToU;
    const GeohashLocation &geohashLocation = mobility->getGeohashLocation();
    double speed = mobility->getSpeed();
    double direction = mobility->getDirection();
    /*
     * Se crea el mensaje y se le agregan los datos.
     */
    const inet::Ptr<HelloVehicle> &helloVehicle =
            inet::makeShared<HelloVehicle>();
    helloVehicle->setSrcAddress(srcAddress);
    helloVehicle->setGeohash(geohashLocation.getBits());
    helloVehicle->setSpeed(speed);
    helloVehicle->setDirection(direction);
    helloVehicle->setU(u);
    helloVehicle->setV(v);
    helloVehicle->setDistanceToU(distanceToU);
    return helloVehicle;
}

/*!
 * @brief Procesar mensaje HOLA-VEHIC.
 *
 * @param HelloVehicle [in] Mensaje a procesar.
 */
void VehicleRoutingProtocol::processHelloVehicle(
        const inet::Ptr<HelloVehicle> &HelloVehicle) {
    /*
     * Se obtienen los datos del mensaje.
     */
    const inet::Ipv6Address &srcAddress = HelloVehicle->getSrcAddress();
    GeohashLocation geohashLocation(HelloVehicle->getGeohash(), 12);
    double speed = HelloVehicle->getSpeed();
    double direction = HelloVehicle->getDirection();
    Vertex u = (Vertex) HelloVehicle->getU();
    Vertex v = (Vertex) HelloVehicle->getV();
    double distanceToU = HelloVehicle->getDistanceToU();
    const Graph &graph =
            roadNetworkDatabase->getRoadNetwork(geohashLocation)->getGraph();
    Edge uv = boost::edge(u, v, graph).first;
    double distanceToV = graph[uv].length - distanceToU;
    LocationOnRoadNetwork locationOnRoadNetwork = { uv, 0, distanceToU,
            distanceToV };
    /*
     * Se guarda el registro en el directorio de vehículos vecinos.
     */
    neighbouringVehicles.getMap()[srcAddress].expiryTime = omnetpp::simTime()
            + neighbouringVehicleValidityTime;
    neighbouringVehicles.getMap()[srcAddress].value = { geohashLocation, speed,
            direction, locationOnRoadNetwork };
    showRoutes();
    schedulePurgeNeighbouringVehiclesTimer();
}

/*
 * Mensajes HOLA_HOST.
 */

/*!
 * @brief Procesar mensaje HOLA_HOST.
 *
 * @param helloHost [in] Mensaje a procesar.
 */
void VehicleRoutingProtocol::processHelloHost(
        const inet::Ptr<HelloHost> &helloHost) {
    /*
     * Se obtienen los datos del mensaje.
     */
    inet::Ipv6Address srcAddress = helloHost->getAddress();
    GeohashLocation geohashLocation(helloHost->getGeohash(), 12);
    /*
     * Se guarda el registro en el directorio de *hosts* vecinos.
     */
    neighbouringHosts.getMap()[srcAddress] = { omnetpp::simTime()
            + neighbouringHostValidityTime, geohashLocation };
    schedulePurgeNeighbouringHostsTimer();
}

/*
 * Directorio de *hosts* vecinos.
 */

/*!
 * @brief Imrpimir el directorio de *hosts* vecinos.
 */
void VehicleRoutingProtocol::showNeighbouringHosts() const {

    EV_INFO << "Neighbouring hosts:" << std::endl;

    NeighbouringHostsConstIt it = neighbouringHosts.getMap().begin();
    NeighbouringHostsConstIt endIt = neighbouringHosts.getMap().end();
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
 *        de *hosts* vecinos.
 */
void VehicleRoutingProtocol::schedulePurgeNeighbouringHostsTimer() {
    omnetpp::simtime_t nextExpiryTime = neighbouringHosts.getNextExpiryTime();
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
 *        de *hosts* vecinos.
 */
void VehicleRoutingProtocol::processPurgeNeighbouringHostsTimer() {
    neighbouringHosts.removeOldValues(omnetpp::simTime());
    removeExpiredRoutes(omnetpp::simTime());
    schedulePurgeNeighbouringHostsTimer();
}

/*
 * Cabecera de opciones de salto por salto.
 */

/*!
 * @brief Validar la cabecera de opciones de salto por salto
 *        de un datagrama.
 *
 * Verifica si la cabecera tiene la opción de ubicación del destino.
 *
 * Si el destino se encuentra en la misma subred, se verifica si
 * la cabecera contiene la opción de ubicación vial del destino. Si no
 * la tiene, la agrega.
 *
 * Si la cabecera no contiene la opción de vértices visitados, se agrega.
 *
 * @param datagram [in] Datagrama cuya cabecera se valida.
 */
bool VehicleRoutingProtocol::validateHopByHopOptionsHeader(
        inet::Packet *datagram) const {
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
    GeohashLocation destGeohashLocation(
            destGeohashLocationOption->getGeohashBits(), 12);
    const GeohashRegion &geohashRegion =
            mobility->getRoadNetwork()->getGeohashRegion();
    if (geohashRegion.contains(destGeohashLocation)) {
        const TlvDestLocationOnRoadNetworkOption *destLocationOnRoadNetworkOption =
                findTlvOption<TlvDestLocationOnRoadNetworkOption>(datagram);
        if (destLocationOnRoadNetworkOption == nullptr) {
            TlvDestLocationOnRoadNetworkOption *destLocationOnRoadNetworkOption =
                    createTlvDestLocationOnRoadNetworkOption(
                            destGeohashLocation);
            setTlvOption<TlvDestLocationOnRoadNetworkOption>(datagram,
                    destLocationOnRoadNetworkOption);
        }
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
        setTlvOption<TlvVisitedVerticesOption>(datagram, visitedVerticesOption);
    }
    return true;
}

/*!
 * @brief Se obtiene el conjunto de vértices visitados.
 *
 * @param visitedVerticesOption [in] Opción de vértices visitados.
 * @return Conjunto de vértices visitados.
 */
VertexSet VehicleRoutingProtocol::getDatagramVisitedVertices(
        inet::Packet *datagram) const {
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
 * @brief Agrega los siguientes vértices visitados de la ruta a la opción
 *        de vértices visitados del datagrama.
 *
 * @param datagram [in] Datagrama cuya opción de
 *                      vértices visitados se actualiza.
 * @param route    [in] Ruta que tomará el datagrama.
 */
void VehicleRoutingProtocol::updateTlvVisitedVerticesOption(
        inet::Packet *datagram, const inet::Ipv6Route *route) const {
    /*
     * Si la ruta no incluye vértices visitados del siguiente salto,
     * no es necesario actualizar la opción de vértices visitados.
     */
    RouteData *routeData = dynamic_cast<RouteData*>(route->getProtocolData());
    if (!routeData)
        return;
    const VertexSet &nextHopVisitedVertices =
            routeData->getNextHopVisitedVertices();
    /*
     * Se crea el nuevo conjunto de vértices visitados y se agregan
     * los siguientes vértices visitados del siguiente salto de la ruta.
     */
    VertexSet visitedVertices;
    VertexSetConstIt it = nextHopVisitedVertices.begin();
    VertexSetConstIt endIt = nextHopVisitedVertices.end();
    while (it != endIt) {
        visitedVertices.insert(*it);
        it++;
    }
    /*
     * Se agregan los vértices visitados del datagrama
     * al nuevo conjunto de vértices visitados.
     */
    const TlvVisitedVerticesOption *visitedVerticesOption = findTlvOption<
            TlvVisitedVerticesOption>(datagram);
    size_t i = 0;
    size_t n = visitedVerticesOption->getVisitedVerticesArraySize();
    while (i < n)
        visitedVertices.insert(visitedVerticesOption->getVisitedVertices(i++));
    /*
     * Se crea una nueva opción de vértices visitados
     * y se inserta en el datagrama.
     */
    TlvVisitedVerticesOption *newVisitedVerticesOption =
            createTlvVisitedVerticesOption(visitedVertices);
    setTlvOption<TlvVisitedVerticesOption>(datagram, newVisitedVerticesOption);
}

/*
 * Cálculo de rutas.
 */

/*!
 * @brief Obtener el conjunto de aristas activas.
 *
 * Las aristas activas son aquellas en las que hay vehículos vecinos.
 *
 * @return Conjunto de aristas activas.
 */
EdgeSet VehicleRoutingProtocol::getActiveEdges() const {
    /*
     * Se obtienen las aristas activas en el primer tramo recto.
     */
    EdgeSet activeEdges;
    const Graph &graph = roadNetworkDatabase->getRoadNetwork(
            mobility->getGeohashLocation())->getGraph();
    Edge edge = mobility->getLocationOnRoadNetwork().edge;
    ShortestPaths shortestPaths(graph, edge);
    VertexVector straightPath = shortestPaths.getStraightPathFromEdge(edge,
            false);
    EdgeSet activeEdgesInStraightPath = getActiveEdgesInPath(straightPath);
    activeEdges.insert(activeEdgesInStraightPath.begin(),
            activeEdgesInStraightPath.end());
    /*
     * Se obtienen las aristas activas en el segundo tramo recto.
     */
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);
    edge = boost::edge(vertexB, vertexA, graph).first;
    straightPath = shortestPaths.getStraightPathFromEdge(edge, false);
    activeEdgesInStraightPath = getActiveEdgesInPath(straightPath);
    activeEdges.insert(activeEdgesInStraightPath.begin(),
            activeEdgesInStraightPath.end());
    /*
     * Se verifica si el vehículo se encuentra en uno de los vértices.
     */
    Vertex vertex;
    if (mobility->isAtVertex(vertexA))
        vertex = vertexA;
    else if (mobility->isAtVertex(vertexB))
        vertex = vertexB;
    else
        return activeEdges;
    /*
     * Si el vehículo se encuentra en uno de los vértices,
     * se obtienen los tramos rectos de todas las arista que salen de este,
     * y se buscan las aristas activas en estos.
     */
    {
        OutEdgeIt it, endIt;
        boost::tie(it, endIt) = boost::out_edges(vertex, graph);
        while (it != endIt) {
            Edge edge = *it++;
            if (!activeEdges.count(edge)) {
                straightPath = shortestPaths.getStraightPathFromEdge(edge);
                activeEdgesInStraightPath = getActiveEdgesInPath(straightPath);
                activeEdges.insert(activeEdgesInStraightPath.begin(),
                        activeEdgesInStraightPath.end());
            }
        }
    }
    return activeEdges;
}

/*!
 * @brief Obtener el conjunto de aristas activas en una
 *        secuencia de aristas.
 *
 * Las aristas activas son aquellas en las que hay vehículos vecinos.
 *
 * @return Conjunto de aristas activas.
 */
EdgeSet VehicleRoutingProtocol::getActiveEdgesInPath(
        const VertexVector &path) const {
    const Graph &graph = roadNetworkDatabase->getRoadNetwork(
            mobility->getGeohashLocation())->getGraph();
    EdgeSet activeEdges;
    /*
     * Se recorren las aristas del tramo recto de la última a la primera
     * hasta encontrar la primera en la que haya un vehículo vecino.
     */
    NeighbouringVehiclesByEdge neighbouringVehiclesByEdge =
            getNeighbouringVehiclesByEdge();
    int i, j;
    for (i = path.size() - 2, j = i + 1; i >= 0; i--, j--) {
        Vertex u = path[i];
        Vertex v = path[j];
        Edge uv = boost::edge(u, v, graph).first;
        if (neighbouringVehiclesByEdge.count(uv))
            break;
    }
    /*
     * Si no se encontró ningún vehículo vecino en alguna
     * de las aristas se devuelve el conjunto de aristas vacío.
     */
    if (i < 0)
        return activeEdges;
    /*
     * Se agregan las aristas del tramo recto hasta la arista en la
     * que se encontró el vehículo vecino.
     */
    for (i = 0; i < j; i++) {
        Vertex u = path[i];
        Vertex v = path[i + 1];
        Edge uv = boost::edge(u, v, graph).first;
        activeEdges.insert(uv);
    }
    return activeEdges;
}

/*!
 * @brief Obtener el vértice de destino local.
 *
 * @param datagram     [in] Datagrama a enrutar.
 * @param shortestPath [in] Rutas más cortas.
 * @return Vértice de destino local y bandera que indica si sí se encontró.
 */
std::pair<Vertex, bool> VehicleRoutingProtocol::getLocalDestVertex(
        inet::Packet *datagram, const ShortestPaths &shortestPaths) const {
    /*
     * Se obtiene la ubicación del destino para saber si se encuentra
     * en la misma subred o en otra.
     */
    const TlvDestGeohashLocationOption *destGeohashLocationOption =
            findTlvOption<TlvDestGeohashLocationOption>(datagram);
    ASSERT(destGeohashLocationOption != nullptr);
    GeohashLocation destGeohashLocation(
            destGeohashLocationOption->getGeohashBits(), 12);
    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    const GeohashRegion &geohashRegion = roadNetwork->getGeohashRegion();
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
        Vertex u = (Vertex) destLocationOnRoadNetworkOption->getU();
        Vertex v = (Vertex) destLocationOnRoadNetworkOption->getV();
        double routeDistanceU = shortestPaths.getRouteDistance(u);
        double routeDistanceV = shortestPaths.getRouteDistance(v);
        localDestVertex = u;
        localDestVertexFound = shortestPaths.routeToVertexFound(u);
        if (routeDistanceV < routeDistanceU) {
            localDestVertex = v;
            localDestVertexFound = shortestPaths.routeToVertexFound(v);
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
            const VertexSet &northGatewayVertices =
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
            const VertexSet &southGatewayVertices =
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
            const VertexSet &eastGatewayVertices =
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
            const VertexSet &westGatewayVertices =
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
        VertexVectorConstIt it = tentativeGatewayVertices.begin();
        VertexVectorConstIt endIt = tentativeGatewayVertices.end();
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

/*
 * Enrutamiento de datagramas.
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
inet::INetfilter::IHook::Result VehicleRoutingProtocol::routeDatagram(
        inet::Packet *datagram) {
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
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();
    /*
     * Se eliminan las rutas viejas y se obtiene la dirección de destino
     * para verificar si existe una ruta para esta.
     */
    removeExpiredRoutes(omnetpp::simTime());
    const inet::Ipv6Route *route = routingTable->doLongestPrefixMatch(
            destAddress);
    if (route) {
        updateTlvVisitedVerticesOption(datagram, route);
        return inet::INetfilter::IHook::ACCEPT;
    }
    /*
     * Si la dirección de destino es de un *host* vecino,
     * se selecciona este como siguiente salto.
     */
    if (neighbouringHosts.getMap().count(destAddress)) {
        return routeDatagramToNeighbouringHost(datagram);
    }
    /*
     * Se obtiene el conjunto de aristas activas y se calcula la ruta
     * más corta a cada uno de los vértices.
     */
    const Graph &graph = roadNetworkDatabase->getRoadNetwork(
            mobility->getGeohashLocation())->getGraph();
    const Edge &edge = mobility->getLocationOnRoadNetwork().edge;
    ShortestPaths shortestPaths(graph, edge);
    EdgeSet activeEdges = getActiveEdges();
    VertexSet visitedVertices = getDatagramVisitedVertices(datagram);
    shortestPaths.computeShortestPaths(visitedVertices, activeEdges);
    Vertex localDestVertex;
    bool localDestVertexFound;
    boost::tie(localDestVertex, localDestVertexFound) = getLocalDestVertex(
            datagram, shortestPaths);
    if (!localDestVertexFound) {
        if (hasGUI())
            inet::getContainingNode(host)->bubble(
                    "No local destination vertex found, dropping packet");
        return inet::INetfilter::IHook::DROP;
    }
    /*
     * Si la ruta tiene una sola arista (en la que está el vehículo),
     * el destino puede estar en la misma arista que el vehículo
     * o en una arista adyacente.
     */
    VertexVector shortestPath = shortestPaths.getShortestPathToVertex(
            localDestVertex);
    if (shortestPath.size() == 2) {
        ASSERT(localDestVertex == shortestPath[1]);
        /*
         * Se verifica si el destino está en la misma región.
         */
        const TlvDestGeohashLocationOption *destGeohashLocationOption =
                findTlvOption<TlvDestGeohashLocationOption>(datagram);
        ASSERT(destGeohashLocationOption != nullptr);
        GeohashLocation destGeohashLocation(
                destGeohashLocationOption->getGeohashBits(), 12);
        const GeohashRegion geohashRegion(mobility->getLocation(), 6);
        if (geohashRegion.contains(destGeohashLocation)) {
            /*
             * Se el destino está en la misma arista,
             * se enruta el datagrama hacia el vehículo vecino
             * más cercano a este.
             */
            const TlvDestLocationOnRoadNetworkOption *destLocationOnRoadNetworkOption =
                    findTlvOption<TlvDestLocationOnRoadNetworkOption>(datagram);
            ASSERT(destLocationOnRoadNetworkOption != nullptr);
            Vertex u = shortestPath[0];
            Vertex v = shortestPath[1];
            Edge edge = boost::edge(u, v, graph).first;
            u = destLocationOnRoadNetworkOption->getU();
            v = destLocationOnRoadNetworkOption->getV();
            Edge destEdge = boost::edge(u, v, graph).first;
            if (edge == destEdge)
                return routeDatagramToLocation(datagram, destGeohashLocation);
            /*
             * Si el destino está en una arista adyacente,
             * se verifica si el tramo que se forma con esta es recto,
             * en cuyo caso, se enruta el datagrama hacia el vehículo vecino
             * más cercano a este.
             */
            else {
                u = shortestPath[1];
                v = boost::source(destEdge, graph);
                bool edgeExists = boost::edge(u, v, graph).second;
                if (!edgeExists)
                    v = boost::source(destEdge, graph);
                edgeExists = boost::edge(u, v, graph).second;
                ASSERT(edgeExists);
                shortestPath.push_back(v);
                if (shortestPaths.isStraightPath(shortestPath))
                    return routeDatagramToLocation(datagram,
                            destGeohashLocation);
                /*
                 * Si el destino no está en el tramo recto,
                 * se selecciona como siguiente salto el vehículo vecino
                 * más cercano a la esquina.
                 */
                else {
                    return routeDatagramClosestToVertex(datagram, u);
                }
            }
            /*
             * Si el destino está en otra región,
             * y el vértice de destino local es *gateway*,
             * se enruta el datagrama hacia la subred adyacente.
             */
        } else {
            const GeohashLocation::Adjacency &adjacency =
                    graph[localDestVertex].adjacency;
            if (adjacency != GeohashLocation::Adjacency::NONE) {
                return routeDatagramToAdjacentNetwork(datagram, adjacency);
            } else {
                if (hasGUI())
                    inet::getContainingNode(host)->bubble(
                            "No next hop found, dropping packet");
                return inet::INetfilter::IHook::DROP;
            }
        }
    }
    /*
     * Se obtiene el tramo recto más largo posible en la ruta más corta.
     * Si este tiene una sola arista (en la que está el vehículo),
     * hay una vuelta en la esquina.
     * En este caso, si el vehículo no se encuentra en la esquina,
     * se intenta enrutar hacia un vehículo vecino que esté más cerca de esta.
     */
    VertexVector straightPath = shortestPaths.getStraightPath(shortestPath);
    if (straightPath.size() == 2) {
        if (!findNextHopClosestToVertex(straightPath[1]).isUnspecified())
            return routeDatagramClosestToVertex(datagram, straightPath[1]);
        /*
         * Si no se encuentra un vehículo vecino más cercano al vértice,
         * se calcula el siguiente tramo recto en la ruta,
         * y se enruta hacia el vehículo vecino más cercano en este.
         */
        else {
            shortestPath.erase(shortestPath.begin());
            straightPath = shortestPaths.getStraightPath(shortestPath);
            return routeDatagramClosestInPath(datagram, straightPath);
        }
        /*
         * Si el tramo recto tiene más de una arista,
         * se enruta el datagrama hacia el vecino más lejano en este,
         * para que el datagrama haga el mayor avance posible.
         */
    } else
        return routeDatagramFurthestInPath(datagram, straightPath);
    /*
     * Si no se encuentra una ruta para el datagrama, se descarta.
     */
    if (hasGUI())
        inet::getContainingNode(host)->bubble(
                "No next hop found, dropping packet");
    return inet::INetfilter::IHook::Result::DROP;
}

/*!
 * @brief Enrutar datagrama hacia un *host* vecino.
 *
 * @param datagram [in] Datagrama a enrutar.
 * @return Resultado del enrutamiento.
 */
inet::INetfilter::IHook::Result VehicleRoutingProtocol::routeDatagramToNeighbouringHost(
        inet::Packet *datagram) {
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();
    ASSERT(neighbouringHosts.getMap().count(destAddress));
    inet::Ipv6Route *newRoute = new inet::Ipv6Route(destAddress, 128,
            inet::IRoute::SourceType::MANET);
    newRoute->setNextHop(destAddress);
    newRoute->setInterface(networkInterface);
    newRoute->setMetric(1);
    RouteData *routeData = new RouteData(
            omnetpp::simTime() + routeValidityTime);
    newRoute->setProtocolData(routeData);
    routingTable->addRoute(newRoute);
    return inet::INetfilter::IHook::ACCEPT;
}

/*!
 * @brief Enrutar datagrama hacia el vecino más cercano a una ubicación.
 *
 * @param datagram        [in] Datagrama a enrutar.
 * @param geohashLocation [in] Ubicación hacia la que
 *        se enruta el datagrama.
 * @return Resultado del enrutamiento.
 */
inet::INetfilter::IHook::Result VehicleRoutingProtocol::routeDatagramToLocation(
        inet::Packet *datagram, const GeohashLocation &geohashLocation) {
    const inet::Ipv6Address nextHopAddress = findNextHopClosestToLocation(
            geohashLocation);
    if (nextHopAddress.isUnspecified()) {
        if (hasGUI())
            inet::getContainingNode(host)->bubble(
                    "No next hop found, dropping packet");
        return inet::INetfilter::IHook::DROP;
    }
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();
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

/*!
 * @brief Enrutar datagrama hacia una subred vecina.
 *
 * @param datagram  [in] Datagrama a enrutar.
 * @param adjacency [in] Adyacencia de la subred a la que se va
 *                       a enrutar el paquete.
 * @return Resultado del enrutamiento.
 */
inet::INetfilter::IHook::Result VehicleRoutingProtocol::routeDatagramToAdjacentNetwork(
        inet::Packet *datagram, GeohashLocation::Adjacency adjacency) {
    const inet::Ipv6Address &nextHopAddress = findNextHopInAdjacentNetwork(
            adjacency);
    if (nextHopAddress.isUnspecified()) {
        if (hasGUI())
            inet::getContainingNode(host)->bubble(
                    "No next hop found, dropping packet");
        return inet::INetfilter::IHook::DROP;
    }
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();
    inet::Ipv6Route *newRoute = new inet::Ipv6Route(destAddress, 128,
            inet::IRoute::SourceType::MANET);
    newRoute->setNextHop(nextHopAddress);
    newRoute->setInterface(networkInterface);
    newRoute->setMetric(1);
    RouteData *routeData = new RouteData(
            omnetpp::simTime() + routeValidityTime);
    newRoute->setProtocolData(routeData);
    routingTable->addRoute(newRoute);
    removeTlvOption<TlvVisitedVerticesOption>(datagram);
    return inet::INetfilter::IHook::ACCEPT;
}

/*!
 * @brief Enrutar el datagrama al vehículo vecino más lejano
 *        en una secuencia de aristas.
 *
 * @param datagram [in] Datagrama a enrutar.
 * @param path     [in] Secuencia de aristas hacia las
 *                      que se va a enrutar el datagrama.
 * @return Resultado del enrutamiento.
 */
inet::INetfilter::IHook::Result VehicleRoutingProtocol::routeDatagramFurthestInPath(
        inet::Packet *datagram, const VertexVector &path) {
    const inet::Ipv6Address &nextHopAddress = findNextHopFurthestInPath(path);
    if (nextHopAddress.isUnspecified()) {
        if (hasGUI())
            inet::getContainingNode(host)->bubble(
                    "No next hop found, dropping packet");
        return inet::INetfilter::IHook::DROP;
    }
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();
    inet::Ipv6Route *newRoute = new inet::Ipv6Route(destAddress, 128,
            inet::IRoute::SourceType::MANET);
    newRoute->setNextHop(nextHopAddress);
    newRoute->setInterface(networkInterface);
    newRoute->setMetric(1);
    VertexSet nextHopVisitedVertices = getNextHopVisitedVertices(nextHopAddress,
            path);
    RouteData *routeData = new RouteData(omnetpp::simTime() + routeValidityTime,
            nextHopVisitedVertices);
    newRoute->setProtocolData(routeData);
    routingTable->addRoute(newRoute);
    updateTlvVisitedVerticesOption(datagram, newRoute);
    return inet::INetfilter::IHook::ACCEPT;
}

/*!
 * @brief Enrutar el datagrama al vehículo vecino más cercano
 *        en una secuencia de aristas.
 *
 * @param datagram [in] Datagrama a enrutar.
 * @param path     [in] Secuencia de aristas hacia las
 *                      que se va a enrutar el datagrama.
 * @return Resultado del enrutamiento.
 */
inet::INetfilter::IHook::Result VehicleRoutingProtocol::routeDatagramClosestInPath(
        inet::Packet *datagram, const VertexVector &path) {
    const inet::Ipv6Address &nextHopAddress = findNextHopClosestInPath(path);
    if (nextHopAddress.isUnspecified()) {
        if (hasGUI())
            inet::getContainingNode(host)->bubble(
                    "No next hop found, dropping packet");
        return inet::INetfilter::IHook::DROP;
    }
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();
    inet::Ipv6Route *newRoute = new inet::Ipv6Route(destAddress, 128,
            inet::IRoute::SourceType::MANET);
    newRoute->setNextHop(nextHopAddress);
    newRoute->setInterface(networkInterface);
    newRoute->setMetric(1);
    VertexSet nextHopVisitedVertices = getNextHopVisitedVertices(nextHopAddress,
            path);
    RouteData *routeData = new RouteData(omnetpp::simTime() + routeValidityTime,
            nextHopVisitedVertices);
    newRoute->setProtocolData(routeData);
    routingTable->addRoute(newRoute);
    updateTlvVisitedVerticesOption(datagram, newRoute);
    return inet::INetfilter::IHook::ACCEPT;
}

/*!
 * @brief Enrutar datagrama al vehículo vecino en la misma arista
 *        que se encuentre más cerca a un vértice.
 *
 * @param datagram [in] Datagrama a enrutar.
 * @param vertex   [in] Vértice hacia el que se enruta el datagrama.
 * @return Resultado del enrutamiento.
 */
inet::INetfilter::IHook::Result VehicleRoutingProtocol::routeDatagramClosestToVertex(
        inet::Packet *datagram, const Vertex vertex) {
    const inet::Ipv6Address &nextHopAddress = findNextHopClosestToVertex(
            vertex);
    if (nextHopAddress.isUnspecified()) {
        if (hasGUI())
            inet::getContainingNode(host)->bubble(
                    "No next hop found, dropping packet");
        return inet::INetfilter::IHook::DROP;
    }
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();
    inet::Ipv6Route *newRoute = new inet::Ipv6Route(destAddress, 128,
            inet::IRoute::SourceType::MANET);
    newRoute->setNextHop(nextHopAddress);
    newRoute->setInterface(networkInterface);
    newRoute->setMetric(1);
    RouteData *routeData = new RouteData(
            omnetpp::simTime() + routeValidityTime);
    newRoute->setProtocolData(routeData);
    routingTable->addRoute(newRoute);
    updateTlvVisitedVerticesOption(datagram, newRoute);
    return inet::INetfilter::IHook::ACCEPT;
}

/*
 * Selección del siguiente salto.
 */

/*!
 * @brief Obtener vehículo vecino en la región Geohash adyacente.
 *
 * @param neighbouringGeohashRegion [in] Región Geohash adyacente.
 * @return Dirección IPv6 del siguiente salto, o `::/128`
 *         si no se encuentra ninguno.
 */
const inet::Ipv6Address& VehicleRoutingProtocol::findNextHopInAdjacentNetwork(
        const GeohashLocation::Adjacency adjacencyDirection) const {
    GeohashLocation adjacentGeohashRegion =
            mobility->getRoadNetwork()->getGeohashRegion().getAdjacentGeohashRegion(
                    adjacencyDirection);
    NeighbouringVehiclesConstIt it = neighbouringVehicles.getMap().begin();
    NeighbouringVehiclesConstIt endIt = neighbouringVehicles.getMap().end();
    while (it != endIt) {
        const GeohashLocation &neighbouringVehicleGeohashLocation =
                it->second.value.geohashLocation;
        if (adjacentGeohashRegion.contains(neighbouringVehicleGeohashLocation))
            return it->first;
    }
    return inet::Ipv6Address::UNSPECIFIED_ADDRESS;
}

/*!
 * @brief Encontrar siguiente salto más cercano a una ubicación.
 *
 * @return Dirección IPv6 del siguiente salto, o `::/128`.
 *         si no se encuentra ninguno.
 */
const inet::Ipv6Address& VehicleRoutingProtocol::findNextHopClosestToLocation(
        const GeohashLocation &geohashLocation) const {
    /*
     * Se recorre el directorio de vehículos vecinos en busca de uno
     * que esté a una distancia menor de la ubiación.
     */
    double minDistance = std::numeric_limits<double>::infinity();
    double distance;
    NeighbouringVehiclesConstIt it = neighbouringVehicles.getMap().begin();
    NeighbouringVehiclesConstIt endIt = neighbouringVehicles.getMap().end();
    NeighbouringVehiclesConstIt chosenIt = neighbouringVehicles.getMap().end();
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
 * @brief Encontrar siguiente salto más lejano en una secuenaia de aristas.
 *
 * @param path [in] Secuencia de aristas en las que se busca
 *                  el siguiente salto.
 * @return Dirección IPv6 del siguiente salto, o `::/128`
 *         si no se encuentra ninguno.
 */
const inet::Ipv6Address& VehicleRoutingProtocol::findNextHopFurthestInPath(
        const VertexVector &path) const {
    const Graph &graph = roadNetworkDatabase->getRoadNetwork(
            mobility->getGeohashLocation())->getGraph();
    /*
     * De la última arista a la primera, se busca el vecino más cercano
     * al segundo vértice de cada una.
     */
    NeighbouringVehiclesByEdge neighbouringVehiclesByEdge =
            getNeighbouringVehiclesByEdge();
    for (int i = path.size() - 2; i >= 0; i--) {
        Vertex u = path[i];
        Vertex v = path[i + 1];
        Edge uv = boost::edge(u, v, graph).first;
        const inet::Ipv6Address &nextHopAddress = findNextHopClosestToVertex(v,
                uv, neighbouringVehiclesByEdge);
        if (!nextHopAddress.isUnspecified())
            return nextHopAddress;
    }
    return inet::Ipv6Address::UNSPECIFIED_ADDRESS;
}

/*!
 * @brief Encontrar siguiente salto más cercano en una secuencia de aristas.
 *
 * @param path [in] Secuencia de aristas en las que se busca
 *                  el siguiente salto.
 * @return Dirección IPv6 del siguiente salto, o `::/128`
 *         si no se encuentra ninguno.
 */
const inet::Ipv6Address& VehicleRoutingProtocol::findNextHopClosestInPath(
        const VertexVector &path) const {
    /*
     * De la primera arista a la última, se busca el vecino más cercano
     * al primer vértice de cada una.
     */
    const Graph &graph = roadNetworkDatabase->getRoadNetwork(
            mobility->getGeohashLocation())->getGraph();
    NeighbouringVehiclesByEdge neighbouringVehiclesByEdge =
            getNeighbouringVehiclesByEdge();
    for (int i = 0; i < path.size() - 1; i++) {
        Vertex u = path[i];
        Vertex v = path[i + 1];
        Edge edge = boost::edge(u, v, graph).first;
        const inet::Ipv6Address &nextHopAddress = findNextHopClosestToVertex(u,
                edge, neighbouringVehiclesByEdge);
        if (!nextHopAddress.isUnspecified())
            return nextHopAddress;
    }
    return inet::Ipv6Address::UNSPECIFIED_ADDRESS;
}

/*!
 * @brief Buscar vehículo vecino más cercano a un vértice entre los que
 *        se encuentran en una arista.
 *
 * @param vertex [in] Vértice de referencia.
 * @param edge   [in] Arista en la que se encuentran los vehículos
 *                    entre los que se hace la búsqueda.
 * @return Dirección IPv6 del siguiente salto, o `::/128`.
 *         si no se encuentra ninguno.
 */
const inet::Ipv6Address& VehicleRoutingProtocol::findNextHopClosestToVertex(
        const Vertex vertex, const Edge edge,
        const NeighbouringVehiclesByEdge &neighbouringVehiclesByEdge) const {
    /*
     * Se recorren los vehículos vecinos que se encuentran en la arista
     * en busca de uno cuya distancia al vértice sea la menor.
     */
    const Graph &graph = roadNetworkDatabase->getRoadNetwork(
            mobility->getGeohashLocation())->getGraph();
    double minDistance = std::numeric_limits<double>::infinity();
    NeighbouringVehiclesByEdgeConstIt it =
            neighbouringVehiclesByEdge.lower_bound(edge);
    NeighbouringVehiclesByEdgeConstIt endIt =
            neighbouringVehiclesByEdge.upper_bound(edge);
    NeighbouringVehiclesConstIt nextHopIt = neighbouringVehicles.getMap().end();
    while (it != endIt) {
        const inet::Ipv6Address &neighbouringVehicleAddress = it->second;
        ASSERT(neighbouringVehicles.getMap().count(neighbouringVehicleAddress));
        const LocationOnRoadNetwork &locationOnRoadNetwork =
                neighbouringVehicles.getMap().find(neighbouringVehicleAddress)->second.value.locationOnRoadNetwork;
        double distance = veins_proj::getDistanceToVertex(locationOnRoadNetwork,
                vertex, graph);
        if (minDistance > distance) {
            minDistance = distance;
            nextHopIt = neighbouringVehicles.getMap().find(
                    neighbouringVehicleAddress);
        }
        it++;
    }
    if (nextHopIt == neighbouringVehicles.getMap().end())
        return inet::Ipv6Address::UNSPECIFIED_ADDRESS;
    return nextHopIt->first;
}

/*!
 * @brief Buscar vehículo vecino más cercano a un vértice que
 *        se encuentra en la misma arista.
 *
 * Se buscan los vehículos vecinos que circulan sobre la misma arista,
 * y se obtiene el que se encuentra a la menor distancia del vértice
 * indicado.
 *
 * @param vertex [in] Vértice de referencia.
 * @return Dirección IPv6 del siguiente salto, o `::/128`.
 *         si no se encuentra ninguno.
 *
 * TODO: Verificar si hace falta.
 */
const inet::Ipv6Address& VehicleRoutingProtocol::findNextHopClosestToVertex(
        Vertex vertex) const {
    /*
     * Se obtiene la distancia del vehículo al vértice
     * para comparar con la distancia de cada vehículo vecino a este.
     */
    const Graph &graph = roadNetworkDatabase->getRoadNetwork(
            mobility->getGeohashLocation())->getGraph();
    const LocationOnRoadNetwork &locationOnRoadNetwork =
            mobility->getLocationOnRoadNetwork();
    double minDistance = getDistanceToVertex(locationOnRoadNetwork, vertex,
            graph);
    /*
     * Se recorre el directorio de vehículos vecinos en busca de uno
     * que esté a una distancia menor del vértice.
     */
    double distance;
    NeighbouringVehiclesConstIt it = neighbouringVehicles.getMap().begin();
    NeighbouringVehiclesConstIt endIt = neighbouringVehicles.getMap().end();
    NeighbouringVehiclesConstIt nextHopIt = neighbouringVehicles.getMap().end();
    while (it != endIt) {
        distance = getDistanceToVertex(it->second.value.locationOnRoadNetwork,
                vertex, graph);
        if (minDistance > distance) {
            minDistance = distance;
            nextHopIt = it;
        }
        it++;
    }
    if (nextHopIt == endIt)
        return inet::Ipv6Address::UNSPECIFIED_ADDRESS;
    return nextHopIt->first;
}

/*
 * Métodos auxiliares.
 */

/*!
 * @brief Agrupar vehículos vecinos según la arista en la que se encuentran.
 *
 * @return Diccinario de vecinos agrupados según la aristan en la
 *         que se encuentran.
 */
VehicleRoutingProtocol::NeighbouringVehiclesByEdge VehicleRoutingProtocol::getNeighbouringVehiclesByEdge() const {
    NeighbouringVehiclesByEdge neighbouringVehiclesByEdge;
    NeighbouringVehiclesConstIt it = neighbouringVehicles.getMap().begin();
    NeighbouringVehiclesConstIt endIt = neighbouringVehicles.getMap().end();
    while (it != endIt) {
        neighbouringVehiclesByEdge.insert(
                std::pair<Edge, inet::Ipv6Address>(
                        it->second.value.locationOnRoadNetwork.edge,
                        it->first));
        it++;
    }
    return neighbouringVehiclesByEdge;
}

/*!
 * @brief Obtener los vértices visitados del siguiente salto.
 *
 * @param nextHopAddress [in] Siguiente salto al que se asocian
 *                            los vértices visitados.
 * @param path           [in] Ruta vial.
 * @return Vértices visitados del siguiente salto.
 */
VertexSet VehicleRoutingProtocol::getNextHopVisitedVertices(
        const inet::Ipv6Address &nextHopAddress,
        const VertexVector &path) const {
    /*
     * Se recorren los vértices de la ruta vial hasta encontrar
     * la arista por la que circula el vehículo del siguiente salto.
     */
    const Graph &graph = roadNetworkDatabase->getRoadNetwork(
            mobility->getGeohashLocation())->getGraph();
    ASSERT(neighbouringVehicles.getMap().count(nextHopAddress));
    const Edge &neighbouringVehicleEdge = neighbouringVehicles.getMap().find(
            nextHopAddress)->second.value.locationOnRoadNetwork.edge;
    Vertex u = boost::source(neighbouringVehicleEdge, graph);
    Vertex v = boost::target(neighbouringVehicleEdge, graph);
    VertexSet nextHopVisitedVertices;
    for (int i = 0; i < path.size(); i++) {
        Vertex w = path[i];
        nextHopVisitedVertices.insert(w);
        if (w == u || w == v)
            break;
    }
    return nextHopVisitedVertices;
}

/*
 * Netfilter.
 */

/*!
 * @brief Procesar datagrama recibido de la capa inferior
 *        antes de enrutarlo.
 *
 * @param datagram [in] Datagrama a procesar.
 * @return Resultado del procesamiento.
 */
inet::INetfilter::IHook::Result VehicleRoutingProtocol::datagramPreRoutingHook(
        inet::Packet *datagram) {
    /*
     * Se obtiene la dirección de destino del datagrama para saber hacia
     * dónde enrutarlo.
     */
    const inet::Ptr<const inet::NetworkHeaderBase> &networkHeader =
            inet::getNetworkProtocolHeader(datagram);
    inet::Ipv6Address destAddress =
            networkHeader->getDestinationAddress().toIpv6();
    /*
     * Si la dirección de destino es una dirección local o si es *multicast*,
     * se acepta el datagrama..
     */
    if (interfaceTable->isLocalAddress(inet::L3Address(destAddress))
            || destAddress.isMulticast())
        return inet::INetfilter::IHook::ACCEPT;
    /*
     * Se enruta el datagrama.
     */
    return routeDatagram(datagram);
}

/*!
 * @brief Procesar datagrama recibido de la capa superior
 *        antes de enrutarlo.
 *
 * @param datagram [in] Datagrama a procesar.
 * @return Resultado del procesamiento.
 */
inet::INetfilter::IHook::Result VehicleRoutingProtocol::datagramLocalOutHook(
        inet::Packet *datagram) {
    return inet::INetfilter::IHook::ACCEPT;
}

/*
 * Lifecycle.
 */

void VehicleRoutingProtocol::handleStartOperation(
        inet::LifecycleOperation *operation) {
    scheduleHelloVehicleTimer(true);
}

void VehicleRoutingProtocol::handleStopOperation(
        inet::LifecycleOperation *operation) {
    RoutingProtocolBase::handleStopOperation(operation);
    cancelAndDelete(helloVehicleTimer);
    cancelAndDelete(purgeNeighbouringHostsTimer);
    neighbouringHosts.getMap().clear();
}

void VehicleRoutingProtocol::handleCrashOperation(
        inet::LifecycleOperation *operation) {
    RoutingProtocolBase::handleCrashOperation(operation);
    cancelAndDelete(helloVehicleTimer);
    cancelAndDelete(purgeNeighbouringHostsTimer);
    neighbouringHosts.getMap().clear();
}

/*
 * Notification.
 */

void VehicleRoutingProtocol::receiveSignal(omnetpp::cComponent *source,
        omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
        cObject *details) {

}
