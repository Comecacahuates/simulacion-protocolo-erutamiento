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
 * @file CarMobility.h
 * @author Adrián Juárez Monroy
 */

#include "veins_proj/mobility/CarMobility.h"
#include "veins/base/utils/Coord.h"
#include "inet/common/INETMath.h"
#include "inet/common/geometry/common/Coord.h"
#include <boost/math/constants/constants.hpp>
#include <boost/tuple/tuple.hpp>
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include <utility>
#include <iomanip>
#include <cmath>
#include <iomanip>

using namespace veins_proj;

Define_Module(CarMobility);

/*!
 * @brief Destructor.
 */
CarMobility::~CarMobility() {
    cancelAndDelete(locationUpdateTimer);
}

/*
 * Interfaz del módulo.
 */

/*!
 * @brief Inicialización.
 *
 * @param stage [in] Etapa de inicialización.
 */
void CarMobility::initialize(int stage) {
    VeinsInetMobility::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        // Parameters
        locationUpdateInterval = par("locationUpdateInterval");
        vertexProximityRadius = par("vertexProximityRadius");

        // Context
        roadNetworkDatabase = omnetpp::check_and_cast<RoadNetworkDatabase*>(
                getModuleByPath(par("roadNetworkDatabaseModule")));

        if (!roadNetworkDatabase)
            throw omnetpp::cRuntimeError("No roadway database module found");

        // Self messages
        locationUpdateTimer = new omnetpp::cMessage("locationUpdateTimer");

    } else if (stage == inet::INITSTAGE_SINGLE_MOBILITY) {
        updateLocation();
        scheduleLocationUpdateTimer();
    }
}

/*!
 * @brief Manejo de mensajes.
 *
 * @param message [in] Mensaje a procesar.
 */
void CarMobility::handleMessage(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("CarMobility::handleMessage");

    if (message == locationUpdateTimer)
        processLocationUpdateTimer();

    else
        throw omnetpp::cRuntimeError("Unknown message");
}

/*
 * Actualización de la ubicación.
 */

/*!
 * @brief Programar el temporizador de actualización de la ubicación.
 */
void CarMobility::scheduleLocationUpdateTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("CarMobility::scheduleLocationUpdateTimer");

    scheduleAt(omnetpp::simTime() + locationUpdateInterval,
            locationUpdateTimer);
}
/*!
 * @brief Procesar el temporizador de actualización de la ubicacion.
 */
void CarMobility::processLocationUpdateTimer() {
    EV_DEBUG << "******************************************************************************************************************************************************************"
            << std::endl
    << "CarMobility::scheduleLocationUpdateTimer" << std::endl;
    Enter_Method
    ("CarMobility::scheduleLocationUpdateTimer");

    updateLocation();
    scheduleLocationUpdateTimer();
}

/*!
 * @brief Actualizar la ubicación.
 */
void CarMobility::updateLocation() {
    locationChanged_ = false;
    edgeChanged_ = false;
    regionChanged_ = false;

    // Obtener coordenadas cartesianas
    inet::Coord inetLocation = veins::VeinsInetMobility::getCurrentPosition();
    veins::Coord veinsLocation = veins::Coord(inetLocation.x, inetLocation.y);

    // Obtener coordenadas geogr��ficas
    double lat, lon;
    boost::tie(lon, lat) = veins::VeinsInetMobility::getCommandInterface()->getLonLat(
            veinsLocation);

    // Obtener velocidad
    inet::Coord inetSpeed = veins::VeinsInetMobility::getCurrentVelocity();
    speed = std::sqrt(inetSpeed.x * inetSpeed.x + inetSpeed.y * inetSpeed.y);

    // Obtener direcci��n
    if (speed > 0)
        direction = std::fmod(
                2.5 * 180.0
                        - inet::math::rad2deg(
                                std::atan2(-inetSpeed.y, inetSpeed.x)), 360.0);

    // Se verifica si la ubicaci��n cambi��
    if (geohashLocation.isNull()
            || !geohashLocation.getBounds().contains(lat, lon)) {
        geohashLocation.setLocation(lat, lon);

        locationChanged_ = true;

        // Se actualiza la red vial
        regionChanged_ = updateRoadNetwork();

        Edge previousEdge = locationOnRoadNetwork.edge;
        bool locationSuccess = roadNetwork->getLocationOnRoadNetwork(
                getLocation(), speed, direction, locationOnRoadNetwork);

        if (locationSuccess)
            EV_INFO << "Ubicaci��n correcta" << std::endl;

        else
            EV_INFO << "Error obtenindo ubicaci��n" << std::endl;

        edgeChanged_ = locationOnRoadNetwork.edge != previousEdge;
    }
}

/*!
 * @brief Determinar si el vehículo se encuentra en una región *gateway*.
 *
 * Si alguno de los dos vértices de la árista en la que se encuentra
 * el vehículo es *gateway*, se devuelve su tipo de adyacencia.
 *
 * @return Tipo de adyacencia de la región
 * en la que se encuentra el vehículo.
 */
GeohashLocation::Adjacency CarMobility::getGatewayRegionAdjacency() const {
    EV_DEBUG << "******************************************************************************************************************************************************************"
            << std::endl
    << "CarMobility::getGatewayRegion" << std::endl;
    Enter_Method
    ("CarMobility::getGatewayRegion");

    const Graph &graph = roadNetwork->getGraph();
    const Edge &edge = locationOnRoadNetwork.edge;
    const Vertex &vertexA = boost::source(edge, graph);
    const Vertex &vertexB = boost::target(edge, graph);
    if (graph[vertexA].adjacency != GeohashLocation::Adjacency::NONE)
        return graph[vertexA].adjacency;
    return graph[vertexB].adjacency;
}

/*!
 * @brief Verificar si el vehículo se encuentra en un vértice.
 *
 * @param vertex [in] Vértice de referencia.
 * @return `true` si el vehículo se encuentra en el vértice.
 *
 * TODO Eliminar.
 */
bool CarMobility::isAtVertex(const Vertex vertex) const {
    const Graph &graph = roadNetwork->getGraph();
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);

    // Si el v��rtice no es un v��rtice de la arista
    if (vertex != vertexA && vertex != vertexB)
        return false;

    double distanceToVertex;

    if (vertex == vertexA)
        distanceToVertex = locationOnRoadNetwork.distanceToVertexA;

    else
        distanceToVertex = locationOnRoadNetwork.distanceToVertexB;

    return distanceToVertex <= vertexProximityRadius;
}

/*!
 * @brief Verificar si el vehículo se encuentra en un vértice _gateway_.
 *
 * @return `true` si el vehículo se encuentra en un vértice _gateway_.
 * TODO Eliminar:
 */
std::pair<Vertex, bool> CarMobility::isAtGateway() const {
    const Graph &graph = roadNetwork->getGraph();
    Vertex vertexA = boost::source(locationOnRoadNetwork.edge, graph);
    Vertex vertexB = boost::target(locationOnRoadNetwork.edge, graph);

    // Si el vértice A es gateway y se encuentra dentro del radio de proximidad
    if (isGateway(vertexA, graph)) {
        if (inVertex(locationOnRoadNetwork, vertexA, graph))
            return std::pair<Vertex, bool>(vertexA, true);

        // Si el v��rtice B es gateway y se encuentra dentro del radio de proximidad
    } else if (isGateway(vertexB, graph)) {
        if (inVertex(locationOnRoadNetwork, vertexB, graph))
            return std::pair<Vertex, bool>(vertexB, true);
    }

    return std::pair<Vertex, bool>(vertexA, false);
}

/*!
 * @brief Actualizar la red vial si es necesario.
 *
 * @return `true` si se actualizó la red vial.
 */
bool CarMobility::updateRoadNetwork() {
    if (roadNetwork == nullptr
            || !roadNetwork->getGeohashRegion().getBounds().contains(
                    geohashLocation.getLocation())) {
        roadNetwork = roadNetworkDatabase->getRoadNetwork(geohashLocation);

        if (roadNetwork == nullptr)
            throw omnetpp::cRuntimeError(
                    "Couldn't find network geohash location %s",
                    geohashLocation.getGeohashString().c_str());

        return true;
    }

    return false;
}
