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

    /*
     * Etapa de inicialización local.
     */
    if (stage == inet::INITSTAGE_LOCAL) {
        /*
         * Parámetros de configuración.
         */
        vertexProximityRadius = par("vertexProximityRadius");

        /*
         * Contexto.
         */
        roadNetworkDatabase = omnetpp::check_and_cast<RoadNetworkDatabase*>(
                getModuleByPath(par("roadNetworkDatabaseModule")));
        if (!roadNetworkDatabase)
            throw omnetpp::cRuntimeError("No roadway database module found");

        /*
         * Etapa de inicialización de movilidad.
         */
    } else if (stage == inet::INITSTAGE_SINGLE_MOBILITY) {
        updateLocation();
    }
}

/*!
 * @brief Actualizar la ubicación.
 */
void CarMobility::updateLocation() {
    // Obtener coordenadas cartesianas
    inet::Coord inetLocation = veins::VeinsInetMobility::getCurrentPosition();
    veins::Coord veinsLocation = veins::Coord(inetLocation.x, inetLocation.y);

    // Obtener coordenadas geográficas
    double lat, lon;
    boost::tie(lon, lat) =
            veins::VeinsInetMobility::getCommandInterface()->getLonLat(
                    veinsLocation);

    // Obtener velocidad
    inet::Coord inetSpeed = veins::VeinsInetMobility::getCurrentVelocity();
    speed = std::sqrt(inetSpeed.x * inetSpeed.x + inetSpeed.y * inetSpeed.y);

    // Obtener dirección
    if (speed > 0)
        direction = std::fmod(
                2.5 * 180.0
                        - inet::math::rad2deg(
                                std::atan2(-inetSpeed.y, inetSpeed.x)), 360.0);

    GeographicLib::GeoCoords location(lat, lon);

    // Se verifica si la ubicación cambió
    if (geohashLocation.isNull() || !geohashLocation.contains(location)) {
        geohashLocation.setLocation(location);

        // Se actualiza la red vial
        updateRoadNetwork();

        bool locationSuccess = roadNetwork->getLocationOnRoadNetwork(
                getLocation(), speed, direction, locationOnRoadNetwork);

        if (locationSuccess)
            EV_INFO << "Ubicación correcta" << std::endl;

        else
            EV_INFO << "Error obtenindo ubicación" << std::endl;
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
             << "CarMobility::getGatewayRegion"
             << std::endl;
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
 */
bool CarMobility::isAtVertex(const Vertex vertex) const {
    const Graph &graph = roadNetwork->getGraph();
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);
    const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertexA;
    const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertexB;
    if (vertex == vertexA)
        return distanceToVertexA < 10;
    else if (vertex == vertexB)
        return distanceToVertexB < 10;
    else
        return false;
}

/*!
 * @brief Actualizar la red vial si es necesario.
 */
void CarMobility::updateRoadNetwork() {
    if (roadNetwork == nullptr
            || !roadNetwork->getGeohashRegion().getBounds().contains(
                    geohashLocation.getLocation())) {
        roadNetwork = roadNetworkDatabase->getRoadNetwork(geohashLocation);

        if (roadNetwork == nullptr)
            throw omnetpp::cRuntimeError(
                    "Couldn't find network geohash location %s",
                    geohashLocation.getGeohash().c_str());
    }
}
