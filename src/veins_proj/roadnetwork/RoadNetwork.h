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
 * @file RoadNetwork.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <GeographicLib/Math.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include <omnetpp.h>
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include <string>
#include <vector>
#include <utility>

namespace veins_proj {

/*!
 * @brief Clase que representa una red vial, y contiene métodos para
 * calcular la ubicación vial y la ubicación respecto a una arista.
 */
class RoadNetwork: public omnetpp::cObject {

protected:

    /*
     * Atributos.
     */
    //! Región Geohash que delimita la red vial.
    GeohashLocation geohashRegion;
    //! Grafo que representa la topología de la red vial.
    Graph graph;
    //! Conjuntos de vértices *gateway*.
    VertexSet gatewayVertices[4];

public:

    /*
     * Constructor.
     */
    /*!
     * @brief Constructor.
     *
     * @param geohash [in] Región Geohash.
     * @param xmlFile [in] Ruta del archivo XML en la base de datos
     * de redes viales que contiene la información de la red vial.
     */
    RoadNetwork(std::string geohash, std::string xmlFile);

    /*
     * Acceso a los atributos.
     */
    /*!
     * @brief Acceso al grafo.
     *
     * @return Grafo de la red vial.
     */
    const Graph& getGraph() const {
        return graph;
    }
    /*!
     * @brief Acceso a la región Geohash.
     *
     * @return Región Geohash que delimita la red vial.
     */
    const GeohashLocation& getGeohashRegion() const {
        return geohashRegion;
    }
    /*!
     * @brief Acceso a los vértices *gateway*.
     *
     * @param adjacency [in] Adyacencia de los vértices *gateway*
     * que se quieren obtener.
     * @return Vértices *gateway*.
     */
    const VertexSet& getGatewayVertices(
            GeohashLocation::Adjacency adjacency) const {
        return gatewayVertices[adjacency];
    }

    /*
     * Cálculo de la ubicación vial.
     */
    /*!
     * @brief Calcular la ubicación vial de un objeto.
     *
     * @param location              [in] Ubicación geográfica de la que
     * se obtiene la ubicación vial.
     * @param speed                 [in] Velocidad de movimiento
     * en metros por segundo.
     * @param direction             [in] Ángulo acimutal, en grados,
     * de la dirección de movimiento.
     * @param locationOnRoadNetwork [out] Ubicación vial.
     * @return `true` si se calculó correctamente la ubicación vial.
     */
    bool getLocationOnRoadNetwork(const GeographicLib::GeoCoords &location,
            const double speed, const double direction,
            LocationOnRoadNetwork &locationOnRoadNetwork) const;
    /*!
     * @brief Calcular la ubicación vial de un objeto a partir de un vértice
     * del grafo.
     *
     * @param vertex                [in] Vértice desde el que se busca
     * la ubicación vial.
     * @param location              [in] Ubicación geográfica de la que
     * se obtiene la ubicación vial.
     * @param speed                 [in] Velocidad de movimiento
     * en metros por segundo.
     * @param direction             [in] Ángulo acimutal, en grados,
     * de la dirección de movimiento.
     * @param locationOnRoadNetwork [out] Ubicación vial.
     * @return `true` si se calculó correctamente la ubicación vial.
     *
     * TODO: Buscar dónde usar este método.
     */
    bool getLocationOnRoadNetworkFromVertex(const Vertex vertex,
            const GeographicLib::GeoCoords &location, const double speed,
            const double direction,
            LocationOnRoadNetwork &locationOnRoadNetwork) const;
    /*!
     * @brief Calcular la ubicación de un objeto respecto a una arista.
     *
     * @param edge                  [in]  Arista respecto a la que se
     * busca la ubicación
     * @param location              [in]  Ubicación geográfica de la que
     * se obtiene la ubicación respecto a la arista.
     * @param locationOnRoadNetwork [out] Ubicación vial.
     */
    void getOnEdgePosition(const Edge edge,
            const GeographicLib::GeoCoords &location,
            LocationOnRoadNetwork &locationOnRoadNetwork) const;
};

}    // namespace veins_proj
