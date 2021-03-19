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
 * @file ShortestPaths.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <omnetpp.h>
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include <vector>
#include <limits>

namespace veins_proj {

//! Clase que implementa el algoritmo de distancia más corta.
/*!
 * Contiene métodos para calcular la ruta más corta desde una arista hacia
 * el resto los vértices de un grafo, y para obtener la ruta hacia un vértice
 * en particular.
 */
class ShortestPaths: public omnetpp::cObject {

protected:

    /*!
     * @brief Vector de predecesores de cada vértice.
     *
     * El valor de cada elemento indica el vértice predecesor, y el índice
     * indica de qué vertice es predecesor.
     */
    VertexVector predecessors;
    /*!
     * @brief Vector de distancias de ruta desde la arista de origen hacia cada
     * uno de los vértices.
     * */
    std::vector<double> routeDistances;

    //! Arista de origen.
    Edge sourceEdge;

public:

    /*!
     * @brief Calcular la ruta más corta hacia el resto de los vértices.
     *
     * @param sourceEdge      [in] Arista de origen.
     * @param graph           [in] Grafo por el que se realiza la búsqueda.
     * @param visitedVertices [in] Vértices por los que el paquete ya ha pasado.
     * @param activeEdges     [in] Aristas inactivas.
     */
    void computeShortestPath(const Edge sourceEdge, const Graph &graph,
            const VertexSet &visitedVertices, const EdgeSet &activeEdges);
    /*!
     * @brief Obtener la distancia de ruta a un vértice.
     *
     * Una vez calculada la ruta más corta al resto de los vértices, devuelve
     * la distancia de ruta a un vértice desde la arista de origen. Se
     * necesita calcular las rutas con el método #computeShortestPath antes
     * de llamar este método.
     *
     * @param [in] vertex Vértice del que se quiere la ruta más corta.
     * @return Vector de vértices que indican la ruta desde el vértice de la
     * arista de origen.
     */
    double getRouteDistance(const Vertex vertex) const {
        return routeDistances[vertex];
    }
    /*!
     * @brief Verificar si se encontró una ruta a un vértice.
     *
     * Si el valor de la distancia de ruta de un vértice es infinito,
     * no se encontró una ruta a este. Se necesita calcular las rutas
     * con el método #computeShortestPath antes de llamar este método.
     *
     * @param vertex [in] Vértice que se quiere verificar.
     * @return `true` si existe una ruta hacia el vértice.
     */
    bool routeToVertexFound(const Vertex vertex) const {
        return routeDistances[vertex] != std::numeric_limits<double>::infinity();
    }
    /*!
     * @brief Obtener la ruta más corta a un vértice específico.
     *
     * Una vez calculada las rutas más cortas, devuelve la ruta más corta a
     * un vértice. Se necesita calcular las rutas con el método
     * #computeShortestPath antes de llamar este método.
     *
     * @param [in] vertex Vértice del que se quiere la ruta más corta.
     * @return Vector de vértices que indican la ruta desde el vértice de la
     * arista de origen.
     */
    VertexVector getShortestPathToVertex(Vertex targetVertex,
            const Graph &graph) const;
};

}    // namespace veins_proj
