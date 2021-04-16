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

    /*
     * Atributos.
     */
    //! Grafo con el que se ejecuta el algoritmo.
    const Graph *graph;
    //! Arista de origen.
    Edge sourceEdge;
    //! Vector de predecesores de cada vértice.
    VertexVector predecessors;
    //! Vector de distancias de ruta hacia cada uno de los vértices.
    std::vector<double> routeDistances;

public:

    /*
     * Constructor.
     */
    /*!
     * @brief Constructor.
     *
     * @param graph      [in] Grafo con el que se ejecuta el algoritmo.
     * @param sourceEdge [in] Arista de origen.
     */
    ShortestPaths(const Graph &graph, const Edge sourceEdge);

    /*
     * Cómputo de rutas.
     */
    /*!
     * @brief Calcular la ruta más corta hacia el resto de los vértices.
     *
     * @param visitedVertices [in] Vértices visitados.
     * @param inactiveEdges   [in] Aristas inactivas.
     */
    void computeShortestPaths(const VertexSet visitedVertices,
            const EdgeSet &inactiveEdges = { });
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
    VertexVector getShortestPathToVertex(Vertex targetVertex) const;
    /*!
     * @brief Calcular el primer tramo recto en la ruta.
     *
     * @param path [in] Ruta de la que se obtiene el primer tramo recto.
     * @return Primer tramo recto de la ruta.
     */
    VertexVector getStraightPath(const VertexVector &path) const;
    /*!
     * @brief Calcular el primer tramo recto desde una arista.
     *
     * @param edge [in] Arista desde la que se busca el tramo recto.
     * @return Tramo recto desde la arista.
     */
    VertexVector getStraightPathFromEdge(const Edge edge) const;
private:
    /*!
     * @brief Calcula el peso de una arista a partir de otra.
     *
     * El peso de la arista *(vw)* desde la arista *(uv)*, cuyo vértice en común
     * es *v*, es la diferencia entre la dirección de *(uv)* desde *u*
     * y la dirección de *(vw)* desde vértice *v*.
     *
     * Para calcular el peso, ambas aristas deben ser adyacentes.
     *
     * @param sourceEdge [in] Arista desde la que se calcula el peso.
     * @param edge       [in] Arista cuyo peso se va a calcular.
     * @return Peso de la arista
     */
    double getEdgeWeight(Edge sourceEdge, Edge edge) const;
};

}    // namespace veins_proj
