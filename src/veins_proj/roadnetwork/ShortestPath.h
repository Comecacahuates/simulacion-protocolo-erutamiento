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
 * \file ShortestPath.h
 * \author Adrián Juárez Monroy
 */
#pragma once

#include <omnetpp.h>
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include <vector>

namespace veins_proj {

//! Clase que implementa el algoritmo de distancia más corta.
/*!
 * Contiene métodos para calcular la ruta más corta desde una arista hacia
 * el resto los vértices de un grafo, y para obtener la ruta hacia un vértice
 * en particular.
 */
class ShortestPath: public omnetpp::cObject {

protected:

	//! Vector de predecesores de cada vértice.
	/*!
	 * El valor de cada elemento indica el vértice predecesor, y el índice
	 * indica de qué vertice es predecesor.
	 */
	VertexVector predecessors;

	//! Vector de distancias de ruta de cada vértice.
	/*!
	 * Indica la distancia de la ruta desde la arista de origen hacia cada
	 * uno de los vértices.
	 * */
	std::vector<double> routeDistances;

	//! Arista de origen.
	Edge sourceEdge;

public:

	//! Calcula la ruta más corta hacia el resto de los vértices.
	/*!
	 * \param [in] sourceEdge Arista de origen.
	 * \param [in] graph Grafo por el que se realiza la búsqueda.
	 * \param [in] inactiveEdges Aristas inactivas.
	 */
	void computeShortestPath(const Edge sourceEdge, const Graph &graph,
			const VertexVector &unavailableVertices = { },
			const EdgeVector &inactiveEdges = { });

	//! Devuelve la distancia de ruta a un vértice.
	/*!
	 * Una vez calculada la ruta más corta al resto de los vértices, devuelve
	 * la distancia de ruta a un vértice desde la arista de origen. Se
	 * necesita calcular las rutas con el método #computeShortestPath antes
	 * de llamar este método.
	 *
	 * \param [in] vertex Vértice del que se quiere la ruta más corta.
	 * \return Vector de vértices que indican la ruta desde el vértice de la
	 * arista de origen.
	 */
	double getRouteDistance(Vertex vertex) const {
		return routeDistances[vertex];
	}

	//! Devuelve la ruta más corta a un vértice específico.
	/*!
	 * Una vez calculada las rutas más cortas, devuelve la ruta más corta a
	 * un vértice. Se necesita calcular las rutas con el método
	 * #computeShortestPath antes de llamar este método.
	 *
	 * \param [in] vertex Vértice del que se quiere la ruta más corta.
	 * \return Vector de vértices que indican la ruta desde el vértice de la
	 * arista de origen.
	 */
	VertexVector getShortestPathToVertex(Vertex targetVertex,
			const Graph &graph) const;
};

} // namespace veins_proj
