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

/**
 * \file ShortestPath.cc
 * @author Adrián Juárez Monroy
 *
 */

#include "ShortestPath.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/detail/adjacency_list.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/tuple/detail/tuple_basic.hpp>
#include <algorithm>
#include <limits>
#include <utility>

using namespace veins_proj;

//! Calcula la ruta más corta hacia el resto de los vértices.
/*!
 * \param [in] sourceEdge Arista de origen.
 * \param [in] graph Grafo por el que se realiza la búsqueda.
 * \param [in] inactiveEdges Aristas inactivas.
 */
void ShortestPath::computeShortestPath(const Edge sourceEdge,
		const Graph &graph, const VertexVector &unavailableVertices,
		const EdgeVector &inactiveEdges) {
	this->sourceEdge = sourceEdge;

	int n = boost::num_vertices(graph); // Número de vértices del grafo
	std::vector<bool> visited(n, false); // Indica los vértices de los que ya se tiene la distancia mínima
	Vertex a = boost::source(sourceEdge, graph); // Primer vértice de la arista de origen
	Vertex b = boost::target(sourceEdge, graph); // Segundo vértice de la arista de origen
	VertexVector &NV = unavailableVertices;
	EdgeVector &NE = inactiveEdges;

	predecessors.resize(n);
	routeDistances.resize(n);
	std::fill(routeDistances.begin(), routeDistances.end(),
			std::numeric_limits<double>::infinity());

	//Se inicializan los vértices de la arista inicial
	routeDistances[a] = 0;
	routeDistances[b] = 0;
	visited[a] = true;
	visited[b] = true;

	/*
	 * Se establece el vértice a como predecesor de sus nodos adyacentes
	 * y se calcula la distancia de ruta de estos, excepto los vértices
	 * no disponibles o que sean parte de una arista inactiva
	 */
	OutEdgeIterator outEdgeIt, endOutEdgeIt;
	boost::tie(outEdgeIt, endOutEdgeIt) = boost::out_edges(a, graph);
	while (outEdgeIt != endOutEdgeIt) {
		Edge f = *outEdgeIt++;
		Vertex u = boost::target(f, graph);

		if (u != b) {
			routeDistances[u] = getEdgeWeight(f, sourceEdge, graph);
			predecessors[u] = a;
		}
	}

	/*
	 * Se establece el vértice b como predecesor de sus nodos adyacentes
	 * y se calcula la distancia de ruta de estos, excepto los vértices
	 * no disponibles o que sean parte de una arista inactiva
	 */
	boost::tie(outEdgeIt, endOutEdgeIt) = boost::out_edges(b, graph);
	while (outEdgeIt != endOutEdgeIt) {
		Edge f = *outEdgeIt++;
		Vertex v = boost::target(f, graph);

		if (v != a) {
			double weight = getEdgeWeight(f, sourceEdge, graph);

			if (routeDistances[v] > weight) {
				routeDistances[v] = weight;
				predecessors[v] = b;
			}
		}
	}

	// Se marcan como visitados los vértices inalcanzables
	for (auto w : unavailableVertices)
		visited[w] = true;

	int numVisited = 2 + unavailableVertices.size();

	VertexIterator vertexIt, endVertexIt;

	while (numVisited < n) {
		double minDistance = std::numeric_limits<double>::infinity();
		Vertex v; // Vértice no visto con la menor distancia

		// Se busca el vértice no visto con la menor distancia
		boost::tie(vertexIt, endVertexIt) = boost::vertices(graph);
		while (vertexIt != endVertexIt) {
			Vertex w = *vertexIt++;

			if (!visited[w] && minDistance > routeDistances[w]) {
				minDistance = routeDistances[w];
				v = w;
			}
		}

		visited[v] = true;
		numVisited++;

		// Para cada sucesor de v
		boost::tie(outEdgeIt, endOutEdgeIt) = boost::out_edges(v, graph);
		while (outEdgeIt != endOutEdgeIt) {
			Edge f = *outEdgeIt++;
			Vertex w = boost::target(f, graph);

			// Si no ha sido visitado
			if (!visited[w]) {
				double distance = routeDistances[v]
						+ getEdgeWeight(f,
								boost::edge(predecessors[v], v, graph).first,
								graph);
				if (routeDistances[w] > distance) {
					routeDistances[w] = distance;
					predecessors[w] = v;
				}
			}
		}
	}
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
VertexVector ShortestPath::getShortestPathToVertex(Vertex vertex,
		const Graph &graph) const {
	Vertex a = boost::source(sourceEdge, graph);
	Vertex b = boost::target(sourceEdge, graph);
	VertexVector shortestPath;

	shortestPath.push_back(vertex);

	do {
		vertex = predecessors[vertex];
		shortestPath.push_back(vertex);

	} while (!(vertex == a || vertex == b));

	if (vertex == a)
		shortestPath.push_back(b);

	else
		shortestPath.push_back(a);

	std::reverse(shortestPath.begin(), shortestPath.end());

	return shortestPath;
}
