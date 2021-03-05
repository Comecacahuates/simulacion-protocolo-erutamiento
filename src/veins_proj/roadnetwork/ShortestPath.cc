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
 * @file ShortestPath.cc
 * @author Adrián Juárez Monroy
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
 * @param [in] sourceEdge Arista de origen.
 * @param [in] graph Grafo por el que se realiza la búsqueda.
 * @param [in] visitedVertices Vértices por los que el paquete ya ha pasado.
 * @param [in] inactiveEdges Aristas inactivas.
 */
void ShortestPath::computeShortestPath(const Edge sourceEdge,
		const Graph &graph, const VertexSet &visitedVertices,
		const EdgeSet &activeEdges) {
	this->sourceEdge = sourceEdge;

	int n = boost::num_vertices(graph); // Número de vértices del grafo.

	const Edge &ab = sourceEdge; // Arista de origen.
	Vertex a = boost::source(sourceEdge, graph); // Primer vértice de la arista de origen.
	Vertex b = boost::target(sourceEdge, graph); // Segundo vértice de la arista de origen.
	const VertexSet &VV = visitedVertices;
	const EdgeSet &AE = activeEdges;

	/*
	 * Se inicializan los vectores de distancias y predecesores para que cada
	 * vértice tenga su propio índice. Las distancias de los vértices de la
	 * arista de origen tienen distancia 0.
	 */
	predecessors.resize(n);
	routeDistances.resize(n);
	std::fill(routeDistances.begin(), routeDistances.end(),
			std::numeric_limits<double>::infinity());
	routeDistances[a] = 0;
	routeDistances[b] = 0;

	/*
	 * Se inicializa el conjunto de vértices que restan por procesar para que
	 * contenga todos los vértices, excepto los vértices de la arista de origen.
	 */
	VertexIterator vertexIt, endVertexIt;
	boost::tie(vertexIt, endVertexIt) = boost::vertices(graph);
	VertexSet S(vertexIt, endVertexIt);
	S.erase(a);
	S.erase(b);

	/*
	 * Se establece el vértice _a_ como predecesor de sus nodos adyacentes
	 * y se calcula la distancia de ruta de estos, excepto los vértices
	 * visitados o que sean parte de una arista inactiva.
	 */
	OutEdgeIterator outEdgeIt, endOutEdgeIt;
	boost::tie(outEdgeIt, endOutEdgeIt) = boost::out_edges(a, graph);
	while (outEdgeIt != endOutEdgeIt) {
		Edge av = *outEdgeIt++;
		Vertex v = boost::target(av, graph);

		if (v != b) {
			if (!VV.count(v) && AE.count(av)) {
				routeDistances[v] = getEdgeWeight(av, ab, graph);
				predecessors[v] = a;

			} else
				S.erase(v);
		}
	}

	/*
	 * Se establece el vértice _b_ como predecesor de sus nodos adyacentes
	 * y se calcula la distancia de ruta de estos, excepto los vértices
	 * visitados o que sean parte de una arista inactiva.
	 */
	boost::tie(outEdgeIt, endOutEdgeIt) = boost::out_edges(b, graph);
	while (outEdgeIt != endOutEdgeIt) {
		Edge bv = *outEdgeIt++;
		Vertex v = boost::target(bv, graph);

		if (v != a) {
			if (!VV.count(v) && AE.count(bv)) {
				double weight = getEdgeWeight(bv, ab, graph);

				if (routeDistances[v] > weight) {
					routeDistances[v] = weight;
					predecessors[v] = b;
				}

			} else
				S.erase(v);
		}
	}

	/*
	 * Se obtiene la distancia de cada uno de los vértices restantes de _S_.
	 */
	VertexSetIterator vertexSetIt, endVertexSetIt;
	while (!S.empty()) {
		/*
		 * Se busca el vértice en _S_ con la menor distancia y se guarda
		 * en _v_.
		 */
		Vertex v;
		vertexSetIt = S.begin();
		endVertexSetIt = S.end();
		double minDistance = std::numeric_limits<double>::infinity();
		while (vertexSetIt != endVertexSetIt) {
			double distance = routeDistances[*vertexSetIt];
			if (minDistance > distance) {
				minDistance = distance;
				v = *vertexSetIt;
			}
			vertexSetIt++;
		}

		/*
		 * Se calcula la distancia a cada vértice _w_ adyacentes a _v_.
		 */
		boost::tie(outEdgeIt, endOutEdgeIt) = boost::out_edges(v, graph);
		while (outEdgeIt != endOutEdgeIt) {
			Edge vw = *outEdgeIt; // Arista cuyo peso se va a calcular.
			Vertex w = boost::target(vw, graph); // Vértice adyacente a _v_.
			Vertex u = predecessors[v]; // Vértice predecesor de _v_.
			Edge uv = boost::edge(u, v, graph).first; // Arista desde la que se va a calcular el peso.

			if (w != u) {
				if (!VV.count(w)) {
					double weight = getEdgeWeight(vw, uv, graph);
					if (routeDistances[w] > routeDistances[v] + weight) {
						routeDistances[w] = routeDistances[v] + weight;
						predecessors[w] = v;
					}

				} else
					S.erase(w);
			}

			outEdgeIt++;
		}

		S.erase(v);
	}
}

//! Devuelve la ruta más corta a un vértice específico.
/*!
 * Una vez calculada las rutas más cortas, devuelve la ruta más corta a
 * un vértice. Se necesita calcular las rutas con el método
 * #computeShortestPath antes de llamar este método.
 *
 * @param [in] vertex Vértice del que se quiere la ruta más corta.
 * @return Vector de vértices que indican la ruta desde el vértice de la
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
