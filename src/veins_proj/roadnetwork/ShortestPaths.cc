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
 * @file ShortestPaths.cc
 * @author Adrián Juárez Monroy
 */

#include "veins_proj/roadnetwork/ShortestPaths.h"

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/detail/adjacency_list.hpp>
#include <boost/iterator/iterator_facade.hpp>
#include <boost/tuple/detail/tuple_basic.hpp>
#include <algorithm>
#include <utility>

using namespace veins_proj;

/*
 * Constructor.
 */

/*!
 * @brief Constructor.
 *
 * @param graph      [in] Grafo con el que se ejecuta el algoritmo.
 * @param sourceEdge [in] Arista de origen.
 */
ShortestPaths::ShortestPaths(const Graph &graph, Edge sourceEdge) :
                graph(&graph),
                sourceEdge(sourceEdge),
                predecessors(boost::num_vertices(graph)),
                routeDistances(boost::num_vertices(graph)) {
}

/*
 * Cómputo de rutas.
 */

/*!
 * @brief Calcular la ruta más corta hacia el resto de los vértices.
 *
 * @param visitedVertices [in] Vértices visitados.
 * @param inactiveEdges   [in] Aristas inactivas.
 */
void ShortestPaths::computeShortestPaths(const VertexSet visitedVertices,
        const EdgeSet &activeEdges) {
    const Graph &graph = *this->graph;
    const Edge &ab = sourceEdge;
    const Vertex a = boost::source(sourceEdge, graph);    // Primer vértice de la arista de origen.
    const Vertex b = boost::target(sourceEdge, graph);    // Segundo vértice de la arista de origen.
    const VertexSet &VV = visitedVertices;
    const EdgeSet &AE = activeEdges;
    /*
     * Se inicializan los vectores de predecesores y distancias de ruta.
     * La distancia de ruta de los vértices de la arista de origen valen 0.
     */
    std::fill(predecessors.begin(), predecessors.end(),
            std::numeric_limits<Vertex>::max());
    std::fill(routeDistances.begin(), routeDistances.end(),
            std::numeric_limits<double>::infinity());
    routeDistances[a] = 0;
    routeDistances[b] = 0;
    /*
     * Se inicializa el conjunto de vértices que restan por procesar.
     * Los vértices de la arista de origen no se procesan.
     */
    VertexSet S;
    {
        VertexIt it, endIt;
        boost::tie(it, endIt) = boost::vertices(graph);
        while (it != endIt)
            S.insert(*it++);
    }
    S.erase(a);
    S.erase(b);
    /*
     * Se establece el vértice *a* como predecesor de sus vértices adyacentes
     * y se calcula la distancia de ruta de estos, excepto los vértices
     * visitados o que sean parte de una arista inactiva.
     */
    OutEdgeIt it, endIt;
    boost::tie(it, endIt) = boost::out_edges(a, graph);
    while (it != endIt) {
        const Edge &av = *it;
        const Vertex v = boost::target(av, graph);
        if (v != b) {
            if (!VV.count(v) && AE.count(av)) {
                routeDistances[v] = getEdgeWeight(av, ab);
                predecessors[v] = a;
            } else
                S.erase(v);
        }
        it++;
    }
    /*
     * Se establece el vértice *b* como predecesor de sus nodos adyacentes
     * y se calcula la distancia de ruta de estos, excepto los vértices
     * visitados o que sean parte de una arista inactiva.
     */
    boost::tie(it, endIt) = boost::out_edges(b, graph);
    while (it != endIt) {
        const Edge &bv = *it;
        Vertex v = boost::target(bv, graph);
        if (v != a) {
            if (!VV.count(v) && AE.count(bv)) {
                double weight = getEdgeWeight(bv, ab);
                if (routeDistances[v] > weight) {
                    routeDistances[v] = weight;
                    predecessors[v] = b;
                }
            } else
                S.erase(v);
        }
        it++;
    }
    /*
     * Se procesan cada uno de los vértices restantes en *S*.
     */
    while (!S.empty()) {
        /*
         * Se busca el vértice en *S* con la menor distancia y se guarda en *v*.
         * Si no se encuentra el vértice, significa que los vértices restantes
         * en *S* son inalcanzables, por lo que se vacía *S*.
         */
        Vertex v;
        bool vertexFound = false;
        {
            VertexSetConstIt it = S.begin();
            VertexSetConstIt endIt = S.end();
            double minDistance = std::numeric_limits<double>::infinity();
            while (it != endIt) {
                double distance = routeDistances[*it];
                if (minDistance > distance) {
                    minDistance = distance;
                    v = *it;
                    vertexFound = true;
                }
                it++;
            }
        }
        if (!vertexFound)
            break;
        /*
         * Se calcula la distancia a cada vértice *w* adyacentes a *v*.
         */
        OutEdgeIt it, endIt;
        boost::tie(it, endIt) = boost::out_edges(v, graph);
        while (it != endIt) {
            Edge vw = *it;    // Arista cuyo peso se va a calcular.
            Vertex w = boost::target(vw, graph);    // Vértice adyacente a *v*.
            Vertex u = predecessors[v];    // Vértice predecesor de *v*.
            Edge uv = boost::edge(u, v, graph).first;    // Arista desde la que se va a calcular el peso.
            if (w != u) {
                if (!VV.count(w)) {
                    double weight = getEdgeWeight(vw, uv);
                    if (routeDistances[w] > routeDistances[v] + weight) {
                        routeDistances[w] = routeDistances[v] + weight;
                        predecessors[w] = v;
                    }
                } else
                    S.erase(w);
            }
            it++;
        }
        S.erase(v);
    }
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
VertexVector ShortestPaths::getShortestPathToVertex(Vertex vertex) const {
    Vertex a = boost::source(sourceEdge, *graph);
    Vertex b = boost::target(sourceEdge, *graph);
    VertexVector shortestPath = { vertex };
    while (!(vertex == a || vertex == b)) {
        vertex = predecessors[vertex];
        shortestPath.push_back(vertex);
    }
    if (vertex == a)
        shortestPath.push_back(b);
    else
        shortestPath.push_back(a);
    std::reverse(shortestPath.begin(), shortestPath.end());
    return shortestPath;
}

/*!
 * @brief Calcula el peso de una arista a partir de otra.
 *
 * El peso de la arista _A_ desde la arista _B_, cuyo vértice en común es _v_,
 * es la diferencia entre la dirección de la arista _A_ desde el vértice _v_
 * y la dirección de la arista _B_ hacia el vértice _v_.
 *
 * Para calcular el peso, ambas aristas deben ser adyacentes.
 *
 * @param edge [in] Arista cuyo peso se va a calcular.
 * @param sourceEdge [in] Arista desde la que se calcula el peso.
 * @param graph [in] Grafo al que pertenecen ambas aristas.
 * @return Peso de la arista
 */
double ShortestPaths::getEdgeWeight(const Edge edge,
        const Edge sourceEdge) const {
    const Graph &graph = *this->graph;
    const Edge &e1 = sourceEdge;
    const Edge &e2 = edge;
    Vertex vertexA1 = boost::source(e1, graph);
    Vertex vertexB1 = boost::target(e1, graph);

    if (!sortedVertices(vertexA1, vertexB1, graph))
        boost::swap(vertexA1, vertexB1);

    Vertex vertexA2 = boost::source(e2, graph);
    Vertex vertexB2 = boost::target(e2, graph);
    if (!sortedVertices(vertexA2, vertexB2, graph))
        boost::swap(vertexA2, vertexB2);

    double directionE1, directionE2;

    if (vertexB1 == vertexA2) {
        directionE1 = graph[e1].direction1;
        directionE2 = graph[e2].direction1;

    } else if (vertexB1 == vertexB2) {
        directionE1 = graph[e1].direction1;
        directionE2 = graph[e2].direction2;

    } else if (vertexA1 == vertexA2) {
        directionE1 = graph[e1].direction2;
        directionE2 = graph[e2].direction1;

    } else /* if (vertexA1 == vertexB2) */{
        directionE1 = graph[e1].direction2;
        directionE2 = graph[e2].direction2;
    }

    return getDirectionDifference(directionE1, directionE2);
}

