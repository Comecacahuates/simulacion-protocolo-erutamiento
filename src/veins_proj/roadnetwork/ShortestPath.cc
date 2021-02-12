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

#include "veins_proj/roadnetwork/ShortestPath.h"
#include "boost/tuple/tuple.hpp"
#include <algorithm>

using namespace veins_proj;


void ShortestPath::computeShortestPath(const Edge e, const Graph &graph, const VertexVector &unavailableVertices, const EdgeVector &unavailableEdges) {
    vertexA = boost::source(e, graph);
    vertexB = boost::target(e, graph);
    predecessors.resize(boost::num_vertices(graph));
    routeDistances.resize(boost::num_vertices(graph));
    std::fill(routeDistances.begin(), routeDistances.end(), std::numeric_limits<double>::infinity());

    int n = boost::num_vertices(graph);
    std::vector<bool> visited(n, false); // Indica los v��rtices de los que ya se tiene la distancia m��nima
    Vertex a = boost::source(e, graph);
    Vertex b = boost::target(e, graph);

    // Se analizan los nodos sucesores de a
    OutEdgeIterator outEdgeIt, endOutEdgeIt;
    boost::tie(outEdgeIt, endOutEdgeIt) = boost::out_edges(a, graph);
    while (outEdgeIt != endOutEdgeIt) {
        Edge f = *outEdgeIt++;
        Vertex v = boost::target(f, graph);

        if (v != b) {
            routeDistances[v] = getEdgeWeight(f, e, graph);
            predecessors[v] = a;
        }
    }

    // Se analizan los nodos sucesores de b
    boost::tie(outEdgeIt, endOutEdgeIt) = boost::out_edges(b, graph);
    while (outEdgeIt != endOutEdgeIt) {
        Edge f = *outEdgeIt++;
        Vertex v = boost::target(f, graph);

        if (v != a) {
            double weight = getEdgeWeight(f, e, graph);

            if (routeDistances[v] > weight) {
                routeDistances[v] = weight;
                predecessors[v] = b;
            }
        }
    }

    routeDistances[a] = 0;
    routeDistances[b] = 0;
    visited[a] = true;
    visited[b] = true;

    // Se marcan como visitados los v��rtices inalcanzables
    for (auto w : unavailableVertices)
        visited[w] = true;

    int numVisited = 2 + unavailableVertices.size();

    VertexIterator vertexIt, endVertexIt;

    while (numVisited < n) {
        double minDistance = std::numeric_limits<double>::infinity();
        Vertex v; // V��rtice no visto con la menor distancia

        // Se busca el v��rtice no visto con la menor distancia
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
        while(outEdgeIt != endOutEdgeIt) {
            Edge f = *outEdgeIt++;
            Vertex w = boost::target(f, graph);

            // Si no ha sido visitado
            if (!visited[w]) {
                double distance = routeDistances[v] + getEdgeWeight(f, boost::edge(predecessors[v], v, graph).first, graph);
                if (routeDistances[w] > distance) {
                    routeDistances[w] = distance;
                    predecessors[w] = v;
                }
            }
        }
    }
}


VertexVector ShortestPath::getShortestPathToVertex(Vertex v, const Graph &graph) const {
    VertexVector shortestPath;

    shortestPath.push_back(v);

    do {
        v = predecessors[v];
        shortestPath.push_back(v);

    } while (!(v == vertexA || v == vertexB));

    if (v == vertexA)
        shortestPath.push_back(vertexB);

    else
        shortestPath.push_back(vertexA);

    std::reverse(shortestPath.begin(), shortestPath.end());

    return shortestPath;
}
