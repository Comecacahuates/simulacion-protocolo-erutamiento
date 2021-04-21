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
                routeDistances[v] = getEdgeWeight(ab, av);
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
                double weight = getEdgeWeight(ab, bv);
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
                    double weight = getEdgeWeight(uv, vw);
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
 * @brief Calcular el primer tramo recto en la ruta.
 *
 * @param path [in] Ruta de la que se obtiene el primer tramo recto.
 * @return Primer tramo recto más largo desde el inicio de la ruta.
 */
VertexVector ShortestPaths::getStraightPath(const VertexVector &path) const {
    VertexVectorConstIt it = path.begin();
    VertexVectorConstIt endIt = std::prev(path.end());
    while (it != endIt) {
        if (routeDistances[*endIt] < 15)
            break;
        endIt--;
    }
    VertexVector straightPath(it, std::next(endIt));
    return straightPath;
}

/*!
 * @brief Calcular el primer tramo recto desde una arista.
 *
 * @param edge        [in] Arista desde la que se busca el tramo recto.
 * @param includeEdge [in] Incluir arista en el tramo recto.
 * @return Tramo recto desde la arista.
 */
VertexVector ShortestPaths::getStraightPathFromEdge(const Edge edge,
        bool includeEdge) const {
    const Graph &graph = *this->graph;
    Vertex u = boost::source(edge, graph);
    Vertex v = boost::target(edge, graph);
    Vertex w = v;
    VertexVector straightPath;
    if (includeEdge)
        straightPath.push_back(u);
    straightPath.push_back(v);
    double minDistance = 0.0;
    /*
     * Se buscan las aristas que formen tramos rectos.
     */
    do {
        /*
         * Se recorren los vértices adyacentes a *v* y se busca uno que
         * forme un tramo recto.
         */
        OutEdgeIt it, endIt;
        boost::tie(it, endIt) = boost::out_edges(v, graph);
        while (it != endIt) {
            Edge uv = boost::edge(u, v, graph).first;
            Edge vw = *it;
            /*
             * Se verifica si *vw* forma un tramo recto con *uv*.
             */
            double distance = minDistance + getEdgeWeight(uv, vw);
            if (distance < 15) {
                minDistance = distance;
                w = boost::target(vw, graph);
                break;
            }
            it++;
        }
        /*
         * Si se encontró una arista *vw* que forme un tramo recto,
         * se agrega *w* al tramo recto y se recorren *u* y *v*.
         */
        if (w != v)
            straightPath.push_back(w);
        u = v;
        v = w;
        /*
         * Mientras se encuentre una arista nueva para el tramo recto.
         */
    } while (u != v);
    return straightPath;
}

/*!
 * @brief Verificar si una secuencia de aristas forma un tramo recto.
 *
 * @param path [in] Secuencia de aristas a verificar.
 * @return `true` si la secuencia de aristas forma un tramo recto.
 */
bool ShortestPaths::isStraightPath(const VertexVector &path) const {
    const Graph &graph = *this->graph;
    /*
     * Se recorre cada vértice y se revisa su distancia de ruta
     * para saber si forma un tramo recto.
     */
    bool isStraightPath = true;
    for (int i = 0; i < path.size() - 2; i++) {
        Vertex u = path[i];
        Vertex v = path[i + 1];
        Vertex w = path[i + 2];
        Edge uv = boost::edge(u, v, graph).first;
        Edge vw = boost::edge(v, w, graph).first;
        double weight = getEdgeWeight(uv, vw);
        if (weight > 15) {
            isStraightPath = false;
            break;
        }
    }
    return isStraightPath;
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
 *         arista de origen.
 */
VertexVector ShortestPaths::getShortestPathToVertex(const Vertex vertex) const {
    Vertex u = boost::source(sourceEdge, *graph);
    Vertex v = boost::target(sourceEdge, *graph);
    Vertex w = vertex;
    /*
     * Se agrega a la ruta el vértice buscado y sus predecesores.
     */
    VertexVector shortestPath = { w };
    while (w != u && w != v) {
        w = predecessors[w];
        shortestPath.push_back(w);
    }
    if (w == u)
        shortestPath.push_back(v);
    else
        shortestPath.push_back(u);
    /*
     * Se invierte el orden de la ruta para que al inicio esté el primer
     * vértice de la ruta y al final el vértice de destino.
     */
    std::reverse(shortestPath.begin(), shortestPath.end());
    return shortestPath;
}

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
double ShortestPaths::getEdgeWeight(const Edge sourceEdge,
        const Edge edge) const {
    const Graph &graph = *this->graph;
    const Edge &e1 = sourceEdge;
    const Edge &e2 = edge;
    Vertex u1 = boost::source(e1, graph);
    Vertex v1 = boost::target(e1, graph);

    if (!sortedVertices(u1, v1, graph))
        boost::swap(u1, v1);

    Vertex u2 = boost::source(e2, graph);
    Vertex v2 = boost::target(e2, graph);
    if (!sortedVertices(u2, v2, graph))
        boost::swap(u2, v2);

    double direction1, direction2;

    if (v1 == u2) {
        direction1 = graph[e1].direction1;
        direction2 = graph[e2].direction1;

    } else if (v1 == v2) {
        direction1 = graph[e1].direction1;
        direction2 = graph[e2].direction2;

    } else if (u1 == u2) {
        direction1 = graph[e1].direction2;
        direction2 = graph[e2].direction1;

    } else /* if (vertexA1 == vertexB2) */{
        direction1 = graph[e1].direction2;
        direction2 = graph[e2].direction2;
    }

    return getDirectionDifference(direction1, direction2);
}

