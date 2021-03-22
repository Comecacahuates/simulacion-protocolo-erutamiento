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
 * @file RoadNetworkGraph.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <GeographicLib/GeoCoords.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/tuple/tuple.hpp>
#include "veins_proj/geohash/GeohashLocation.h"
#include <omnetpp.h>
#include <vector>
#include <set>

namespace veins_proj {

//! Datos de cada vértice.
struct VertexData {
    //! Ubicación del vértice.
    GeographicLib::GeoCoords location;
    //! Tipo de _gateway_.
    GeohashLocation::Adjacency adjacency;
};

//! Datos de cada arista.
struct EdgeData {
    //! Peso de la arista.
    double weight;
    //! Longitud de la arista en metros.
    double length;
    //! Dirección del segundo vértice respecto al primero.
    double direction1;
    //! Dirección del primer vértice respecto a segundo.
    double direction2;
};

//! Grafo vial.
typedef boost::adjacency_list<boost::listS, boost::vecS, boost::undirectedS,
        VertexData, EdgeData> Graph;
//! Vértice del grafo vial.
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
//! Iterador de vértices para grafo vial.
typedef boost::graph_traits<Graph>::vertex_iterator VertexIt;
//! Arista del grafo vial.
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
//! Iterador de aristas para grafo vial.
typedef boost::graph_traits<Graph>::edge_iterator EdgeIt;
//! Iterador de aristas de salida para grafo vial.
typedef boost::graph_traits<Graph>::out_edge_iterator OutEdgeIt;
//! Iterador de aristas de entrada para grafo vial.
typedef boost::graph_traits<Graph>::in_edge_iterator InEdgeIt;

//! Vector de vértices.
typedef std::vector<Vertex> VertexVector;
//! Iterador de vértices para vector.
typedef VertexVector::iterator VertexVectorIt;
//! Iterador de vértices para vector constante.
typedef VertexVector::const_iterator VertexVectorConstIt;

//! Conjunto de vértices.
typedef std::set<Vertex> VertexSet;
//! Iterador de vértices para conjunto.
typedef VertexSet::iterator VertexSetIt;
//! Iterador de vértices para conjunto constante.
typedef VertexSet::const_iterator VertexSetConstIt;

//! Vector de aristas.
typedef std::vector<Edge> EdgeVector;
//! Iterador de aristas para vector.
typedef EdgeVector::iterator EdgeVectorIt;
//! Iterador de aristas para vector constante.
typedef EdgeVector::const_iterator EdgeVectorConstIt;

//! Conjunto de aristas.
typedef std::set<Edge> EdgeSet;
//! Iterador de aristas para conjunto.
typedef EdgeSet::iterator EdgeSetIt;
//! Iterador de aristas para conjunto constante.
typedef EdgeSet::const_iterator EdgeSetConstIt;

//! Ubicación vial de un objeto.
struct LocationOnRoadNetwork {
    //! Arista en la que se ubica el objeto.
    Edge edge;
    //! Distancia hacia la arista.
    double distanceToEdge;
    //! Sistancia hacia el primer vértice de la arista.
    double distanceToVertexA;
    //! Distancia hacia el segundo vértice de la arista.
    double distanceToVertexB;
};

/*!
 * @brief Determina si la ubicación del primer vértice es
 * menor a la del segundo.
 *
 * Para que la ubicación del primer vértice sea menor, su latitud debe ser
 * menor a la del segundo. Si las latitudes son iguales, la longitud debe ser
 * menor.
 *
 * @param vertexA [in] Primer vértice cuya ubicación se va a comparar.
 * @param vertexB [in] Segundo vértice cuya ubicación se va a comparar.
 * @param graph [in] Grafo al que pertenecen los dos vértices.
 * @return `true` si la ubicación del primer vértice es menor a la del
 * segundo.
 */
bool sortedVertices(Vertex vertexA, Vertex vertexB, const Graph &graph);
/*!
 * @brief Determina si un vértice es *gateway*.
 *
 * @param vertex [in] Vértice que se va a revisar.
 * @param graph [in] Grafo al que pertenece el vértice.
 * @return `true` en caso de que el vértice sea _gateway_.
 */
bool isGateway(Vertex vertex, const Graph &graph);
/*!
 * @brief Verifica si dos direcciones acimutales coinciden.
 *
 * Para que dos direcciones coincidan, su diferencia debe ser menor
 * a 15 grados.
 *
 * @param direction1 [in] Primera dirección en grados.
 * @param direction2 [in] Segunda dirección en grados.
 * @return `true` si las dos direcciones coinciden.
 */
bool directionMatches(double direction1, double direction2);
/*!
 *  @brief Calcula la diferencia de dos direcciones acimutales.
 *
 * La diferencia es el ángulo menor, en grados, que forman dos direcciones.
 *
 * @param direction1 [in] Primera dirección en grados.
 * @param direction2 [in] Segunda dirección en grados.
 * @return Valor absoluto de la diferencia de las dos direcciones.
 */
double getDirectionDifference(double direction1, double direction2);
/*!
 * @brief Verifica si una ubicación vial se encuentra dentro
 * del dominio de la arista.
 *
 * La ubicación vial se encuentra dentro del dominio de la arista si
 * la distancia a esta es menor a 20 metros.
 *
 * @param locationOnRoadNetwork [in] Ubicación vial a verificar.
 * @return `true` si la distancia a la arista es menor que 20.
 */
bool inEdgeDomain(const LocationOnRoadNetwork &locationOnRoadNetwork);
/*!
 * @brief Obtiene la distancia en metros de una ubicación vial hacia un vértice.
 *
 * Para obtener la distancia, el vértice debe ser parte de la arista de la
 * ubicación vial.
 *
 * @param locationOnRoadNetwork [in] Ubicación vial cuya distancia al vértice
 * se va a obtener.
 * @param vertex [in] Vértice al que se va a obtener la distancia.
 * @param graph  [in] Grafo al que pertenece el vértice.
 * @return Distancia al vértice en metros, o infinito si no se encuentra
 * en una arista con el vértice.
 */
double getDistanceToVertex(const LocationOnRoadNetwork &locationOnRoadNetwork,
        Vertex vertex, const Graph &graph);
}    // namespace veins_proj
