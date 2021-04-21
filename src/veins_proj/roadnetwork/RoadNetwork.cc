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
 * @file RoadNetwork.cc
 * @author Adrián Juárez Monroy
 */

#include <GeographicLib/Geodesic.hpp>
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/swap.hpp>
#define BOOST_NO_CXX11_SCOPED_ENUMS
#include <boost/filesystem.hpp>
#undef BOOST_NO_CXX11_SCOPED_ENUMS
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

using namespace veins_proj;

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
RoadNetwork::RoadNetwork(std::string geohash, std::string xmlFile) :
        geohashRegion(geohash) {
    ASSERT(geohash.length() >= 6);
    ASSERT(
            boost::filesystem::exists(xmlFile)
                    && boost::filesystem::is_regular_file(xmlFile));
    boost::property_tree::ptree tree;
    boost::property_tree::read_xml(xmlFile, tree);
    /*
     * Para cada uno de los nodos `vertex` en el árbol XML,
     * se crea un vértice y se agrega al grafo.
     */
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, tree.get_child("roadnetwork.vertices")) {
        if (v.first != "vertex")
            continue;
        double lat = v.second.get<double>("<xmlattr>.lat");
        double lon = v.second.get<double>("<xmlattr>.lon");
        GeohashLocation::Adjacency adjacency =
                static_cast<GeohashLocation::Adjacency>(v.second.get<int>(
                        "<xmlattr>.adjacency"));
        GeographicLib::GeoCoords location(lat, lon);
        Vertex vertex = boost::add_vertex( { location, adjacency }, graph);
        /*
         * Si el vértice es *gateway*, se agrega al vector correspondiente.
         */
        if (adjacency != GeohashLocation::Adjacency::NONE)
            gatewayVertices[adjacency].insert(vertex);
    }
    /*
     * Para cada uno de los nodos `edge` en el árbol XML,
     * se crea una arista y se agrega al grafo.
     */
    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, tree.get_child("roadnetwork.edges")) {
        if (v.first != "edge")
            continue;
        /*
         * Se obtienen los dos vértices y se verifica si están ordenados
         * para poder calcular la dirección de cada uno.
         */
        Vertex vertexA = v.second.get<int>("<xmlattr>.vertex-a");
        Vertex vertexB = v.second.get<int>("<xmlattr>.vertex-b");
        if (!sortedVertices(vertexA, vertexB, graph))
            boost::swap(vertexA, vertexB);
        const GeographicLib::GeoCoords &locationA = graph[vertexA].location;
        const GeographicLib::GeoCoords &locationB = graph[vertexB].location;
        double length, direction1, direction2, azi;
        const GeographicLib::Geodesic &geod = GeographicLib::Geodesic::WGS84();
        geod.Inverse(locationA.Latitude(), locationA.Longitude(),
                locationB.Latitude(), locationB.Longitude(), length, direction1,
                azi);
        geod.Inverse(locationB.Latitude(), locationB.Longitude(),
                locationA.Latitude(), locationA.Longitude(), direction2, azi);
        boost::add_edge(vertexA, vertexB, { 1, length, direction1, direction2 },
                graph);
    }
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
bool RoadNetwork::getLocationOnRoadNetwork(
        const GeographicLib::GeoCoords &location, const double speed,
        const double direction,
        LocationOnRoadNetwork &locationOnRoadNetwork) const {
    /*
     * Se busca la ubicación del objeto respecto a cada arista
     * para determinar en qué arista se encuentra.
     */
    double closestDistanceToEdge = std::numeric_limits<double>::infinity();
    bool success = false;
    EdgeIt edgeIt, endEdgeIt;
    boost::tie(edgeIt, endEdgeIt) = boost::edges(graph);
    while (edgeIt != endEdgeIt) {
        Edge edge = *edgeIt++;
        Vertex vertexA = boost::source(edge, graph);
        Vertex vertexB = boost::target(edge, graph);
        /*
         * Si el objeto está en movimiento y su movimiento no coincide
         * con alguna de las direcciones de la arista, esta se descarta.
         */
        if (speed > 0) {
            if (directionMatches(direction, graph[edge].direction2))
                edge = boost::edge(vertexB, vertexA, graph).first;
            else if (!directionMatches(direction, graph[edge].direction1))
                continue;
        }
        /*
         * Se obtiene la ubicación del objeto respecto a la arista.
         * Si se encuentra dentro del dominio de esta,
         * se guarda como ubicación vial tentativa.
         */
        LocationOnRoadNetwork locationOnRoadNetworkAux;
        getOnEdgePosition(edge, location, locationOnRoadNetworkAux);
        if (inEdgeDomain(locationOnRoadNetworkAux)) {
            success = true;
            if (locationOnRoadNetworkAux.distanceToEdge
                    < closestDistanceToEdge) {
                closestDistanceToEdge = locationOnRoadNetworkAux.distanceToEdge;
                locationOnRoadNetwork = locationOnRoadNetworkAux;
            }
        }
    }
    return success;
}

bool RoadNetwork::getLocationOnRoadNetworkFromVertex(const Vertex vertex,
        const GeographicLib::GeoCoords &location, const double speed,
        const double direction,
        LocationOnRoadNetwork &locationOnRoadNetwork) const {
    /*
     * Se busca la ubicación del objeto respecto a cada arista
     * para determinar en qué arista se encuentra.
     */
    double closestDistanceToEdge = std::numeric_limits<double>::infinity();
    bool success = false;
    OutEdgeIt edgeIt, endEdgeIt;
    boost::tie(edgeIt, endEdgeIt) = boost::out_edges(vertex, graph);
    while (edgeIt != endEdgeIt) {
        Edge edge = *edgeIt++;
        Vertex vertexA = boost::source(edge, graph);
        Vertex vertexB = boost::target(edge, graph);
        /*
         * Si el objeto está en movimiento y su movimiento no coincide
         * con alguna de las direcciones de la arista, esta se descarta.
         */
        if (speed > 0) {
            if (directionMatches(direction, graph[edge].direction2))
                edge = boost::edge(vertexB, vertexA, graph).first;
            else if (!directionMatches(direction, graph[edge].direction1))
                continue;
        }
        /*
         * Se obtiene la ubicación del objeto respecto a la arista.
         * Si se encuentra dentro del dominio de esta,
         * se guarda como ubicación vial tentativa.
         */
        LocationOnRoadNetwork locationOnRoadNetworkAux;
        getOnEdgePosition(edge, location, locationOnRoadNetworkAux);
        if (inEdgeDomain(locationOnRoadNetworkAux)) {
            success = true;
            if (locationOnRoadNetworkAux.distanceToEdge
                    < closestDistanceToEdge) {
                closestDistanceToEdge = locationOnRoadNetworkAux.distanceToEdge;
                locationOnRoadNetwork = locationOnRoadNetworkAux;
            }
        }
    }
    return success;
}

/*!
 * @brief Calcular la ubicación de un objeto respecto a una arista.
 *
 * @param edge                  [in]  Arista respecto a la que se
 * busca la ubicación
 * @param location              [in]  Ubicación geográfica de la que
 * se obtiene la ubicación respecto a la arista.
 * @param locationOnRoadNetwork [out] Ubicación vial.
 */
void RoadNetwork::getOnEdgePosition(const Edge edge,
        const GeographicLib::GeoCoords &location,
        LocationOnRoadNetwork &locationOnRoadNetwork) const {
    const GeographicLib::Geodesic &geodesic = GeographicLib::Geodesic::WGS84();
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);
    /*
     * Se forma un triángulo con las ubicaciones de los dos vértices
     * y la ubicación de interés.
     */
    const GeographicLib::GeoCoords &A = graph[vertexA].location;    // Ubicación del vértice A.
    const GeographicLib::GeoCoords &B = graph[vertexB].location;    // Ubicación del vértice B.
    const GeographicLib::GeoCoords &C = location;    // Ubicación de interés.
    /*
     * Se obtienen las longitudes de cada lado del triángulo.
     */
    double a, b, c;
    c = graph[edge].length;    // Longitud del lado AB.
    geodesic.Inverse(A.Latitude(), A.Longitude(), C.Latitude(), C.Longitude(),
            b);    // Longitud del lado AC.
    geodesic.Inverse(B.Latitude(), B.Longitude(), C.Latitude(), C.Longitude(),
            a);    // Longitud del lado BC.
    /*
     * Se calcula el coseno del ángulo formado por los lado B y C
     * con el teorema del coseno.
     *
     * Debido a que las longitude de los lados no forman un triángulo perfecto,
     * se corrige el valor calculado en caso de que no esté entre -1 y 1.
     */
    double cosalpha = (b * b + c * c - a * a) / (2 * b * c);
    if (cosalpha > 1)
        cosalpha = 1;
    else if (cosalpha < -1)
        cosalpha = -1;
    /*
     * Se calcula la proyección del punto C (ubicación de interés)
     * sobre el lado AB para obtener la distancia de esta a los puntos A Y B
     * (los vértices de la arista).
     */
    double alpha = std::acos(cosalpha);
    locationOnRoadNetwork.edge = edge;
    locationOnRoadNetwork.distanceToEdge = b * std::sin(alpha);
    locationOnRoadNetwork.distanceToU = b * cosalpha;
    locationOnRoadNetwork.distanceToV = graph[edge].length
            - locationOnRoadNetwork.distanceToU;
}
