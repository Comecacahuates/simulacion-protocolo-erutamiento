/*
 * RoadNetwork.cc
 *
 *  Created on: Jul 13, 2020
 *      Author: adrian
 */

#include <GeographicLib/Geodesic.hpp>
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/xml_parser.hpp>
#include <boost/foreach.hpp>
#include <boost/swap.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>

using namespace veins_proj;

RoadNetwork::RoadNetwork(std::string geohash, std::string fileName) :
        geohashRegion(geohash) {

    boost::property_tree::ptree tree;

    boost::property_tree::read_xml(fileName, tree);

    double lat, lon;
    GeohashLocation::Adjacency adjacency;
    unsigned int i = 0;

    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, tree.get_child("roadnetwork.vertices")) {
        if (v.first != "vertex")
            continue;

        lat = v.second.get<double>("<xmlattr>.lat");
        lon = v.second.get<double>("<xmlattr>.lon");
        adjacency = static_cast<GeohashLocation::Adjacency>(v.second.get<int>(
                "<xmlattr>.adjacency"));
        GeographicLib::GeoCoords location(lat, lon);

        boost::add_vertex( { location, adjacency }, graph);

        if (adjacency != GeohashLocation::Adjacency::NONE)
            gatewayVertices[adjacency].push_back(i);
        i++;
    }

    Vertex vertexA, vertexB;
    double length, direction1, direction2;

    BOOST_FOREACH(boost::property_tree::ptree::value_type &v, tree.get_child("roadnetwork.edges")) {
        if (v.first != "edge")
            continue;

        vertexA = v.second.get<int>("<xmlattr>.vertex-a");
        vertexB = v.second.get<int>("<xmlattr>.vertex-b");
        if (sortedVertices(vertexA, vertexB, graph))
            boost::swap(vertexA, vertexB);

        const GeographicLib::GeoCoords &locationA = graph[vertexA].location;
        const GeographicLib::GeoCoords &locationB = graph[vertexB].location;
        double azi;
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

bool RoadNetwork::getLocationOnRoadNetwork(
        const GeographicLib::GeoCoords &location, const double speed,
        const double direction,
        LocationOnRoadNetwork &locationOnRoadNetwork) const {
    bool success = false;

    Edge edge;
    LocationOnRoadNetwork locationOnRoadNetworkAux;

    EdgeIterator edgeIt, endEdgeIt;
    boost::tie(edgeIt, endEdgeIt) = boost::edges(graph);

    double closestDistanceToEdge = std::numeric_limits<double>::infinity();

    while (edgeIt != endEdgeIt) {
        edge = *edgeIt;
        edgeIt++;
        Vertex vertexA = boost::source(edge, graph);
        Vertex vertexB = boost::target(edge, graph);

        if (speed > 0) {
            // Verificar la direcci��n del veh��culo
            if (directionMatches(direction, graph[edge].direction2))
                edge = boost::edge(vertexB, vertexA, graph).first;

            else if (!directionMatches(direction, graph[edge].direction1))
                continue;
        }

        // Obtener y verificar la posici��n del veh��culo respecto a la arista
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
    bool success = false;

    Edge edge;
    LocationOnRoadNetwork locationOnRoadNetworkAux;

    OutEdgeIterator edgeIt, endEdgeIt;
    boost::tie(edgeIt, endEdgeIt) = boost::out_edges(vertex, graph);

    double closestDistanceToEdge = std::numeric_limits<double>::infinity();

    while (edgeIt != endEdgeIt) {
        edge = *edgeIt;
        edgeIt++;
        Vertex vertexA = boost::source(edge, graph);
        Vertex vertexB = boost::target(edge, graph);

        if (speed > 0) {
            // Verificar la direcci��n del veh��culo
            if (directionMatches(direction, graph[edge].direction2))
                edge = boost::edge(vertexB, vertexA, graph).first;

            else if (!directionMatches(direction, graph[edge].direction1))
                continue;
        }

        // Obtener y verificar la posici��n del veh��culo respecto a la arista
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

void RoadNetwork::getOnEdgePosition(const Edge edge,
        const GeographicLib::GeoCoords &location,
        LocationOnRoadNetwork &locationOnRoadNetwork) const {
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);

    const GeographicLib::GeoCoords &A = graph[vertexA].location;
    const GeographicLib::GeoCoords &B = graph[vertexB].location;
    const GeographicLib::GeoCoords &C = location;

    double a, b, c;

    const GeographicLib::Geodesic &geodesic = GeographicLib::Geodesic::WGS84();

    c = graph[edge].length;

    geodesic.Inverse(A.Latitude(), A.Longitude(), C.Latitude(), C.Longitude(),
            b);
    geodesic.Inverse(B.Latitude(), B.Longitude(), C.Latitude(), C.Longitude(),
            a);

    double cosalpha = (b * b + c * c - a * a) / (2 * b * c);

    if (cosalpha > 1)
        cosalpha = 1;
    else if (cosalpha < -1)
        cosalpha = -1;

    double alpha = std::acos(cosalpha);

    locationOnRoadNetwork.edge = edge;
    locationOnRoadNetwork.distanceToEdge = b * std::sin(alpha);
    locationOnRoadNetwork.distanceToVertexA = b * cosalpha;
    locationOnRoadNetwork.distanceToVertexB = graph[edge].length
            - locationOnRoadNetwork.distanceToVertexA;
}
