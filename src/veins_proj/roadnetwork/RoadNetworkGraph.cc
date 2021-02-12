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

#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "boost/swap.hpp"
#include <utility>
#include <limits>

using namespace veins_proj;


bool veins_proj::sortedVertices(Vertex vertexA, Vertex vertexB, const Graph &graph) {
    const GeographicLib::GeoCoords &locationA = graph[vertexA].location;
    const GeographicLib::GeoCoords &locationB = graph[vertexB].location;

    if (locationA.Latitude() > locationB.Latitude())
        return false;

    else if (locationA.Latitude() == locationB.Latitude())
        if (locationA.Longitude() > locationB.Longitude())
            return false;

    return true;
}


bool veins_proj::isGateway(Vertex vertex, const Graph &graph) {
    return graph[vertex].gatewayType != GeohashLocation::Direction::NONE;
}


bool veins_proj::directionMatches(double direction1, double direction2) {
    return getDirectionDifference(direction1, direction2) < 15.0;
}


double veins_proj::getDirectionDifference(double direction1, double direction2) {
    double directionDifference = std::abs(direction1 - direction2);

    if (directionDifference > 180.0)
        directionDifference = 360.0 - directionDifference;

    return directionDifference;
}


bool veins_proj::inEdgeDomain(const LocationOnRoadNetwork &locationOnRoadNetwork) {
    // Si la distancia al vértice A está dentro del dominio
    if (locationOnRoadNetwork.distanceToVertexA < 0)
        return false;

    if (locationOnRoadNetwork.distanceToVertexB < 0)
        return false;

    if (locationOnRoadNetwork.distanceToEdge > 20.0)
        return false;

    return true;
}


bool veins_proj::inVertexProximityRadius(Vertex vertex, const LocationOnRoadNetwork &locationOnRoadNetwork, const Graph &graph) {
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);
    const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertexA;
    const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertexB;

    double distanceToVertex;
    if (vertex == vertexA)
        distanceToVertex = distanceToVertexA;

    else if (vertex == vertexB)
        distanceToVertex = distanceToVertexB;

    else
        return false;

    if (distanceToVertex < VERTEX_PROXIMITY_RADIUS)
        return true;

    return false;
}


double veins_proj::getEdgeWeight(Edge f, Edge e, const Graph &graph) {
    Vertex vertexAE = boost::source(e, graph);
    Vertex vertexBE = boost::target(e, graph);
    if (sortedVertices(vertexAE, vertexBE, graph))
        boost::swap(vertexAE, vertexBE);

    Vertex vertexAF = boost::source(f, graph);
    Vertex vertexBF = boost::target(f, graph);
    if (sortedVertices(vertexAF, vertexBF, graph))
        boost::swap(vertexAF, vertexBF);

    double directionE, directionF;

    if (vertexBE == vertexAF) {
        directionE = graph[e].direction1;
        directionF = graph[f].direction1;

    } else if (vertexBE == vertexBF) {
        directionE = graph[e].direction1;
        directionF = graph[f].direction2;

    } else if (vertexAE == vertexAF) {
        directionE = graph[e].direction2;
        directionF = graph[f].direction1;

    } else if (vertexAE == vertexBF) {
        directionE = graph[e].direction2;
        directionF = graph[f].direction2;
    }

    return getDirectionDifference(directionE, directionF);
}


double veins_proj::getDistanceToVertex(Vertex vertex, const LocationOnRoadNetwork &locationOnRoadNetwork, const Graph &graph) {
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);
    const double &distanceToVertexA = locationOnRoadNetwork.distanceToVertexA;
    const double &distanceToVertexB = locationOnRoadNetwork.distanceToVertexB;

    if (vertex == vertexA)
        return distanceToVertexA;

    else if (vertex == vertexB)
        return distanceToVertexB;

    return std::numeric_limits<double>::infinity();
}
