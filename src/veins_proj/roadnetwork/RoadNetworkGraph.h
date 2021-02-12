/*
 * RoadNetworkGraph.h
 *
 *  Created on: Dec 31, 2020
 *      Author: adrian
 */

#pragma once

#include <GeographicLib/GeoCoords.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/tuple/tuple.hpp>
#include "veins_proj/geohash/GeohashLocation.h"
#include <omnetpp.h>
#include <vector>

#define VERTEX_PROXIMITY_RADIUS 10


namespace veins_proj {


struct VertexData {
    GeographicLib::GeoCoords location;
    GeohashLocation::Direction gatewayType;
};

struct EdgeData {
    double weight;
    double length;
    double direction1;
    double direction2;
};

typedef boost::adjacency_list<boost::listS,
                              boost::vecS,
                              boost::undirectedS,
                              VertexData,
                              EdgeData> Graph;
typedef boost::graph_traits<Graph>::vertex_descriptor Vertex;
typedef boost::graph_traits<Graph>::vertex_iterator VertexIterator;
typedef boost::graph_traits<Graph>::edge_descriptor Edge;
typedef boost::graph_traits<Graph>::edge_iterator EdgeIterator;
typedef boost::graph_traits<Graph>::out_edge_iterator OutEdgeIterator;
typedef boost::graph_traits<Graph>::in_edge_iterator InEdgeIterator;
typedef std::vector<Vertex> VertexVector;
typedef VertexVector::iterator VertexVectorIterator;
typedef VertexVector::const_iterator VertexVectorConstIterator;
typedef std::vector<Edge> EdgeVector;
typedef EdgeVector::iterator EdgeVectorIterator;
typedef EdgeVector::const_iterator EdgeVectorConstIterator;

struct LocationOnRoadNetwork {
    Edge edge;
    double distanceToEdge;
    double distanceToVertexA;
    double distanceToVertexB;
};

bool sortedVertices(Vertex vertexA, Vertex vertexB, const Graph &graph);
bool isGateway(Vertex vertex, const Graph &graph);
bool directionMatches(double direction1, double direction2);
double getDirectionDifference(double direction1, double direction2);
bool inEdgeDomain(const LocationOnRoadNetwork &locationOnRoadNetwork);
bool inVertexProximityRadius(Vertex vertex, const LocationOnRoadNetwork &locationOnRoadNetwork, const Graph &graph);
double getEdgeWeight(Edge edge, Edge sourceEdge, const Graph &graph);
double getDistanceToVertex(Vertex vertex, const LocationOnRoadNetwork &locationOnRoadNetwork, const Graph &graph);


} // namespace veins_proj
