/*
 * RoadNetwork.h
 *
 *  Created on: Jul 13, 2020
 *      Author: adrian
 */

#pragma once

#include <GeographicLib/Math.hpp>
#include <GeographicLib/GeoCoords.hpp>
#include <omnetpp.h>
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include <string>
#include <vector>
#include <utility>

namespace veins_proj {


class RoadNetwork: public omnetpp::cObject {

protected:
    GeohashLocation geohashRegion;
    Graph graph;
    VertexVector gatewayVertices[4];

public:
    RoadNetwork(std::string geohash, std::string fileName);

    const Graph &getGraph() const { return graph; }

    const GeohashLocation &getGeohashRegion() const { return geohashRegion; }

    const VertexVector &getGatewayVertices(GeohashLocation::Direction direction) const { return gatewayVertices[direction]; }

    bool getLocationOnRoadNetwork(const GeographicLib::GeoCoords &location, const double speed, const double direction, LocationOnRoadNetwork &locationOnRoadNetwork) const;

    bool getLocationOnRoadNetworkFromVertex(const Vertex vertex, const GeographicLib::GeoCoords &location, const double speed, const double direction, LocationOnRoadNetwork &locationOnRoadNetwork) const;

    void getOnEdgePosition(const Edge edge, const GeographicLib::GeoCoords &location, LocationOnRoadNetwork &locationOnRoadNetwork) const;
};


} // namespace veins_proj
