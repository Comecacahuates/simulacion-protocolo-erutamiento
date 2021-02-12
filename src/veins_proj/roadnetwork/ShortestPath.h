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

#pragma once

#include <omnetpp.h>
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include <vector>

namespace veins_proj {


class ShortestPath : public omnetpp::cObject {

protected:
    VertexVector predecessors;
    std::vector<double> routeDistances;
    Vertex vertexA;
    Vertex vertexB;

public:
    void computeShortestPath(const Edge sourceEdge, const Graph &graph, const VertexVector &unavailableVertices = {}, const EdgeVector &unavailableEdges = {});
    double getRouteDistance(Vertex vertex) const { return routeDistances[vertex]; }
    VertexVector getShortestPathToVertex(Vertex targetVertex, const Graph &graph) const;
};


} // namespace veins_proj
