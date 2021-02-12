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

#include "veins_proj/networklayer/configurator/AddressCache.h"
#include "veins_inet/veins_inet.h"
#include <omnetpp.h>
#include "inet/common/packet/Packet.h"
#include "inet/common/Ptr.h"
#include "inet/networklayer/common/InterfaceTable.h"
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "inet/networklayer/contract/INetfilter.h"
#include "inet/networklayer/ipv6/Ipv6RoutingTable.h"
#include "veins_proj/routing/RoutingProtocolBase.h"
#include "veins_proj/routing/Routing_m.h"
#include "veins_proj/locationservice/HostsLocationTable.h"
#include "veins_proj/mobility/CarMobility.h"
#include "veins_proj/roadnetwork/RoadNetworkDatabase.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/roadnetwork/ShortestPath.h"
#include <string>
#include <map>
#include <utility>

namespace veins_proj {


class RoutingProtocolCar : public RoutingProtocolBase {

protected:
    struct NeighbouringHostEntry {
        omnetpp::simtime_t lastUpdateTime;
        GeohashLocation geohashLocation;
    };

protected:
    // Context
    CarMobility *mobility = nullptr;

    // Internal
    omnetpp::simtime_t lastEdgeValidityTime;

    // Neighbouring cars by edge
    typedef std::pair<Edge, inet::Ipv6Address> NeighbouringCarByEdge;
    typedef std::multimap<Edge, inet::Ipv6Address> NeighbouringCarsByEdgeMap;
    typedef NeighbouringCarsByEdgeMap::iterator NeighbouringCarsByEdgeIterator;
    typedef NeighbouringCarsByEdgeMap::const_iterator NeighbouringCarsByEdgeConstIterator;
    NeighbouringCarsByEdgeMap neighbouringCarsByEdge;

    // Neighbouring hosts
    typedef std::pair<inet::Ipv6Address, NeighbouringHostEntry> NeighbouringHost;
    typedef std::map<inet::Ipv6Address, NeighbouringHostEntry> NeighbouringHostsMap;
    typedef NeighbouringHostsMap::iterator NeighbouringHostsIterator;
    typedef NeighbouringHostsMap::const_iterator NeighbouringHostsConstIterator;
    NeighbouringHostsMap neighbouringHosts;

    // Active edges
    typedef std::pair<Edge, omnetpp::simtime_t> ActiveEdge;
    typedef std::map<Edge, omnetpp::simtime_t> ActiveEdgesMap;
    typedef ActiveEdgesMap::iterator ActiveEdgesIterator;
    typedef ActiveEdgesMap::const_iterator ActiveEdgesConstIterator;
    ActiveEdgesMap activeEdges;

    // Inactive edges
    typedef std::pair<Edge, omnetpp::simtime_t> InactiveEdge;
    typedef std::map<Edge, omnetpp::simtime_t> InactiveEdgesMap;
    typedef InactiveEdgesMap::iterator InactiveEdgesIterator;
    typedef InactiveEdgesMap::const_iterator InactiveEdgesConstIterator;
    InactiveEdgesMap inactiveEdges;

    // Pong timeout timers
    typedef std::pair<Edge, PongTimeout *> PongTimeoutTimer;
    typedef std::map<Edge, PongTimeout *> PongTimeoutTimersMap;
    typedef PongTimeoutTimersMap::iterator PongTimeoutTimersIterator;
    typedef PongTimeoutTimersMap::const_iterator PongTimeoutTimersConstIterator;
    PongTimeoutTimersMap pongTimeoutTimers;

    // Delayed packets
    typedef std::pair<Edge, inet::Packet *> DelayedPacket;
    typedef std::multimap<Edge, inet::Packet *> DelayedPacketsMultimap;
    typedef DelayedPacketsMultimap::iterator DelayedPacketsIterator;
    typedef DelayedPacketsMultimap::const_iterator DelayedPacketsConstIterator;
    DelayedPacketsMultimap delayedPackets;

    // Self messages
    omnetpp::cMessage *helloCarTimer;
    omnetpp::cMessage *pongTimeoutTimer;
    omnetpp::cMessage *purgeNeighbouringHostsTimer;
    omnetpp::cMessage *purgeActiveEdgesTimer;
    omnetpp::cMessage *purgeInactiveEdgesTimer;

protected:
    // Module interface
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;

    //  Message handling
    virtual void processSelfMessage(omnetpp::cMessage *message) override;

    // Hello car timer
    virtual void scheduleHelloCarTimer();
    virtual void processHelloCarTimer();

    // Pong timeout timer
    virtual void schedulePongTimeoutTimer();
    virtual void processPongTimeoutTimer();

    // Purge neighbouring hosts timer
    virtual void schedulePurgeNeighbouringHostsTimer();
    virtual void processPurgeNeighbouringHostsTimer();

    // Purge active edges timer
    virtual void schedulePurgeActiveEdgesTimer();
    virtual void processPurgeActiveEdgesTimer();

    // Purge inactive edges timer
    virtual void schedulePurgeInactiveEdgesTimer();
    virtual void processPurgeInactiveEdgesTimer();

    // Hello car
    virtual const inet::Ptr<HelloCar> createHelloCar(const inet::Ipv6Address &carAddress) const;
    virtual void sendHelloCar(const inet::Ptr<HelloCar> &helloCar, const inet::Ipv6Address &destAddress);
    virtual void processHelloCar(const inet::Ptr<HelloCar> &helloCar) override;

    // Hello host
    virtual void processHelloHost(const inet::Ptr<HelloHost> &helloHost) override;

    // Ping
    virtual const inet::Ptr<Ping> createPing(const inet::Ipv6Address &carAddress, Vertex vertexA, Vertex vertexB) const;
    virtual void sendPing(const inet::Ptr<Ping> &ping, const inet::Ipv6Address &destAddress);
    virtual void processPing(const inet::Ptr<Ping> &ping) override;

    // Pong
    virtual const inet::Ptr<Pong> createPong(const inet::Ipv6Address &destAddress, bool N, omnetpp::simtime_t validityTime, int neighboursAverage, Vertex vertexA, Vertex vertexB) const;
    virtual void sendPong(const inet::Ptr<Pong> &pong, const inet::Ipv6Address &destAddress);
    virtual void processPong(const inet::Ptr<Pong> &pong) override;

    // Neighbouring cars
    void show() const;
    void removeOldNeighbouringCars(omnetpp::simtime_t time) override;
    inet::Ipv6Address getRandomNeighbouringCarAddressAheadOnEdge(Vertex targetVertex) const;
    inet::Ipv6Address getNeighbouringCarAddressOnEdgeClosestToVertex(Vertex targetVertex);
    int getNeighbouringCarsOnEdgeCount() const;

    // Neighbouring hosts
    void removeOldNeighbouringHosts(omnetpp::simtime_t time);
    omnetpp::simtime_t getOldestNeighbouringHostTime() const;
    omnetpp::simtime_t getNextNeighbouringHostExpirationTime() const;
    void purgeNeighbouringHosts();

    // Active edges
    void removeOldActiveEdges(omnetpp::simtime_t time);
    omnetpp::simtime_t getOldestActiveEdgeTime() const;
    omnetpp::simtime_t getNextActiveEdgeExpirationTime() const;
    void purgeActiveEdges();

    // Inactive edges
    void removeOldInactiveEdges(omnetpp::simtime_t time);
    omnetpp::simtime_t getOldestInactiveEdgeTime() const;
    omnetpp::simtime_t getNextInactiveEdgeExpirationTime() const;
    void purgeInactiveEdges();

    // Routing
    inet::INetfilter::IHook::Result routeDatagram(inet::Packet *datagram, const inet::Ipv6Address &destAddress) override;
    bool validateHopByHopOptionsHeader(inet::Packet *datagram) const;
    Vertex getLocalDestVertex(inet::Packet *datagram, const ShortestPath &shortestPath) const;
    VertexVector getUnavailableVertices(TlvRoutingOption *routingOption) const;
    Vertex getDestVertex(const GeohashLocation &destGeohashLocation, Edge destEdge, const ShortestPath &shortestPath) const;
    EdgeVector getReachableEdges(const VertexVector &shortestPathToDestVertex, const ShortestPath &shortestPath) const;
    inet::Ipv6Address findNextHop(const VertexVector &shortestPathToDestVertex, const ShortestPath &shortestPath) const;
    inet::Ipv6Address findNeighbourInNeighbourinRegion(const GeohashLocation &neighbouringGeohashRegion) const;

    // Netfilter
    virtual inet::INetfilter::IHook::Result datagramPreRoutingHook(inet::Packet *datagram) override;
    virtual inet::INetfilter::IHook::Result datagramLocalOutHook(inet::Packet *datagram) override;

    // Lifecycle
    virtual void handleStartOperation(inet::LifecycleOperation *operation) override;
    virtual void handleStopOperation(inet::LifecycleOperation *operation) override;
    virtual void handleCrashOperation(inet::LifecycleOperation *operation) override;

    // Notification
    virtual void receiveSignal(omnetpp::cComponent *source, omnetpp::simsignal_t signalID, omnetpp::cObject *obj, cObject *details) override;
};


} // namespace veins_proj
