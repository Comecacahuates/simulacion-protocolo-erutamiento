/*
 * VehicleMobility.h
 *
 *  Created on: Jun 16, 2020
 *      Author: adrian
 */

#pragma once

#include <GeographicLib/GeoCoords.hpp>
#include <omnetpp.h>
#include "veins_inet/VeinsInetMobility.h"
#include "inet/common/lifecycle/LifecycleOperation.h"
#include "veins_proj/veins_proj.h"
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "veins_proj/roadnetwork/RoadNetworkDatabase.h"
#include <vector>
#include <algorithm>
#include <utility>

namespace veins_proj {


class CarMobility : public veins::VeinsInetMobility {

protected:
    // Parameters
    omnetpp::simtime_t locationUpdateInterval;
    double vertexProximityRadius;

    // Context
    RoadNetworkDatabase *roadNetworkDatabase = nullptr;

    // Internal
    GeohashLocation geohashLocation;
    LocationOnRoadNetwork locationOnRoadNetwork;
    double speed;
    double direction;
    RoadNetwork *roadNetwork = nullptr;

    // State
    bool locationChanged_;
    bool edgeChanged_;
    bool regionChanged_;

    // Self messages
    omnetpp::cMessage *locationUpdateTimer;

public:
    virtual ~CarMobility();

protected:
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessage(omnetpp::cMessage *message) override;

    // Location update timer
    void scheduleLocationUpdateTimer();
    void processLocationUpdateTimer();

public:
    const GeohashLocation &getGeohashLocation() const { return geohashLocation; }
    const GeographicLib::GeoCoords &getLocation() const { return geohashLocation.getLocation(); }
    const double getSpeed() const { return speed; }
    const double getDirection() const { return direction; }
    const RoadNetwork *getRoadNetwork() const { return roadNetwork; }
    const LocationOnRoadNetwork &getLocationOnRoadNetwork() const { return locationOnRoadNetwork; }

    bool locationChanged() const { return locationChanged_; }
    bool edgeChanged() const { return edgeChanged_; }
    bool regionChanged() const { return regionChanged_; }

    void updateLocation();
    std::pair<Vertex, bool> isAtGateway() const;
    bool isAtVertex(const Vertex vertex) const;

private:
    bool updateRoadNetwork();
};


} // namespace veins_proj
