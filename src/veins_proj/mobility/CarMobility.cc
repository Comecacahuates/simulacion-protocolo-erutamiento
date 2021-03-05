/*
 * VehicleMobility.cc
 *
 *  Created on: Jun 16, 2020
 *      Author: adrian
 */

#include "veins_proj/mobility/CarMobility.h"
#include "veins/base/utils/Coord.h"
#include "inet/common/INETMath.h"
#include "inet/common/geometry/common/Coord.h"
#include <boost/math/constants/constants.hpp>
#include <boost/tuple/tuple.hpp>
#include "veins_proj/geohash/GeohashLocation.h"
#include <utility>
#include <iomanip>
#include <cmath>
#include <iomanip>

using namespace veins_proj;


Define_Module(CarMobility);


CarMobility::~CarMobility() {
    cancelAndDelete(locationUpdateTimer);
}


void CarMobility::initialize(int stage) {
    VeinsInetMobility::initialize(stage);

    if (stage == inet::INITSTAGE_LOCAL) {
        // Parameters
        locationUpdateInterval = par("locationUpdateInterval");
        vertexProximityRadius = par("vertexProximityRadius");

        // Context
        roadNetworkDatabase = omnetpp::check_and_cast<RoadNetworkDatabase *>(getModuleByPath(par("roadNetworkDatabaseModule")));

        if (!roadNetworkDatabase)
            throw omnetpp::cRuntimeError("No roadway database module found");

        // Self messages
        locationUpdateTimer = new omnetpp::cMessage("locationUpdateTimer");

    } else if (stage == inet::INITSTAGE_SINGLE_MOBILITY) {
        updateLocation();
        scheduleLocationUpdateTimer();
    }
}


void CarMobility::handleMessage(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("CarMobility::handleMessage");

    if (message == locationUpdateTimer)
        processLocationUpdateTimer();

    else
        throw omnetpp::cRuntimeError("Unknown message");
}


void CarMobility::scheduleLocationUpdateTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("CarMobility::scheduleLocationUpdateTimer");

    scheduleAt(omnetpp::simTime() + locationUpdateInterval, locationUpdateTimer);
}


void CarMobility::processLocationUpdateTimer() {
    EV_INFO << "******************************************************************************************************************************************************************" << std::endl;
    Enter_Method("CarMobility::scheduleLocationUpdateTimer");

    updateLocation();
    scheduleLocationUpdateTimer();
}


void CarMobility::updateLocation() {
    locationChanged_ = false;
    edgeChanged_ = false;
    regionChanged_ = false;

    // Obtener coordenadas cartesianas
    inet::Coord inetLocation = veins::VeinsInetMobility::getCurrentPosition();
    veins::Coord veinsLocation = veins::Coord(inetLocation.x, inetLocation.y);

    // Obtener coordenadas geogr��ficas
    double lat, lon;
    boost::tie(lon, lat) = veins::VeinsInetMobility::getCommandInterface()->getLonLat(veinsLocation);

    // Obtener velocidad
    inet::Coord inetSpeed = veins::VeinsInetMobility::getCurrentVelocity();
    speed = std::sqrt(inetSpeed.x * inetSpeed.x + inetSpeed.y * inetSpeed.y);

    // Obtener direcci��n
    if (speed > 0)
        direction = std::fmod(2.5 * 180.0 - inet::math::rad2deg(std::atan2(-inetSpeed.y, inetSpeed.x)), 360.0);

    // Se verifica si la ubicaci��n cambi��
    if (geohashLocation.isNull() || !geohashLocation.getBounds().contains(lat, lon)) {
        geohashLocation.setLocation(lat, lon);

        locationChanged_ = true;

        // Se actualiza la red vial
        regionChanged_ = updateRoadNetwork();

        Edge previousEdge = locationOnRoadNetwork.edge;
        bool locationSuccess = roadNetwork->getLocationOnRoadNetwork(getLocation(), speed, direction, locationOnRoadNetwork);

        if (locationSuccess)
            EV_INFO << "Ubicaci��n correcta" << std::endl;

        else
            EV_INFO << "Error obtenindo ubicaci��n" << std::endl;

        edgeChanged_ = locationOnRoadNetwork.edge != previousEdge;
    }
}


std::pair<Vertex, bool> CarMobility::isAtGateway() const {
    const Graph &graph = roadNetwork->getGraph();
    Vertex vertexA = boost::source(locationOnRoadNetwork.edge, graph);
    Vertex vertexB = boost::target(locationOnRoadNetwork.edge, graph);

    // Si el v��rtice A es gateway y se encuentra dentro del radio de proximidad
    if (isGateway(vertexA, graph)) {
        if (inVertexProximityRadius(locationOnRoadNetwork, vertexA, graph))
            return std::pair<Vertex, bool>(vertexA, true);

    // Si el v��rtice B es gateway y se encuentra dentro del radio de proximidad
    } else if (isGateway(vertexB, graph)) {
        if (inVertexProximityRadius(locationOnRoadNetwork, vertexB, graph))
            return std::pair<Vertex, bool>(vertexB, true);
    }

    return std::pair<Vertex, bool>(vertexA, false);
}


bool CarMobility::isAtVertex(const Vertex vertex) const {
    const Graph &graph = roadNetwork->getGraph();
    const Edge &edge = locationOnRoadNetwork.edge;
    Vertex vertexA = boost::source(edge, graph);
    Vertex vertexB = boost::target(edge, graph);

    // Si el v��rtice no es un v��rtice de la arista
    if (vertex != vertexA && vertex != vertexB)
        return false;

    double distanceToVertex;

    if (vertex == vertexA)
        distanceToVertex = locationOnRoadNetwork.distanceToVertex1;

    else
        distanceToVertex = locationOnRoadNetwork.distanceToVertex2;

    return distanceToVertex <= vertexProximityRadius;
}


bool CarMobility::updateRoadNetwork() {
    if (roadNetwork == nullptr || !roadNetwork->getGeohashRegion().getBounds().contains(geohashLocation.getLocation())) {
        roadNetwork = roadNetworkDatabase->getRoadNetwork(geohashLocation);

        if (roadNetwork == nullptr)
            throw omnetpp::cRuntimeError("Couldn't find network geohash location %s", geohashLocation.getGeohashString().c_str());

        return true;
    }

    return false;
}
