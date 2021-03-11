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
 * @file CarConfigurator.cc
 * @author Adrián Juárez Monroy
 */

#include "veins_proj/networklayer/configurator/CarConfigurator.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/geometry/common/Coord.h"
#include "veins/base/utils/Coord.h"
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/networklayer/ipv6/Ipv6GeohashAddress.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/roadnetwork/RoadNetwork.h"
#include "boost/tuple/tuple.hpp"
#include "boost/swap.hpp"

using namespace veins_proj;

Define_Module(CarConfigurator);

/*
 * Interfaz del módulo.
 */

/*!
 * @brief Inicialización.
 *
 * @param stage [in] Etapa de inicialización.
 */
void CarConfigurator::initialize(int stage) {
    ConfiguratorBase::initialize(stage);

    /*
     * Etapa de inicialización local.
     */
    if (stage == inet::INITSTAGE_LOCAL) {
        /*
         * Parámetros de configuración.
         */
        locationUpdateInterval = par("locationUpdateInterval");

        /*
         * Contexto.
         */
        mobility = omnetpp::check_and_cast<CarMobility*>(
                host->getSubmodule("mobility"));
        if (!mobility)
            throw omnetpp::cRuntimeError("No mobility module found");

        /*
         * Mensajes propios.
         */
        locationUpdateTimer = new omnetpp::cMessage("LocationUpdateTimer");
    }
}

/*!
 * @brief Manejo de mensajes.
 *
 * @param message [in] Mensaje a procesar.
 */
void CarConfigurator::handleMessageWhenUp(omnetpp::cMessage *message) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("CarConfigurator::handleMessageWhenUp");

    if (message->isSelfMessage())
        processSelfMessage(message);
}

/*
 * Manejo de mensajes.
 */

/*!
 * @brief Manejo de mensajes propios.
 *
 * @param message [in] Mensaje a procesar.
 */
void CarConfigurator::processSelfMessage(omnetpp::cMessage *message) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "CarConfigurator::processSelfMessage" << std::endl;

    if (message == locationUpdateTimer)
        processLocationUpdateTimer();
}

/*
 * Actualización de la ubicación.
 */

/*!
 * @brief Programar el temporizador de actualización de la ubicación.
 */
void CarConfigurator::scheduleLocationUpdateTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "CarConfigurator::scheduleLocationUpdateTimer" << std::endl;

    scheduleAt(omnetpp::simTime() + locationUpdateInterval,
            locationUpdateTimer);
}

/*!
 * @brief Procesar el temporizador de actualización de la ubicación.
 *
 * Se revisa si la ubicación del vehículo cambió. En ese caso,
 * si se encuentra en un vértice *gateway*,
 * se configura la dirección correspondiente a la subred adyacente
 * como subred secundaria.
 *
 * TODO Implementar con un FSM.
 */
void CarConfigurator::processLocationUpdateTimer() {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    EV_INFO << "CarConfigurator::processLocationUpdateTimer" << std::endl;

    if (mobility->locationChanged()) {
        EV_INFO << "Cambió la ubicación" << std::endl;

        const Graph &graph = mobility->getRoadNetwork()->getGraph();

        Vertex gatewayVertex;
        bool isAtGateway;
        boost::tie(gatewayVertex, isAtGateway) = mobility->isAtGateway();

        // Si el vehículo se encuentra en un gateway, se une a la red secundaria
        if (isAtGateway) {
            GeohashLocation::Direction gatewayType =
                    graph[gatewayVertex].gatewayType;
            GeohashLocation neighbourGeohashRegion;
            mobility->getRoadNetwork()->getGeohashRegion().getNeighbour(
                    gatewayType, neighbourGeohashRegion);

            joinNetwork(neighbourGeohashRegion, NetworkType::SECONDARY);

            // Si el vehículo no se encuentra en un gateway, se sale de la red secundaria
        } else
            leaveNetwork(NetworkType::SECONDARY);
    } else
        EV_INFO << "No cambió la ubicación" << std::endl;

    // Si el vehículo cambió de una región a otra, se intercambian las subredes primaria y secundaria
    if (mobility->regionChanged()) {
        swapNetworks();
    }

    scheduleLocationUpdateTimer();
}

/*
 * Lifecycle.
 */

void CarConfigurator::handleStartOperation(
        inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("CarConfigurator::handleStartOperation");

    inet::Ipv6InterfaceData *ipv6Data =
            networkInterface->findProtocolDataForUpdate<inet::Ipv6InterfaceData>();
    ipv6Data->setAdvSendAdvertisements(false);
    joinNetwork(mobility->getGeohashLocation(), NetworkType::PRIMARY);

    scheduleLocationUpdateTimer();
}

void CarConfigurator::handleStopOperation(inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("CarConfigurator::handleStartOperation");

    cancelAndDelete(locationUpdateTimer);
    leaveNetwork(NetworkType::PRIMARY);
    leaveNetwork(NetworkType::SECONDARY);
}

void CarConfigurator::handleCrashOperation(
        inet::LifecycleOperation *operation) {
    EV_INFO << "******************************************************************************************************************************************************************"
            << std::endl;
    Enter_Method
    ("CarConfigurator::handleStartOperation");

    cancelAndDelete(locationUpdateTimer);
    leaveNetwork(NetworkType::PRIMARY);
    leaveNetwork(NetworkType::SECONDARY);
}
