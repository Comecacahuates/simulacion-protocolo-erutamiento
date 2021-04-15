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
        mobility = omnetpp::check_and_cast<VehicleMobility*>(
                host->getSubmodule("mobility"));
        if (!mobility)
            throw omnetpp::cRuntimeError("No mobility module found");
        /*
         * Mensajes propios.
         */
        locationUpdateTimer = new omnetpp::cMessage("LocationUpdateTimer");
        /*
         * FSM de actualización de la ubicación.
         */
        fsm.setName("CarConfigurator::FSM");
    }
}

/*!
 * @brief Manejo de mensajes.
 *
 * @param message [in] Mensaje a procesar.
 */
void CarConfigurator::handleMessageWhenUp(omnetpp::cMessage *message) {
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
    if (message == locationUpdateTimer)
        processLocationUpdateTimer();
}

/*
 * Actualización de la ubicación.
 */

/*!
 * @brief Programar el temporizador de actualización de la ubicación.
 *
 * @param start [in] Indica si se va a programar el temporizador
 * a la hora de inicio.
 */
void CarConfigurator::scheduleLocationUpdateTimer(bool start) {
    if (start && omnetpp::simTime() < startTime)
        scheduleAt(startTime, locationUpdateTimer);
    else
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
 * Se calcula la ubicación del vehícula y se compara con la anterior
 * para saber si el vehículo se movió y si se encuentra
 * en la misma región Geohash.
 * Después, se usan estos datos para realizar la transición del FSM
 * y realizar la configuración correspondiente.
 */
void CarConfigurator::processLocationUpdateTimer() {
    /*
     * Se guarda la ubicación previa y se calcula la nueva ubicación
     * para determinar qué transiciones debe realizar el FSM.
     */
    GeohashLocation geohashLocation = mobility->getGeohashLocation();
    const RoadNetwork *roadNetwork = mobility->getRoadNetwork();
    mobility->updateLocation();
    const GeohashLocation &newGeohashLocation = mobility->getGeohashLocation();
    const RoadNetwork *newRoadNetwork = mobility->getRoadNetwork();
    GeohashLocation::Adjacency gatewayRegionAdjacency =
            mobility->getGatewayRegionAdjacency();
    /*
     * Si la ubicación cambió, se hace ningún cambio en la
     * configuración de la interfaz.
     */
    if (geohashLocation != newGeohashLocation) {
        /*
         * Si la ubicación cambió, se hace una transición del FSM.
         */
        FSM_Switch(fsm)
        {
            case FSM_Exit(INIT): {
                /*
                 * Si el vehículo no está en una región *gateway*,
                 * se hace una transición al estado NO_GATEWAY.
                 */
                if (gatewayRegionAdjacency == GeohashLocation::Adjacency::NONE)
                    FSM_Goto(fsm, NO_GATEWAY);
                /*
                 * Si el vehículo está en una región *gateway*,
                 * se hace una transición al estado GATEWAY.
                 */
                else
                    FSM_Goto(fsm, GATEWAY);
                break;
            }
            case FSM_Enter(NO_GATEWAY): {
                /*
                 * Cuando se entra al estado NO_GATEWAY,
                 * se sale de la subred secundaria.
                 */
                ASSERT(
                        gatewayRegionAdjacency
                                == GeohashLocation::Adjacency::NONE);
                leaveNetwork(NetworkType::SECONDARY);
                break;
            }
            case FSM_Exit(NO_GATEWAY): {
                /*
                 * Si el vehículo no está en una región *gateway*,
                 * se hace una transición al estado NO_GATEWAY.
                 */
                if (gatewayRegionAdjacency == GeohashLocation::Adjacency::NONE)
                    FSM_Goto(fsm, NO_GATEWAY);
                /*
                 * Si el vehículo está en una región *gateway*,
                 * se hace una transición al estado GATEWAY.
                 */
                else
                    FSM_Goto(fsm, GATEWAY);
                break;
            }
            case FSM_Enter(GATEWAY): {
                /*
                 * Cuando se entra al estado GATEWAY,
                 * se une a la subred adyacente como subred secundaria.
                 */
                ASSERT(
                        gatewayRegionAdjacency
                                != GeohashLocation::Adjacency::NONE);
                GeohashLocation geohashRegion(
                        geohashLocation.getGeohash().substr(0, 6));
                GeohashLocation adjacentGeohashRegion =
                        geohashRegion.getAdjacentGeohashRegion(
                                gatewayRegionAdjacency);
                joinNetwork(adjacentGeohashRegion, NetworkType::SECONDARY);
                break;
            }
            case FSM_Exit(GATEWAY): {
                /*
                 * Si el vehículo cambió a otra región Geohash,
                 * se intercambia la subred primaria con la secundaria
                 * y se hace una transición al estado GATEWAY.
                 */
                if (roadNetwork != newRoadNetwork) {
                    ASSERT(
                            gatewayRegionAdjacency
                                    != GeohashLocation::Adjacency::NONE);
                    swapNetworks();
                    FSM_Goto(fsm, GATEWAY);
                    /*
                     * Si el vehículo no está en una región *gateway*,
                     * se hace una transición al estado NO_GATEWAY.
                     */
                } else if (gatewayRegionAdjacency
                        == GeohashLocation::Adjacency::NONE)
                    FSM_Goto(fsm, NO_GATEWAY);
                /*
                 * Si el vehículo está en una región *gateway*,
                 * se hace una transición al estado GATEWAY.
                 */
                else
                    FSM_Goto(fsm, GATEWAY);
                break;
            }
        }
    }
    scheduleLocationUpdateTimer();
}

/*
 * Lifecycle.
 */

void CarConfigurator::handleStartOperation(
        inet::LifecycleOperation *operation) {
    inet::Ipv6InterfaceData *ipv6Data =
            networkInterface->findProtocolDataForUpdate<inet::Ipv6InterfaceData>();
    ipv6Data->setAdvSendAdvertisements(false);
    joinNetwork(mobility->getGeohashLocation(), NetworkType::PRIMARY);

    scheduleLocationUpdateTimer(true);
}

void CarConfigurator::handleStopOperation(inet::LifecycleOperation *operation) {
    cancelAndDelete(locationUpdateTimer);
    leaveNetwork(NetworkType::PRIMARY);
    leaveNetwork(NetworkType::SECONDARY);
}

void CarConfigurator::handleCrashOperation(
        inet::LifecycleOperation *operation) {
    cancelAndDelete(locationUpdateTimer);
    leaveNetwork(NetworkType::PRIMARY);
    leaveNetwork(NetworkType::SECONDARY);
}
