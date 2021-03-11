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
 * @file StaticHostConfigurator.cc
 * @author Adrián Juárez Monroy
 */

#include "veins_proj/networklayer/configurator/StaticHostConfigurator.h"
#include "inet/common/ModuleAccess.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/lifecycle/NodeStatus.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/common/geometry/common/Coord.h"
#include "veins/base/utils/Coord.h"
#include <utility>
#include "veins_proj/networklayer/ipv6/Ipv6GeohashAddress.h"
#include "veins_proj/roadnetwork/RoadNetwork.h"

using namespace veins_proj;

Define_Module(StaticHostConfigurator);

/*
 * Interfaz del módulo.
 */

/*!
 * @brief Inicialización.
 *
 * @param stage [in] Etapa de inicialización.
 */
void StaticHostConfigurator::initialize(int stage) {
    ConfiguratorBase::initialize(stage);

    /*
     * Etapa de inicialización local.
     */
    if (stage == inet::INITSTAGE_LOCAL) {
        /*
         * Contexto.
         */
        mobility = omnetpp::check_and_cast<StaticHostMobility*>(
                host->getSubmodule("mobility"));
        if (!mobility)
            throw omnetpp::cRuntimeError("No mobility module found");
        hostsLocationTable = omnetpp::check_and_cast<HostsLocationTable*>(
                getModuleByPath(par("hostsLocationTableModule")));
        if (!hostsLocationTable)
            throw omnetpp::cRuntimeError(
                    "No hosts location table module found");
    }
}

/*
 * Lifecycle.
 */

void StaticHostConfigurator::handleStartOperation(
        inet::LifecycleOperation *operation) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("StaticHostConfigurator::handleStartOperation");

    inet::Ipv6InterfaceData *ipv6Data =
            networkInterface->findProtocolDataForUpdate<inet::Ipv6InterfaceData>();
    ipv6Data->setAdvSendAdvertisements(false);
    joinNetwork(mobility->getGeohashLocation(), NetworkType::PRIMARY);
}

void StaticHostConfigurator::handleStopOperation(
        inet::LifecycleOperation *operation) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("StaticHostConfigurator::handleStopOperation");

    leaveNetwork(NetworkType::PRIMARY);
}

void StaticHostConfigurator::handleCrashOperation(
        inet::LifecycleOperation *operation) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("StaticHostConfigurator::handleCrashOperation");

    leaveNetwork(NetworkType::PRIMARY);
}
