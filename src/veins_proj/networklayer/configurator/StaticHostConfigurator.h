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
 * @file StaticHostConfigurator.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <omnetpp.h>
#include "veins_proj/geohash/GeohashLocation.h"
#include "inet/common/INETDefs.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/lifecycle/OperationalBase.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "inet/networklayer/ipv6/Ipv6RoutingTable.h"
#include "inet/networklayer/common/InterfaceToken.h"
#include "inet/networklayer/common/NetworkInterface.h"
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "inet/networklayer/ipv6/Ipv6InterfaceData.h"
#include "veins_proj/veins_proj.h"
#include "veins_proj/mobility/StaticHostMobility.h"
#include "veins_proj/networklayer/configurator/AddressCache.h"
#include "veins_proj/locationservice/HostsLocationTable.h"
#include <vector>
#include <algorithm>

#define PRIMARY_NETWORK   0
#define SECONDARY_NETWORK 1

namespace veins_proj {

/*!
 * @brief Módulo que implementa la configuración de la interfaz de los _hosts_.
 */
class StaticHostConfigurator: public inet::OperationalBase {

protected:
    // Parameter
    std::string interface; // TODO Cambiar al módulo ConfiguratorBase.

    /*
     * Contexto.
     */
    omnetpp::cModule *host;  // TODO Cambiar al módulo ConfiguratorBase.
    inet::IInterfaceTable *interfaceTable;  // TODO Cambiar al módulo ConfiguratorBase.
    inet::NetworkInterface *networkInterface;  // TODO Cambiar al módulo ConfiguratorBase.
    StaticHostMobility *mobility;
    AddressCache *addressCache;
    HostsLocationTable *hostsLocationTable;

    // Internal
    GeohashLocation geohashRegions[2];  // TODO Cambiar al módulo ConfiguratorBase.

public:
    const GeohashLocation &getGeohashRegion(const int &networkType) const { return geohashRegions[networkType]; }

protected:
    // Module interface
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(omnetpp::cMessage *message) override {}

protected:
    // lifecylce
    virtual void handleStartOperation(inet::LifecycleOperation *operation) override;
    virtual void handleStopOperation(inet::LifecycleOperation *operation) override {}
    virtual void handleCrashOperation(inet::LifecycleOperation *operation) override {}
    virtual bool isInitializeStage(int stage) override { return stage == inet::INITSTAGE_NETWORK_CONFIGURATION; }
    virtual bool isModuleStartStage(int stage) override { return stage == inet::ModuleStartOperation::STAGE_NETWORK_LAYER; }
    virtual bool isModuleStopStage(int stage) override { return stage == inet::ModuleStopOperation::STAGE_NETWORK_LAYER; }

    virtual void joinNetwork(const GeohashLocation &geohashRegion);
    virtual void leaveNetwork();

    virtual void initInterface();
    virtual void showAddresses() const;
};

} // namespace veins_proj
