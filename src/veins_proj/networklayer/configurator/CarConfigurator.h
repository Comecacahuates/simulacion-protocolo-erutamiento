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
#include "veins_proj/geohash/GeohashLocation.h"
#include "inet/common/INETDefs.h"
#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/lifecycle/OperationalBase.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "inet/networklayer/ipv6/Ipv6RoutingTable.h"
#include "inet/networklayer/common/NetworkInterface.h"
#include "inet/networklayer/common/InterfaceToken.h"
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "inet/networklayer/ipv6/Ipv6InterfaceData.h"
#include "veins_proj/veins_proj.h"
#include "veins_proj/mobility/CarMobility.h"
#include "veins_proj/networklayer/configurator/AddressCache.h"
#include <vector>
#include <algorithm>

#define PRIMARY_NETWORK   0
#define SECONDARY_NETWORK 1

namespace veins_proj {


class CarConfigurator : public inet::OperationalBase {

public:
    enum NetworkType { PRIMARY = 0, SECONDARY = 1 };

protected:
    // Parameters
    std::string interface;
    omnetpp::simtime_t locationUpdateInterval;

    // Context
    omnetpp::cModule *host;  // TODO Cambiar al módulo ConfiguratorBase.
    inet::IInterfaceTable *interfaceTable;  // TODO Cambiar al módulo ConfiguratorBase.
    inet::NetworkInterface *networkInterface;  // TODO Cambiar al módulo ConfiguratorBase.
    CarMobility *mobility;
    AddressCache *addressCache;  // TODO Eliminar.

    // Internal
    GeohashLocation geohashRegions[2];  // TODO Cambiar al módulo ConfiguratorBase.

    // Self messages
    omnetpp::cMessage *locationUpdateTimer;

public:
    const GeohashLocation &getGeohashRegion(const int &networkType) const { return geohashRegions[networkType]; }


protected:
    // Module interface
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;
    virtual void handleMessageWhenUp(omnetpp::cMessage *message) override;

    // Message handling
    void processSelfMessage(omnetpp::cMessage *message);

    // Location update timer
    void processLocationUpdateTimer();
    void scheduleLocationUpdateTimer();

protected:
    // Lifecylce
    virtual void handleStartOperation(inet::LifecycleOperation *operation) override;
    virtual void handleStopOperation(inet::LifecycleOperation *operation) override;
    virtual void handleCrashOperation(inet::LifecycleOperation *operation) override;
    virtual bool isInitializeStage(int stage) override { return stage == inet::INITSTAGE_NETWORK_CONFIGURATION; }
    virtual bool isModuleStartStage(int stage) override { return stage == inet::ModuleStartOperation::STAGE_NETWORK_LAYER; }
    virtual bool isModuleStopStage(int stage) override { return stage == inet::ModuleStopOperation::STAGE_NETWORK_LAYER; }

    virtual void initInterface();
    virtual void joinNetwork(const GeohashLocation &geohashRegion, const int &networkType);
    virtual void leaveNetwork(const int &networkType);
    virtual void swapNetworks();

    virtual void showAddresses() const;
};


} // namespace veins_proj
