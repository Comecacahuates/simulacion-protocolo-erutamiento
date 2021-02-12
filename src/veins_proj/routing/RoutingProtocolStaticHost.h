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
#include "veins_proj/routing/RoutingProtocolBase.h"
#include "veins_proj/locationservice/HostsLocationTable.h"
#include "veins_proj/mobility/StaticHostMobility.h"
#include "veins_proj/routing/Routing_m.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "inet/networklayer/contract/INetfilter.h"

namespace veins_proj {

/**
 * TODO - Generated class
 */
class RoutingProtocolStaticHost : public RoutingProtocolBase {

private:
    // Context
    StaticHostMobility *mobility = nullptr;
    HostsLocationTable *hostsLocationTable = nullptr;

    // Self messages
    omnetpp::cMessage *helloHostTimer = nullptr;

protected:
    // Module interface
    virtual int numInitStages() const override { return inet::NUM_INIT_STAGES; }
    virtual void initialize(int stage) override;

    //  Message handling
    virtual void processSelfMessage(omnetpp::cMessage *message) override;

    // Hello host timer
    virtual void scheduleHelloHostTimer();
    virtual void processHelloHostTimer();

    // Hello Car
    virtual void processHelloCar(const inet::Ptr<HelloCar> &helloCar) override;

    // Hello host
    virtual const inet::Ptr<HelloHost> createHelloHost(const inet::Ipv6Address &hostAddress) const;
    virtual void sendHelloHost(const inet::Ptr<HelloHost> &helloHost, const inet::Ipv6Address &destAddress);

    // Routes
    virtual bool addRouteIfNotExists(const inet::Ipv6Address &destAddress);

    // Routing
    virtual inet::INetfilter::IHook::Result routeDatagram(inet::Packet *datagram, const inet::Ipv6Address &destAddress) override;

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

}
