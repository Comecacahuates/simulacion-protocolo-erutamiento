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
#include "inet/common/TlvOptions_m.h"
#include "inet/networklayer/common/NetworkInterface.h"
#include "inet/networklayer/common/InterfaceTable.h"
#include "inet/networklayer/common/L3Tools.h"
#include "inet/networklayer/contract/INetfilter.h"
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "inet/networklayer/ipv6/Ipv6ExtensionHeaders_m.h"
#include "inet/networklayer/ipv6/Ipv6Header_m.h"
#include "inet/networklayer/ipv6/Ipv6RoutingTable.h"
#include "inet/routing/base/RoutingProtocolBase.h"
#include "veins_proj/routing/Routing_m.h"
#include "veins_proj/roadnetwork/RoadNetworkDatabase.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include <string>
#include <map>
#include <utility>

#define ROUTING_PROTOCOL_UDP_PORT                       4096
#define IPV6TLVOPTION_TLV_DEST_GEOHASH_LOCATION         75
#define IPV6TLVOPTION_TLV_DEST_ON_ROAD_NETWORK_LOCATION 76
#define IPV6TLVOPTION_TLV_ROUTING                       77

namespace veins_proj {

class RoutingProtocolBase: public inet::RoutingProtocolBase,
		public omnetpp::cListener,
		public inet::NetfilterBase::HookBase {

protected:
	struct NeighbouringCarEntry {
		omnetpp::simtime_t lastUpdateTime;
		GeohashLocation geohashLocation;
		double speed;
		double direction;
		LocationOnRoadNetwork locationOnRoadNetwork;
	};

protected:
	// Routing parameters
	omnetpp::simtime_t helloCarInterval;
	omnetpp::simtime_t neighbouringCarValidityTime;
	omnetpp::simtime_t helloHostInterval;
	omnetpp::simtime_t neighbouringHostValidityTime;
	omnetpp::simtime_t pingInterval;
	omnetpp::simtime_t pongTimeout;
	omnetpp::simtime_t activeEdgeValidityTime;
	omnetpp::simtime_t inactiveEdgeValidityTime;
	omnetpp::simtime_t routeValidityTime;
	double vertexProximityRadius;

	// Context
	omnetpp::cModule *host = nullptr;
	inet::IInterfaceTable *interfaceTable = nullptr;
	inet::Ipv6RoutingTable *routingTable = nullptr;
	inet::INetfilter *networkProtocol = nullptr;
	AddressCache *addressCache = nullptr;
	inet::NetworkInterface *networkInterface = nullptr;
	RoadNetworkDatabase *roadNetworkDatabase = nullptr;

	// Internal

	// Neighbouring cars
	typedef std::pair<inet::Ipv6Address, NeighbouringCarEntry> NeighbouringCar;
	typedef std::map<inet::Ipv6Address, NeighbouringCarEntry> NeighbouringCarsMap;
	typedef NeighbouringCarsMap::iterator NeighbouringCarsIterator;
	typedef NeighbouringCarsMap::const_iterator NeighbouringCarsConstIterator;
	NeighbouringCarsMap neighbouringCars;

	// Self messages
	omnetpp::cMessage *purgeNeighbouringCarsTimer = nullptr;

protected:
	// Module interface
	virtual int numInitStages() const override {
		return inet::NUM_INIT_STAGES;
	}
	virtual void initialize(int stage) override;
	virtual void handleMessageWhenUp(omnetpp::cMessage *message) override;

	//  Message handling
	virtual void processSelfMessage(omnetpp::cMessage *message);
	virtual void processMessage(omnetpp::cMessage *message);

	// Purge neighbouring cars timer
	virtual void schedulePurgeNeighbouringCarsTimer();
	virtual void processPurgeNeighbouringCarsTimer();

	// UDP packets
	virtual void sendUdpPacket(inet::Packet *packet);
	virtual void processUdpPacket(inet::Packet *packet);

	// Acks
	virtual const inet::Ptr<Ack> createAck(
			const inet::Ipv6Address &address) const;
	virtual void sendAck(const inet::Ptr<Ack> &ack,
			const inet::Ipv6Address &destinationAddress);
	virtual void processAck(const inet::Ptr<Ack> &ack);

	// Hello Car
	virtual void processHelloCar(const inet::Ptr<HelloCar> &helloCar) {
	}

	// Hello host
	virtual void processHelloHost(const inet::Ptr<HelloHost> &helloHost) {
	}

	// Ping
	virtual void processPing(const inet::Ptr<Ping> &ping) {
	}

	// Pong
	virtual void processPong(const inet::Ptr<Pong> &pong) {
	}

	// Neighbouring cars
	void showNeighbouringCars() const;
	virtual void removeOldNeighbouringCars(omnetpp::simtime_t time);
	omnetpp::simtime_t getOldestNeighbouringCarTime() const;
	omnetpp::simtime_t getNextNeighbouringCarExpirationTime() const;
	void purgeNeighbouringCars();
	inet::Ipv6Address getClosestNeighbouringCarAddress(
			const GeohashLocation &geohashLocation) const;

	// Routes
	virtual void addRoute(const inet::Ipv6Address &destPrefix,
			const short prefixLength, const inet::Ipv6Address &nextHop,
			int metric, omnetpp::simtime_t expiryTime);
	virtual void purgeNextHopRoutes(const inet::Ipv6Address &nextHopAddress);
	virtual void removeOldRoutes(omnetpp::simtime_t time);
	virtual void showRoutes() const;

	// Routing
	virtual inet::INetfilter::IHook::Result routeDatagram(
			inet::Packet *datagram, const inet::Ipv6Address &destAddress) = 0;

	// TLV options
	void setTlvOption(inet::Packet *datagram,
			inet::TlvOptionBase *tlvOption) const;
	template<class T> const T* findTlvOption(inet::Packet *datagram) const {
		EV_INFO
						<< "******************************************************************************************************************************************************************"
						<< std::endl;
		EV_INFO << "RoutingProtocolBase::findTlvOption" << std::endl;

		const T *tlvOption = nullptr;
		inet::Ptr<const inet::Ipv6Header> ipv6Header = inet::dynamicPtrCast<
				const inet::Ipv6Header>(
				inet::getNetworkProtocolHeader(datagram));
		const inet::Ipv6ExtensionHeader *extensionHeader =
				ipv6Header->findExtensionHeaderByType(
						inet::IpProtocolId::IP_PROT_IPv6EXT_HOP);
		const inet::Ipv6HopByHopOptionsHeader *optionsHeader =
				omnetpp::check_and_cast_nullable<
						const inet::Ipv6HopByHopOptionsHeader*>(
						extensionHeader);

		if (optionsHeader) {
			const inet::TlvOptions &tlvOptions = optionsHeader->getTlvOptions();

			int i = 0;
			while (i < tlvOptions.getTlvOptionArraySize()) {
				tlvOption =
						dynamic_cast<const T*>(tlvOptions.getTlvOption(i++));

				if (tlvOption != nullptr)
					break;
			}
		}

		return tlvOption;
	}
	template<class T> T* findTlvOptionForUpdate(inet::Packet *datagram) {
		EV_INFO
						<< "******************************************************************************************************************************************************************"
						<< std::endl;
		EV_INFO << "RoutingProtocolBase::findTlvOptionForUpdate" << std::endl;

		T *tlvOption = nullptr;
		inet::Ptr<inet::Ipv6Header> ipv6Header = inet::constPtrCast<
				inet::Ipv6Header>(
				inet::dynamicPtrCast<const inet::Ipv6Header>(
						inet::getNetworkProtocolHeader(datagram)));
		inet::Ipv6ExtensionHeader *extensionHeader =
				ipv6Header->findExtensionHeaderByTypeForUpdate(
						inet::IpProtocolId::IP_PROT_IPv6EXT_HOP);
		inet::Ipv6HopByHopOptionsHeader *optionsHeader =
				omnetpp::check_and_cast_nullable<
						inet::Ipv6HopByHopOptionsHeader*>(extensionHeader);

		if (optionsHeader) {
			inet::TlvOptions &tlvOptions =
					optionsHeader->getTlvOptionsForUpdate();

			int i = 0;
			while (i < tlvOptions.getTlvOptionArraySize()) {
				tlvOption = dynamic_cast<T*>(tlvOptions.getTlvOptionForUpdate(
						i++));

				if (tlvOption != nullptr)
					break;
			}
		}

		return tlvOption;
	}

	// Tlv destination geohash location option
	TlvDestGeohashLocationOption* createTlvDestGeohashLocationOption(
			uint64_t geohashLocationBits) const;
	int computeTlvOptionLength(TlvDestGeohashLocationOption *tlvOption) const;

	// TLV destination on road network location option
	TlvDestLocationOnRoadNetworkOption* createTlvDestLocationOnRoadNetworkOption(
			Vertex vertexA, Vertex vertexB, double distanceToVertexA) const;
	void setTlvDestLocationOnRoadNetworkOption(inet::Packet *datagram,
			const GeohashLocation &destGeohashLocation) const;
	int computeTlvOptionLength(
			TlvDestLocationOnRoadNetworkOption *tlvOption) const;

	// Tlv routing option
	TlvRoutingOption* createTlvRoutingOption() const;
	void setTlvRoutingOption(inet::Packet *datagram) const;
	int computeTlvOptionLength(TlvRoutingOption *tlvOption) const;

	// Netfilter
	virtual inet::INetfilter::IHook::Result datagramPreRoutingHook(
			inet::Packet *datagram) override {
		return inet::INetfilter::IHook::ACCEPT;
	}
	virtual inet::INetfilter::IHook::Result datagramForwardHook(
			inet::Packet *datagram) override {
		return inet::INetfilter::IHook::ACCEPT;
	}
	virtual inet::INetfilter::IHook::Result datagramPostRoutingHook(
			inet::Packet *datagram) override {
		return inet::INetfilter::IHook::ACCEPT;
	}
	virtual inet::INetfilter::IHook::Result datagramLocalInHook(
			inet::Packet *datagram) override {
		return inet::INetfilter::IHook::ACCEPT;
	}
	virtual inet::INetfilter::IHook::Result datagramLocalOutHook(
			inet::Packet *datagram) override {
		return inet::INetfilter::IHook::ACCEPT;
	}

	// Lifecycle
	virtual void handleStartOperation(inet::LifecycleOperation *operation)
			override;
	virtual void handleStopOperation(inet::LifecycleOperation *operation)
			override;
	virtual void handleCrashOperation(inet::LifecycleOperation *operation)
			override;

	// Notification
	virtual void receiveSignal(omnetpp::cComponent *source,
			omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
			cObject *details) override = 0;
};

}
