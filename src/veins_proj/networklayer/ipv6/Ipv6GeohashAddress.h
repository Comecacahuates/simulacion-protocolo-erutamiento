/*
 * Ipv6GeohashMulticastAddress.h
 *
 *  Created on: Jun 16, 2020
 *      Author: adrian
 */

#pragma once

#include "veins_proj/veins_proj.h"
#include <omnetpp.h>
#include "veins_proj/geohash/GeohashLocation.h"
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "inet/networklayer/common/InterfaceToken.h"
#include <string>

namespace veins_proj {


class Ipv6GeohashAddress {

public:
    static inet::Ipv6Address ipv6UnicastAddress(const GeohashLocation &geohashLocation, const inet::InterfaceToken &interfaceToken);
    static inet::Ipv6Address ipv6MulticastAddress(const GeohashLocation &geohashLocation);
    static inet::Ipv6Address ipv6UnicastAddressPrefix(const GeohashLocation &geohashLocation);
    static bool isIpv6UnicastGeohashAddress(const inet::Ipv6Address &address, const GeohashLocation &geohashLocation);
    static bool isIpv6MulticastGeohashAddress(const inet::Ipv6Address &address, const GeohashLocation &geohashLocation);
};


} // namespace veins_proj
