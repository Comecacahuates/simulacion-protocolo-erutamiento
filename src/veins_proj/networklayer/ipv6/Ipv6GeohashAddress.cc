/*
 * Ipv6GeohashMulticastAddress.cc
 *
 *  Created on: Jun 16, 2020
 *      Author: adrian
 */

#include "veins_proj/networklayer/ipv6/Ipv6GeohashAddress.h"

using namespace veins_proj;


inet::Ipv6Address Ipv6GeohashAddress::ipv6UnicastAddress(const GeohashLocation &geohashLocation, const inet::InterfaceToken &interfaceToken) {
    uint32_t d[4] = { 0 };

    uint64_t geohashBits = geohashLocation.getBits();

    d[0] = 0xFEC00000;
    d[1] = (geohashBits >> 34) & 0x3FFFFFFF;
    d[2] = interfaceToken.normal();
    d[3] = interfaceToken.low();

    return inet::Ipv6Address(d[0], d[1], d[2], d[3]);
}


inet::Ipv6Address Ipv6GeohashAddress::ipv6MulticastAddress(const GeohashLocation &geohashLocation) {
    uint32_t d[4] = { 0 };

    uint64_t geohashBits = geohashLocation.getBits();

    d[0] = 0xFF020000;
    d[3] = (geohashBits >> 34) & 0x3FFFFFFF;

    return inet::Ipv6Address(d[0], d[1], d[2], d[3]);
}


inet::Ipv6Address Ipv6GeohashAddress::ipv6UnicastAddressPrefix(const GeohashLocation &geohashLocation) {
    uint32_t d[4] = { 0 };

    uint64_t geohashBits = geohashLocation.getBits();

    d[0] = 0xFEC00000;
    d[1] = (geohashBits >> 34) & 0x3FFFFFFF;

    return inet::Ipv6Address(d[0], d[1], d[2], d[3]);
}


bool Ipv6GeohashAddress::isIpv6UnicastGeohashAddress(const inet::Ipv6Address &address, const GeohashLocation &geohashLocation) {
    const uint32_t *d = address.words();
    uint64_t addressBits = ((uint64_t)d[1] << 34) & 0xFFFFFFFC00000000;

    return addressBits == (geohashLocation.getBits() & 0xFFFFFFFC00000000);
}


bool Ipv6GeohashAddress::isIpv6MulticastGeohashAddress(const inet::Ipv6Address &address, const GeohashLocation &geohashLocation) {
    const uint32_t *d = address.words();
    uint64_t addressBits = ((uint64_t)d[3] << 34) & 0xFFFFFFFC00000000;

    return addressBits == (geohashLocation.getBits() & 0xFFFFFFFC00000000);
}
