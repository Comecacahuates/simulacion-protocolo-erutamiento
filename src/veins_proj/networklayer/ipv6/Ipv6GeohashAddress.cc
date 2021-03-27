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
 * @file Ipv6GeohashAddress.cc
 * @author Adrián Juárez Monroy
 */

#include "veins_proj/networklayer/ipv6/Ipv6GeohashAddress.h"

using namespace veins_proj;

/*!
 * @brief Construir una dirección IPv6 Geohash *unicast*.
 *
 * @param geohashLocation [in] Región Geohash.
 * @param interfaceToken  [in] Token de la interfaz.
 * @return Dirección IPv6 Geohash *unicast*.
 */
inet::Ipv6Address Ipv6GeohashAddress::ipv6UnicastAddress(
        const GeohashLocation &geohashLocation,
        const inet::InterfaceToken &interfaceToken) {
    uint32_t d[4] = { 0 };

    uint64_t geohashBits = geohashLocation.getBits();

    d[0] = 0xFEC00000;
    d[1] = (geohashBits >> 34) & 0x3FFFFFFF;
    d[2] = interfaceToken.normal();
    d[3] = interfaceToken.low();

    return inet::Ipv6Address(d[0], d[1], d[2], d[3]);
}

/*!
 * @brief Construit dirección IPv6 Geohash *multicast*.
 *
 * @param geohashLocation [in] Región Geohash.
 * @return Dirección IPv6 Geohash *multicast*.
 */
inet::Ipv6Address Ipv6GeohashAddress::ipv6MulticastAddress(
        const GeohashLocation &geohashLocation) {
    uint32_t d[4] = { 0 };

    uint64_t geohashBits = geohashLocation.getBits();

    d[0] = 0xFF020000;
    d[3] = (geohashBits >> 34) & 0x3FFFFFFF;

    return inet::Ipv6Address(d[0], d[1], d[2], d[3]);
}

/*!
 * @brief Construir prefijo de direcciones IPv6 Geohash *unicast*.
 *
 * @param geohashLocation [in] Región Geohash.
 * @return Prefijo de direcciones Geohash *unicast*.
 */
inet::Ipv6Address Ipv6GeohashAddress::ipv6UnicastAddressPrefix(
        const GeohashLocation &geohashLocation) {
    uint32_t d[4] = { 0 };

    uint64_t geohashBits = geohashLocation.getBits();

    d[0] = 0xFEC00000;
    d[1] = (geohashBits >> 34) & 0x3FFFFFFF;

    return inet::Ipv6Address(d[0], d[1], d[2], d[3]);
}

/*!
 * @brief Verificar si una dirección IPv6
 * es dirección Ipv6 Geohash *unicast* de una región.
 *
 * @param address         [in] Dirección IPv6.
 * @param geohashLocation [in] Región Geohash.
 * @return `true` si es una dirección IPv6 Geohash *unicast* de la región.
 */
bool Ipv6GeohashAddress::isIpv6UnicastGeohashAddress(
        const inet::Ipv6Address &address,
        const GeohashLocation &geohashLocation) {
    const uint32_t *d = address.words();
    uint64_t addressBits = ((uint64_t) d[1] << 34) & 0xFFFFFFFC00000000;

    return addressBits == (geohashLocation.getBits() & 0xFFFFFFFC00000000);
}

/*!
 * @brief Verificar si una dirección IPv6
 * es dirección IPv6 Geohash *multicast* de una región.
 *
 * @param address [in] Dirección IPv6.
 * @param geohashLocation [in] Región Geohash.
 * @return `true` si es una dirección IPv6 Geohash *multicast* de la región.
 */
bool Ipv6GeohashAddress::isIpv6MulticastGeohashAddress(
        const inet::Ipv6Address &address,
        const GeohashLocation &geohashLocation) {
    const uint32_t *d = address.words();
    uint64_t addressBits = ((uint64_t) d[3] << 34) & 0xFFFFFFFC00000000;

    return addressBits == (geohashLocation.getBits() & 0xFFFFFFFC00000000);
}
