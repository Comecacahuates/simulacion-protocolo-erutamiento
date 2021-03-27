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
 * @file Ipv6GeohashAddress.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include "veins_proj/veins_proj.h"
#include <omnetpp.h>
#include "veins_proj/geohash/GeohashLocation.h"
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "inet/networklayer/common/InterfaceToken.h"
#include <string>

namespace veins_proj {

/*!
 * @brief Clase que contiene métodos estáticos para construir
 * direcciones IPv6 Geohash.
 */
class Ipv6GeohashAddress {

public:

    /*!
     * @brief Construir una dirección IPv6 Geohash *unicast*.
     *
     * @param geohashLocation [in] Región Geohash.
     * @param interfaceToken  [in] Token de la interfaz.
     * @return Dirección IPv6 Geohash *unicast*.
     */
    static inet::Ipv6Address ipv6UnicastAddress(
            const GeohashLocation &geohashLocation,
            const inet::InterfaceToken &interfaceToken);
    /*!
     * @brief Construit dirección IPv6 Geohash *multicast*.
     *
     * @param geohashLocation [in] Región Geohash.
     * @return Dirección IPv6 Geohash *multicast*.
     */
    static inet::Ipv6Address ipv6MulticastAddress(
            const GeohashLocation &geohashLocation);
    /*!
     * @brief Construir prefijo de direcciones IPv6 Geohash *unicast*.
     *
     * @param geohashLocation [in] Región Geohash.
     * @return Prefijo de direcciones Geohash *unicast*.
     */
    static inet::Ipv6Address ipv6UnicastAddressPrefix(
            const GeohashLocation &geohashLocation);
    /*!
     * @brief Verificar si una dirección IPv6
     * es dirección Ipv6 Geohash *unicast* de una región.
     *
     * @param address         [in] Dirección IPv6.
     * @param geohashLocation [in] Región Geohash.
     * @return `true` si es una dirección IPv6 Geohash *unicast* de la región.
     */
    static bool isIpv6UnicastGeohashAddress(const inet::Ipv6Address &address,
            const GeohashLocation &geohashLocation);
    /*!
     * @brief Verificar si una dirección IPv6
     * es dirección IPv6 Geohash *multicast* de una región.
     *
     * @param address [in] Dirección IPv6.
     * @param geohashLocation [in] Región Geohash.
     * @return `true` si es una dirección IPv6 Geohash *multicast* de la región.
     */
    static bool isIpv6MulticastGeohashAddress(const inet::Ipv6Address &address,
            const GeohashLocation &geohashLocation);
};

}    // namespace veins_proj
