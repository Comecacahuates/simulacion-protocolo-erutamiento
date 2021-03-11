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
 * @file ConfiguratorBase.cc
 * @author Adrián Juárez Monroy
 */

#include "veins_proj/networklayer/configurator/ConfiguratorBase.h"
#include "inet/common/InitStages.h"
#include "inet/networklayer/common/L3AddressResolver.h"
#include "inet/networklayer/ipv6/Ipv6InterfaceData.h"
#include "boost/swap.hpp"
#include "veins_proj/networklayer/ipv6/Ipv6GeohashAddress.h"

using namespace veins_proj;

Register_Abstract_Class(ConfiguratorBase);

/*
 * Interfaz del módulo.
 */

/*!
 * @brief Inicialización.
 *
 * @param stage [in] Etapa de inicialización.
 */
void ConfiguratorBase::initialize(int stage) {
    OperationalBase::initialize(stage);

    /*
     * Inicialización local.
     */
    if (stage == inet::INITSTAGE_LOCAL) {
        /*
         * Parámetros de configuración.
         */
        interface = par("interface").stdstringValue();

        /*
         * Contexto.
         */
        host = inet::getContainingNode(this);
        interfaceTable = inet::L3AddressResolver().interfaceTableOf(host);
        if (!interfaceTable)
            throw omnetpp::cRuntimeError("No interface table found");

        /*
         * Inicialización de las interfaces.
         */
    } else if (stage == inet::INITSTAGE_NETWORK_INTERFACE_CONFIGURATION) {
        /*
         * Contexto.
         */
        networkInterface = interfaceTable->findInterfaceByName(
                interface.c_str());
        if (!networkInterface)
            throw omnetpp::cRuntimeError("No such interface '%s'",
                    interface.c_str());
        if (networkInterface->isLoopback())
            throw omnetpp::cRuntimeError("Interface %s is loopback",
                    interface.c_str());
    }
}

/*
 * Configuración de la interfaz.
 */

/*!
 * @brief Imprimir las direcciones.
 */
void ConfiguratorBase::showAddresses() const {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("StaticHostConfigurator::showAddresses");

    const inet::Ipv6InterfaceData *ipv6Data =
            networkInterface->findProtocolData<inet::Ipv6InterfaceData>();

    EV_INFO << "Unicast addresses" << std::endl;

    for (int i = 0; i < ipv6Data->getNumAddresses(); i++)
        EV_INFO << ipv6Data->getAddress(i).str() << std::endl;

    EV_INFO << "Multicast addresses" << std::endl;

    for (const inet::Ipv6Address &multicastAddress : ipv6Data->getJoinedMulticastGroups())
        EV_INFO << multicastAddress.str() << std::endl;
}

/*!
 * @brief Unirse a la subred correspondiente a la región Geohash.
 *
 * Se utiliza cuando un vehículo o *host* se une a la red por primera vez,
 * o cuando un vehículo entra a una región *gateway*.
 *
 * @param geohashRegion [in] Región Geohash a cuya red se une
 * @param networkType [in] Tipo de red a la que se va a unir.
 */
void ConfiguratorBase::joinNetwork(const GeohashLocation &geohashRegion,
        const NetworkType networkType) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("StaticHostConfigurator::joinNetwork");

    /*
     * Si ya forma parte de la región Geohash solicitada,
     * no se reconfigura la interfaz.
     */
    if (geohashRegions[networkType] == geohashRegion)
        return;

    /*
     * Se crean las direcciones *unicast* y *multicast*
     * y se asigna a la interfaz.
     */
    inet::Ipv6InterfaceData *ipv6Data =
            networkInterface->findProtocolDataForUpdate<inet::Ipv6InterfaceData>();
    unicastAddresses[networkType] = Ipv6GeohashAddress::ipv6UnicastAddress(
            geohashRegion, networkInterface->getInterfaceToken());
    multicastAddresses[networkType] = Ipv6GeohashAddress::ipv6MulticastAddress(
            geohashRegion);
    geohashRegions[networkType] = geohashRegion;
    ipv6Data->assignAddress(unicastAddresses[networkType], false, SIMTIME_ZERO,
    SIMTIME_ZERO);
    ipv6Data->assignAddress(multicastAddresses[networkType], false,
    SIMTIME_ZERO,
    SIMTIME_ZERO);
    ipv6Data->joinMulticastGroup(multicastAddresses[networkType]);
}

/*!
 * @brief Salirse de una subred.
 *
 * Se utiliza cuando un vehículo sale de la región *gateway*.
 *
 * @param networkType [in] Tipo de subred de la que se va a salir.
 */
void ConfiguratorBase::leaveNetwork(const NetworkType networkType) {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("StaticHostConfigurator::leaveNetwork");

    /*
     * Se reconfigura la interfaz para desasignar las direcciones
     * *unicast* y *multicast*.
     */
    inet::Ipv6InterfaceData *ipv6Data =
            networkInterface->findProtocolDataForUpdate<inet::Ipv6InterfaceData>();
    if (ipv6Data->hasAddress(unicastAddresses[networkType]))
        ipv6Data->removeAddress(unicastAddresses[networkType]);
    unicastAddresses[networkType] = inet::Ipv6Address::UNSPECIFIED_ADDRESS;
    if (ipv6Data->hasAddress(multicastAddresses[networkType]))
        ipv6Data->removeAddress(multicastAddresses[networkType]);
    if (ipv6Data->isMemberOfMulticastGroup(multicastAddresses[networkType]))
        ipv6Data->leaveMulticastGroup(multicastAddresses[networkType]);
    multicastAddresses[networkType] = inet::Ipv6Address::UNSPECIFIED_ADDRESS;
    geohashRegions[networkType].setNull();
}

/*!
 * @brief Intercambia la subred primaria y la subred secundaria.
 *
 * Se utiliza cuando un vehículo cambia de una región Geohash a otra.
 */
void ConfiguratorBase::swapNetworks() {
    EV_DEBUG << "******************************************************************************************************************************************************************"
             << std::endl;
    Enter_Method
    ("StaticHostConfigurator::swapNetworks");

    boost::swap(unicastAddresses[NetworkType::PRIMARY], unicastAddresses[NetworkType::SECONDARY]);
    boost::swap(multicastAddresses[NetworkType::PRIMARY], multicastAddresses[NetworkType::SECONDARY]);
    boost::swap(geohashRegions[NetworkType::PRIMARY], geohashRegions[NetworkType::SECONDARY]);
}
