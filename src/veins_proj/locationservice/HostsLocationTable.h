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
 * @file HostsLocationTable.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <omnetpp.h>
#include <map>
#include <vector>
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "veins_proj/geohash/GeohashLocation.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"

namespace veins_proj {

/*!
 * @brief Módulo global que contiene las ubicaciones de los *hosts*.
 *
 * Cumple la función de servicio de localización.
 * Cuando un *host* va a enviar un paquete, consulta la
 * ubicación del destino en la tabla de ubicaciónes de *hosts*,
 * y la incluye en la cabecera del datagrama.
 */
class HostsLocationTable: public omnetpp::cSimpleModule {

private:

    //! Diccionario de la tabla de ubicaciones de *hosts*.
    typedef std::map<inet::Ipv6Address, GeohashLocation> HostsLocationMap;
    //! Tabla de ubicaciones de *hosts*.
    HostsLocationMap hostsLocation;

protected:

    /*
     * Interfaz del módulo.
     */
    /*!
     * @brief Inicialización.
     */
    virtual void initialize() override {
    }
    /*!
     * @brief Manejo de mensajes.
     *
     * Este módulo no recibe ningún mensaje.
     *
     * @param message [in] Mensaje a procesar.
     */
    virtual void handleMessage(omnetpp::cMessage *message) override {
    }

public:

    /*!
     * @brief Registrar la ubicación de un *host*.
     *
     * @param address         [in] Dirección IPv6 del *host*.
     * @param geohashLocation [in] Ubicación Geohash del *host*.
     */
    void registerHostLocation(const inet::Ipv6Address &address,
            const GeohashLocation &geohashLocation) {
        hostsLocation[address] = geohashLocation;
    }
    /*!
     * @brief Obtener ubicación de un *host*.
     *
     * @param address [in] Dirección IPv6 del *host*.
     * @return Ubicación Geohash del *host*.
     */
    const GeohashLocation& getHostLocation(const inet::Ipv6Address &address) {
        return hostsLocation[address];
    }
};

}    // namespace veins_proj
