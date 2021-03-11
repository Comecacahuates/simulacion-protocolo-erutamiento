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
#include "veins_proj/mobility/StaticHostMobility.h"
#include "veins_proj/networklayer/configurator/ConfiguratorBase.h"
#include "veins_proj/locationservice/HostsLocationTable.h"
#include <vector>
#include <algorithm>

namespace veins_proj {

/*!
 * @brief Módulo que implementa la configuración
 * de la interfaz de los *hosts*.
 */
class StaticHostConfigurator: public ConfiguratorBase {

protected:

    /*
     * Contexto.
     */
    /*!
     * @brief Módulo de movilidad.
     */
    StaticHostMobility *mobility;
    /*!
     * @brief Módulo de la tabla de ubicación de *hosts*.
     */
    HostsLocationTable *hostsLocationTable;

    /*
     * Interfaz del módulo.
     */
    /*!
     * @brief Inicialización.
     *
     * @param stage [in] Etapa de inicialización.
     */
    virtual void initialize(int stage) override;
    /*!
     * @brief Manejo de mensajes.
     *
     * @param message [in] Mensaje a procesar.
     */
    virtual void handleMessageWhenUp(omnetpp::cMessage *message) override {
    }

    /*
     * Lifecycle.
     */
    virtual void handleStartOperation(inet::LifecycleOperation *operation)
            override;
    virtual void handleStopOperation(inet::LifecycleOperation *operation)
            override;
    virtual void handleCrashOperation(inet::LifecycleOperation *operation)
            override;
};

}    // namespace veins_proj
