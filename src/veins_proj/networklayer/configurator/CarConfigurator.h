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
 * @file CarConfigurator.h
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
#include "inet/networklayer/common/NetworkInterface.h"
#include "inet/networklayer/common/InterfaceToken.h"
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "inet/networklayer/ipv6/Ipv6InterfaceData.h"
#include "veins_proj/veins_proj.h"
#include "veins_proj/mobility/CarMobility.h"
#include "veins_proj/networklayer/configurator/ConfiguratorBase.h"
#include <vector>
#include <algorithm>

namespace veins_proj {

/*!
 * @brief Módulo que implementa la configuración
 * de la interfaz de los vehículos.
 */
class CarConfigurator: public ConfiguratorBase {

protected:

    /*
     * Parámetros de configuración.
     */
    //! Hora de inicio.
    omnetpp::simtime_t startTime;
    //! Intervalo de actualización de la ubicación.
    omnetpp::simtime_t locationUpdateInterval;

    /*
     * Contexto.
     */
    CarMobility *mobility;

    /*
     * Mensajes propios.
     */
    omnetpp::cMessage *locationUpdateTimer;

    /*
     * FSM de actualización de la ubicación.
     */
    /*!
     * @brief Estados del FSM de actualización de la ubicación.
     */
    enum {
        /*!
         * @brief Estado inicial.
         */
        INIT = 0,
        /*!
         * @brief El vehículo está fuera de cualquier región *gateway*.
         */
        NO_GATEWAY = FSM_Steady(1),
        /*!
         * @brief El vehículo está en una región *gateway*.
         */
        GATEWAY = FSM_Steady(2),
    };
    /*!
     * @brief FSM de actualización de la ubicación.
     */
    omnetpp::cFSM fsm;

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
    virtual void handleMessageWhenUp(omnetpp::cMessage *message) override;

    /*
     * Manejo de mensajes.
     */
    /*!
     * @brief Manejo de mensajes propios.
     *
     * @param message [in] Mensaje a procesar.
     */
    void processSelfMessage(omnetpp::cMessage *message);

    /*
     * Actualización de la ubicación.
     */
    /*!
     * @brief Programar el temporizador de actualización de la ubicación.
     *
     * @param start [in] Indica si se va a programar el temporizador
     * a la hora de inicio.
     */
    void scheduleLocationUpdateTimer(bool start = false);
    /*!
     * @brief Procesar el temporizador de actualización de la ubicación.
     */
    void processLocationUpdateTimer();

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
