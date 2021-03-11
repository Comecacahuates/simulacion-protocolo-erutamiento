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
 * @file ConfiguratorBase.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include "inet/common/lifecycle/ModuleOperations.h"
#include "inet/common/lifecycle/OperationalBase.h"
#include "inet/networklayer/contract/IInterfaceTable.h"
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "veins_proj/geohash/GeohashLocation.h"

namespace veins_proj {

/*!
 * @brief Módulo que implementa las operaciones de configuración de la interfaz
 * comunes para vehículos y _hosts_.
 *
 * Contiene el registro de las direcciones IPv6 *unicast* y *multicast*
 * primarias y secundarias. Además, se encarga del intercambio de estas
 * cuando se cambia de una subred a otra.
 */
class ConfiguratorBase: public inet::OperationalBase {

protected:

    /*
     * Parámetros de configuración.
     */
    /*!
     * @brief Interfaz que se configura.
     */
    std::string interface;

    /*
     * Contexto.
     */
    /*!
     * @brief Módulo del vehículo o *host* contenedor.
     */
    omnetpp::cModule *host;
    /*!
     * @brief Módulo de la tabla de interfaces.
     */
    inet::IInterfaceTable *interfaceTable;
    /*!
     * @brief Módulo de la interfaz que se configura.
     */
    inet::NetworkInterface *networkInterface;

    /*
     * Direcciones.
     */
    /*!
     * @brief Direcciones *unicast*.
     */
    inet::Ipv6Address unicastAddresses[2];
    /*!
     * @brief Direcciones *multicast*.
     */
    inet::Ipv6Address multicastAddresses[2];
    /*!
     * Regiones Geohash.
     */
    GeohashLocation geohashRegions[2];

public:

    /*!
     * @brief Tipo de red.
     */
    enum NetworkType {
        /*!
         * @brief Subred de la región Geohash en la que se encuentra.
         */
        PRIMARY = 0,
        /*!
         * @brief Subred de la región Geohash adyacente
         * cuando se encuentra en un vértice *gateway*.
         */
        SECONDARY = 1,
    };

protected:

    /*
     * Interfaz del módulo.
     */
    /*!
     * @brief Número de etapas de inicialización.
     *
     * @return Número de etapas de inicialización.
     */
    virtual int numInitStages() const override {
        return inet::NUM_INIT_STAGES;
    }
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
    virtual void handleMessageWhenUp(omnetpp::cMessage *message) override = 0;

public:

    /*
     * Acceso a las direcciones.
     */
    /*!
     * Obtener una de las dos direcciones *unicast*.
     *
     * @param networktype [in] Tipo de dirección que se quiere obtener.
     *
     * @return Dirección asignada.
     */
    const inet::Ipv6Address& getUnicastAddress(
            const NetworkType networktype) const {
        showAddresses();
        return unicastAddresses[networktype];
    }
    /*!
     * Obtener una de las dos direcciones *multicast*.
     *
     * @param networktype [in] Tipo de dirección que se quiere obtener.
     *
     * @return Dirección asignada.
     */
    const inet::Ipv6Address& getMulticastAddress(
            const NetworkType networktype) const {
        showAddresses();
        return multicastAddresses[networktype];
    }

protected:

    /*
     * Configuración de la interfaz.
     */
    /*!
     * @brief Imprimir las direcciones.
     */
    void showAddresses() const;
    /*!
     * @brief Unirse a la subred correspondiente a la región Geohash.
     *
     * Se utiliza cuando un vehículo o *host* se une a la red por primera vez,
     * o cuando un vehículo entra a una región *gateway*.
     *
     * @param geohashRegion [in] Región Geohash a cuya red se une
     * @param networkType [in] Tipo de red a la que se va a unir.
     */
    void joinNetwork(const GeohashLocation &geohashRegion,
            const NetworkType networkType);
    /*!
     * @brief Salirse de una subred.
     *
     * Se utiliza cuando un vehículo sale de la región *gateway*.
     *
     * @param networkType [in] Tipo de subred de la que se va a salir.
     */
    void leaveNetwork(const NetworkType networkType);
    /*!
     * @brief Intercambia la subred primaria y la subred secundaria.
     *
     * Se utiliza cuando un vehículo cambia de una región Geohash a otra.
     */
    void swapNetworks();

    /*
     * Lifecycle.
     */
    virtual void handleStartOperation(inet::LifecycleOperation *operation)
            override = 0;
    virtual void handleStopOperation(inet::LifecycleOperation *operation)
            override = 0;
    virtual void handleCrashOperation(inet::LifecycleOperation *operation)
            override = 0;
    virtual bool isInitializeStage(int stage) override {
        return stage == inet::INITSTAGE_NETWORK_CONFIGURATION;
    }
    virtual bool isModuleStartStage(int stage) override {
        return stage == inet::ModuleStartOperation::STAGE_NETWORK_LAYER;
    }
    virtual bool isModuleStopStage(int stage) override {
        return stage == inet::ModuleStopOperation::STAGE_NETWORK_LAYER;
    }
};

}    // namespace veins_proj
