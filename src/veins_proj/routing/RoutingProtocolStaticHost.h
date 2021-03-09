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
 * @file RoutingProtocolHost.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <omnetpp.h>
#include "veins_proj/routing/RoutingProtocolBase.h"
#include "veins_proj/locationservice/HostsLocationTable.h"
#include "veins_proj/mobility/StaticHostMobility.h"
#include "veins_proj/routing/Routing_m.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "inet/networklayer/contract/INetfilter.h"

namespace veins_proj {

/*!
 * @brief Módulo que implementa las operaciones de enrutamiento
 * exclusivamnte para los _hosts_.
 */
class RoutingProtocolStaticHost: public RoutingProtocolBase {

protected:

    /*
     * Contexto.
     */
    /*!
     * @brief Módulo de movilidad.
     */
    StaticHostMobility *mobility = nullptr;
    /*!
     * @brief Módulo de la tabla de ubicación de _hosts_.
     */
    HostsLocationTable *hostsLocationTable = nullptr;

    /*
     * Mensajes propios.
     */
    /*!
     * @brief Temporizador de transmisión de mensajes HOLA_HOST.
     */
    omnetpp::cMessage *helloHostTimer = nullptr;

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

    /*
     * Manejo de mensajes.
     */
    /*!
     * @brief Manejo de mensajes propios.
     *
     * @param message [in] Mensaje a procesar.
     */
    virtual void processSelfMessage(omnetpp::cMessage *message) override;

    /*
     * Mensajes HOLA_VEHIC.
     */
    /*!
     * @brief Procesar mensaje HOLA_VEIC.
     *
     * @param helloCar [in] Mensaje a procesar.
     */
    virtual void processHelloCar(const inet::Ptr<HelloCar> &helloCar) override;

    /*
     * Mensajes HOLA_HOST
     */
    /*!
     * @brief Programar el temporizador de transmisión de mensajes HOLA_HOST.
     */
    virtual void scheduleHelloHostTimer();
    /*!
     * @brief Procesar el temporizador de transmisión de mensajes HOLA_HOST.
     */
    virtual void processHelloHostTimer();
    /*!
     * @brief Crear mensaje HOLA_HOST.
     *
     * @param hostAddress [in] Dirección del _host_ que transmite el mnesaje.
     *
     * @return Mensaje HOLA_HOST.
     */
    virtual const inet::Ptr<HelloHost> createHelloHost(
            const inet::Ipv6Address &hostAddress) const;

    /*
     * Rutas.
     */
    /*!
     * @brief Agregar ruta hacia un destino a la tabla de enrutamiento.
     *
     * Se busca el vehículo vecino más cercano y se selecciona como
     * siguiente salto para la ruta. Si se encuentra el siguiente salto,
     * se crea la ruta y se agrega a la tabla de enrutamiento
     * si esta no existe todavía, o se actualiza si ya existía.
     *
     * @param destAddress [in] Dirección de destino.
     *
     * @return `true` si se creo la ruta o si ya existía.
     */
    virtual bool addRouteToDest(const inet::Ipv6Address &destAddress);

    /*
     * Enrutamiento.
     */
    /*!
     * @brief Enrutar datagrama.
     *
     * Revisa si existe en la tabla de enrutamiento una ruta hacia la
     * dirección de destino. Si no existe, se intenta descubrir y crear una
     * ruta. Si no se encuentra la ruta, se descarta el datagrama.
     *
     * @param datagram [in] Datagrama a enrutar.
     * @param destAddress [in] Dirección IPv6 de destino.
     *
     * @return Resultado del enrutamiento.
     */
    virtual inet::INetfilter::IHook::Result routeDatagram(
            inet::Packet *datagram, const inet::Ipv6Address &destAddress)
                    override;

    /*
     * Netfilter.
     */
    virtual inet::INetfilter::IHook::Result datagramPreRoutingHook(
            inet::Packet *datagram) override;
    virtual inet::INetfilter::IHook::Result datagramLocalOutHook(
            inet::Packet *datagram) override;

    /*
     * Lifecycle.
     */
    virtual void handleStartOperation(inet::LifecycleOperation *operation)
            override;
    virtual void handleStopOperation(inet::LifecycleOperation *operation)
            override;
    virtual void handleCrashOperation(inet::LifecycleOperation *operation)
            override;

    /*
     * Notification.
     */
    virtual void receiveSignal(omnetpp::cComponent *source,
            omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
            cObject *details) override;
};

}
