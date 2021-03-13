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
 * @file RoutingProtocolBase.h
 * @author Adrián Juárez Monroy
 */

#pragma once

#include <omnetpp.h>
#include "inet/common/packet/Packet.h"
#include "inet/common/Ptr.h"
#include "inet/common/TlvOptions_m.h"
#include "inet/networklayer/common/NetworkInterface.h"
#include "inet/networklayer/common/InterfaceTable.h"
#include "inet/networklayer/common/L3Tools.h"
#include "inet/networklayer/contract/INetfilter.h"
#include "inet/networklayer/contract/ipv6/Ipv6Address.h"
#include "inet/networklayer/ipv6/Ipv6ExtensionHeaders_m.h"
#include "inet/networklayer/ipv6/Ipv6Header_m.h"
#include "inet/networklayer/ipv6/Ipv6RoutingTable.h"
#include "inet/routing/base/RoutingProtocolBase.h"
#include "veins_proj/routing/Routing_m.h"
#include "veins_proj/roadnetwork/RoadNetworkDatabase.h"
#include "veins_proj/roadnetwork/RoadNetworkGraph.h"
#include "veins_proj/util/ExpiringValuesMap.h"
#include <string>
#include <map>
#include <utility>

#define ROUTING_PROTOCOL_UDP_PORT                       4096
#define IPV6TLVOPTION_TLV_DEST_GEOHASH_LOCATION         75
#define IPV6TLVOPTION_TLV_DEST_ON_ROAD_NETWORK_LOCATION 76
#define IPV6TLVOPTION_TLV_ROUTING                       77

namespace veins_proj {

/*!
 * @brief Módulo que implementa las operaciones de enrutamiento comunes para
 * vehículos y _hosts_.
 *
 * Contiene los parámetros de enrutamiento para configurar el protocolo.
 * Las unidades de los parámetros se especifican en el archivo de configuración.
 */
class RoutingProtocolBase: public inet::RoutingProtocolBase,
                           public omnetpp::cListener,
                           public inet::NetfilterBase::HookBase {

protected:

    /*
     * Parámetros de configuración.
     */
    //! Intervalo de transmisión de mensajes HOLA_VEHIC.
    omnetpp::simtime_t helloCarInterval;
    //! Intervalo de transmisión de mensajes HOLA_HOST.
    omnetpp::simtime_t helloHostInterval;
    //! Tiempo de vigencia de los registros del de vehículos vecinos.
    omnetpp::simtime_t neighbouringCarValidityTime;
    //! Tiempo de vigencia de los registros del directorio de *hosts* vecinos.
    omnetpp::simtime_t neighbouringHostValidityTime;
    //! Tiempo de espera para los mensahes PONG.
    omnetpp::simtime_t pongTimeout;
    //! Tiempo de vigencia de aristas activas.
    omnetpp::simtime_t edgeStatusValidityTime;
    //! Tiempo de vigencia de las rutas.
    omnetpp::simtime_t routeValidityTime;
    //! Tiempo de vigencia de los datagramas demorados.
    omnetpp::simtime_t delayedDatagramValidityTime;
    //! Radio de proximidad a los vértices.
    double vertexProximityRadius;

    /*
     * Contexto.
     */
    //! Módulo del vehículo o *host* o vehículo contenedor.
    omnetpp::cModule *host = nullptr;
    //! Módulo de la tabla de interfaces.
    inet::IInterfaceTable *interfaceTable = nullptr;
    //! Módulo de la interfaz de red.
    inet::NetworkInterface *networkInterface = nullptr;
    //! Módulo de la tabla de enrutamiento IPv6.
    inet::Ipv6RoutingTable *routingTable = nullptr;
    //! Módulo del protocolo de red.
    inet::INetfilter *networkProtocol = nullptr;
    //! Módulo de la base de datos de redes viales.
    RoadNetworkDatabase *roadNetworkDatabase = nullptr;

    /*
     * Mensajes propios.
     */
    //! Temporizador de limpieza del directorio de vehículos vecinos.
    omnetpp::cMessage *purgeNeighbouringCarsTimer = nullptr;

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
    virtual void handleMessageWhenUp(omnetpp::cMessage *message) override;

    /*
     * Manejo de mensajes.
     */
    /*!
     * @brief Manejo de mensajes.
     *
     * @param message [in] Mensaje a procesar.
     */
    virtual void processMessage(omnetpp::cMessage *message);
    /*!
     * @brief Manejo de mensajes propios.
     *
     * @param message [in] Mensaje a procesar.
     */
    virtual void processSelfMessage(omnetpp::cMessage *message);

    /*
     * Paquetes UDP.
     */
    /*!
     * @brief Enviar paquete UDP.
     *
     * Envía un paquete UDP hacia la compuerta que se conecta con el
     * protocolo IP.
     *
     * @param packet [in] Paque a enviar.
     */
    void sendUdpPacket(inet::Packet *packet);
    /*!
     * @brief Procesar paquete UDP.
     *
     * Identifica el tipo de paquete y lo envía a la función de procesamiento
     * correspondiente.
     *
     * @param packet [in] Paquete a procesar.
     */
    void processUdpPacket(inet::Packet *packet);

    /*
     * Mensajes de enrutamiento.
     */
    /*!
     * @brief Se encapsula el mensaje de enrutamiento en un datagrama UDP
     * y se envía a la dirección indicada.
     *
     * @param routingMessage [in] Mensaje a enviar.
     * @param name [in] Nombre del mensaje.
     * @param srcAddress [in] Dirección de origen del mensaje.
     * @param destAddress [in] Dirección de destino del mensaje.
     */
    void sendRoutingMessage(const inet::Ptr<RoutingPacket> routingMessage,
            const char *name, const inet::Ipv6Address &srcAddress,
            const inet::Ipv6Address &destAddress);

    /*
     * Mensajes ACK.
     */
    /*!
     * @brief Crear mensaje ACK.
     *
     * @param address [in] Dirección del remitente.
     *
     * @return Mensaje ACK.
     */
    virtual const inet::Ptr<Ack> createAck(
            const inet::Ipv6Address &address) const;
    /*!
     * @brief Procesar mensaje ACK.
     *
     * @param ack [in] Mensaje a procesar.
     */
    virtual void processAck(const inet::Ptr<Ack> &ack);

    /*
     * Mensajes HOLA_VEHIC.
     */
    /*!
     * @brief Procesar mensaje HOLA_VEHIC.
     *
     * @param helloCar [in] Mensaje a procesar.
     */
    virtual void processHelloCar(const inet::Ptr<HelloCar> &helloCar) {
    }

    /*
     * Mensajes HOLA_HOST.
     */
    /*!
     * @brief Procesar mensaje HOLA_HOST.
     *
     * @param helloHost [in] Mensaje a procesar.
     */
    virtual void processHelloHost(const inet::Ptr<HelloHost> &helloHost) {
    }

    /*
     * Mensajes PING.
     */
    /*!
     * @brief Procesar mensaje PING.
     *
     * @param ping [in] Mensaje a procesar.
     */
    virtual void processPing(const inet::Ptr<Ping> &ping) {
    }

    /*
     * Mensajes PONG.
     */
    /*!
     * @brief Procesar mensaje PONG.
     *
     * @param pong [in] Mensaje a procesar.
     */
    virtual void processPong(const inet::Ptr<Pong> &pong) {
    }

    /*
     * Directorio de vehículos vecinos.
     */
    /*!
     * @brief Registro de vehículo vecino.
     */
    struct NeighbouringCarValue {
        /*!
         * @brief Ubicación Geohash del vehículo.
         */
        GeohashLocation geohashLocation;
        /*!
         * @brief Velocidad de movimiento del vehículo en metros por segundo.
         */
        double speed;
        /*!
         * @brief Dirección acimutal del movimiento del vehículo en grados.
         */
        double direction;
        /*!
         * @brief Ubicación vial del vehículo.
         */
        LocationOnRoadNetwork locationOnRoadNetwork;
    };
    /*!
     * @brief Diccionario de directorio de vehículos vecinos.
     *
     * La clave es la dirección del vehículo vecino, y el valor es el
     * registro de vehículo vecino.
     * */
    typedef ExpiringValuesMap<inet::Ipv6Address, NeighbouringCarValue> NeighbouringCars;
    /*!
     * @brief Valor.
     */
    typedef NeighbouringCars::MapValue NeighbouringCar;
    /*!
     * @brief Iterador de registros para diccionario del directorio de
     * vehículos vecinos.
     */
    typedef NeighbouringCars::Iterator NeighbouringCarsIterator;
    /*!
     * @brief Iterador de registros para diccionario de directorio de vehículos
     * vecinos constante.
     */
    typedef NeighbouringCars::ConstIterator NeighbouringCarsConstIterator;
    /*!
     * @brief Directorio de vehículos vecinos.
     */
    NeighbouringCars neighbouringCars;
    /*!
     * @brief Imprimir el directorio de vehículos vecinos.
     */
    virtual void showNeighbouringCars() const;
    /*!
     * @brief Programar el temporizador de limpieza del directorio de
     * vehículos vecinos.
     */
    virtual void schedulePurgeNeighbouringCarsTimer();
    /*!
     * @brief Procesar el temporizador de limpieza del directorio de
     * vehículos vecinos.
     */
    virtual void processPurgeNeighbouringCarsTimer();
    /*!
     * @brief Obtener el vehículo vecino más cercano.
     *
     * Encuentra el vehículo vecino cuya ubicación es la más cercana
     * a la ubicación indicada.
     *
     * @param geohashLocation [in] Ubicación Geohash de referencia.
     *
     * @return Dirección IPv6 del vehículo vecino más cercano.
     */
    inet::Ipv6Address findClosestNeighbouringCar(
            const GeohashLocation &geohashLocation) const;

    /*
     * Rutas.
     */
    /*!
     * @brief Mostrar rutas en la tabla de enrutamiento.
     */
    void showRoutes() const;
    /*!
     * @brief Eliminar rutas por dirección IPv6 de siguiente salto.
     *
     * Elimina las rutas de la tabla de enrutamiento cuya dirección IPv6
     * de siguiente salto sea igual a la dirección indicada.
     *
     * @param nextHopAddress [in] Dirección IPv6 de siguiente salto de las
     * rutas a eliminar.
     */
    void purgeNextHopRoutes(const inet::Ipv6Address &nextHopAddress);
    /*!
     * @brief Eliminar rutas viejas.
     *
     * Elimina las rutas de la tabla de enrutamiento cuya hora de expiración
     * sea anterior a la hora indicada.
     *
     * @param expiryTime [in] Hora de expiración máxima para eliminar las rutas.
     */
    void removeOldRoutes(omnetpp::simtime_t expiryTime);

    /*
     * Enrutamiento
     */
    /*!
     * @brief Enrutar datagrama.
     *
     * Revisa si existe en la tabla de enrutamiento una ruta hacia la
     * dirección de destino. Si no existe, se intenta descubrir y crear una
     * ruta. Si no se encuentra la ruta, se descarta el datagrama.
     *
     * @param datagram [in] Datagrama a enrutar.
     *
     * @return Resultado del enrutamiento.
     */
    virtual inet::INetfilter::IHook::Result routeDatagram(
            inet::Packet *datagram) = 0;

    /*
     * Opciones TLV.
     */
    /*!
     * @brief Agregar opción TLV a un datagrama.
     *
     * @param datagram [in] Datagrama al que se le agregará la opción TLV.
     * @param tlvOption [in] Opción TLV a agregar al datagrama.
     */
    void setTlvOption(inet::Packet *datagram,
            inet::TlvOptionBase *tlvOption) const;
    /*!
     * @brief Obtener opción TLV constante de un datagrama.
     *
     * @tparam T Tipo de opción TLV a obtener.
     * @param datagram [in] Datagrama del que se obtendrá la opción TLV.
     *
     * @return Opción TLV.
     */
    template<class T> const T* findTlvOption(inet::Packet *datagram) const {
        const T *tlvOption = nullptr;
        inet::Ptr<const inet::Ipv6Header> ipv6Header = inet::dynamicPtrCast<
                const inet::Ipv6Header>(
                inet::getNetworkProtocolHeader(datagram));
        const inet::Ipv6ExtensionHeader *extensionHeader =
                ipv6Header->findExtensionHeaderByType(
                        inet::IpProtocolId::IP_PROT_IPv6EXT_HOP);
        const inet::Ipv6HopByHopOptionsHeader *optionsHeader =
                omnetpp::check_and_cast_nullable<
                        const inet::Ipv6HopByHopOptionsHeader*>(
                        extensionHeader);

        if (optionsHeader) {
            const inet::TlvOptions &tlvOptions = optionsHeader->getTlvOptions();

            int i = 0;
            while (i < tlvOptions.getTlvOptionArraySize()) {
                tlvOption =
                        dynamic_cast<const T*>(tlvOptions.getTlvOption(i++));

                if (tlvOption != nullptr)
                    break;
            }
        }

        return tlvOption;
    }
    /*!
     * @brief Obtener opción TLV de un datagrama.
     *
     * @tparam T Tipo de opción TLV a obtener.
     * @param datagram [in] Datagrama del que se obtendrá la opción TLV.
     *
     * @return Opción TLV.
     */
    template<class T> T* findTlvOptionForUpdate(inet::Packet *datagram) {
        T *tlvOption = nullptr;
        inet::Ptr<inet::Ipv6Header> ipv6Header = inet::constPtrCast<
                inet::Ipv6Header>(
                inet::dynamicPtrCast<const inet::Ipv6Header>(
                        inet::getNetworkProtocolHeader(datagram)));
        inet::Ipv6ExtensionHeader *extensionHeader =
                ipv6Header->findExtensionHeaderByTypeForUpdate(
                        inet::IpProtocolId::IP_PROT_IPv6EXT_HOP);
        inet::Ipv6HopByHopOptionsHeader *optionsHeader =
                omnetpp::check_and_cast_nullable<
                        inet::Ipv6HopByHopOptionsHeader*>(extensionHeader);

        if (optionsHeader) {
            inet::TlvOptions &tlvOptions =
                    optionsHeader->getTlvOptionsForUpdate();

            int i = 0;
            while (i < tlvOptions.getTlvOptionArraySize()) {
                tlvOption = dynamic_cast<T*>(tlvOptions.getTlvOptionForUpdate(
                        i++));

                if (tlvOption != nullptr)
                    break;
            }
        }

        return tlvOption;
    }
    /*!
     * @brief Crear opción TLV de ubicación del destino.
     *
     * @param geohashLocationBits [in] Ubicación Geohash del destino.
     *
     * @return Opción TLV de ubicación del destino.
     */
    TlvDestGeohashLocationOption* createTlvDestGeohashLocationOption(
            uint64_t geohashLocationBits) const;
    /*!
     * @brief Calcular la longitud en octetos de una opción TLV de ubicación
     * del destino.
     *
     * @param tlvOption [in] Opción TLV cuya longitud se calcula.
     *
     * @return Longitud de la opción TLV.
     */
    int computeTlvOptionLength(TlvDestGeohashLocationOption *tlvOption) const;
    /*!
     * @brief Crear opción TLV de ubicación vial del destino.
     *
     * @param vertexA [in] Primer vértice de la arista de la ubicación vial.
     * @param vertexB [in] Segundo vértice de la arista de la ubicación vial.
     * @param distanceToVertexA [in] Distancia al primer vértice.
     *
     * @return Opción TLV de ubicación vial del destino.
     */
    TlvDestLocationOnRoadNetworkOption* createTlvDestLocationOnRoadNetworkOption(
            Vertex vertexA, Vertex vertexB, double distanceToVertexA) const;
    /*!
     * @brief Agregar opción TLV de ubicación vial del destino a un datagrama.
     *
     * @param datagram [inout] Datagrama al que se le agregará la opción TLV.
     * destGeohashLocation tlvOption [in] Ubicación Geohash del destino.
     */
    void setTlvDestLocationOnRoadNetworkOption(inet::Packet *datagram,
            const GeohashLocation &destGeohashLocation) const;
    /*!
     * @brief Calcular la longitud en octetos de una opción TLV de
     * ubicación vial del destino.
     *
     * @param tlvOption [in] Opción TLV cuya longitud se calcula.
     *
     * @return Longitud de la opción TLV.
     */
    int computeTlvOptionLength(
            TlvDestLocationOnRoadNetworkOption *tlvOption) const;
    /*!
     * @brief Crear opción TLV de vértices visitados vacía.
     *
     * @return Opción TLV de vértices visitados.
     */
    TlvVisitedVerticesOption* createTlvVisitedVerticesOption() const;
    /*!
     * @brief Calcular la longitud en octetos de una opción TLV de
     * vértices visitados.
     *
     * @param tlvOption [in] Opción TLV cuya longitud se calcula.
     *
     * @return Longitud de la opción TLV.
     */
    int computeTlvOptionLength(TlvVisitedVerticesOption *tlvOption) const;

    /*
     * Netfilter.
     */
    /*!
     * @brief Procesar datagrama recibido de la capa inferior
     * antes de enrutarlo.
     *
     * @param datagram [in] Datagrama a procesar.
     *
     * @return Resultado del procesamiento.
     */
    virtual inet::INetfilter::IHook::Result datagramPreRoutingHook(
            inet::Packet *datagram) override {
        return inet::INetfilter::IHook::ACCEPT;
    }
    virtual inet::INetfilter::IHook::Result datagramForwardHook(
            inet::Packet *datagram) override {
        return inet::INetfilter::IHook::ACCEPT;
    }
    virtual inet::INetfilter::IHook::Result datagramPostRoutingHook(
            inet::Packet *datagram) override {
        return inet::INetfilter::IHook::ACCEPT;
    }
    virtual inet::INetfilter::IHook::Result datagramLocalInHook(
            inet::Packet *datagram) override {
        return inet::INetfilter::IHook::ACCEPT;
    }
    /*!
     * @brief Procesar datagrama recibido de la capa superior
     * antes de enrutarlo.
     *
     * @param datagram [in] Datagrama a procesar.
     *
     * @return Resultado del procesamiento.
     */
    virtual inet::INetfilter::IHook::Result datagramLocalOutHook(
            inet::Packet *datagram) override {
        return inet::INetfilter::IHook::ACCEPT;
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

    /*
     * Notification.
     */
    virtual void receiveSignal(omnetpp::cComponent *source,
            omnetpp::simsignal_t signalID, omnetpp::cObject *obj,
            cObject *details) override = 0;
};

}    // namespace veins_proj
